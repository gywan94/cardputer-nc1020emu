/*
 * cardputer-nc1020 — wqx NC1020 emulator on M5Cardputer (ESP32-S3), ESP-IDF.
 *
 * Milestone 2: bring up the hardware (ST7789 LCD, SD, GPIO keyboard) and prove
 * the drivers work — fill the screen, mount SD, and echo pressed key codes.
 * The emulator core is configured + linked but NOT run yet: the StampS3 has no
 * PSRAM, so LoadNC2k()'s 12 MB ROM alloc fails until rom.cpp/nor.cpp are made
 * SD-paged (Milestone 3). Flip RUN_EMULATOR to 1 once M3 lands.
 */
#include <string>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_system.h"    /* esp_restart() for the Fn+Del reset combo */
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"   /* Core0->Core1 frame queue (no shared 6502 RAM) */

#include "cardputer_bsp.h"
#include "comm.h"
#include "nc2000.h"
#include "key_new.h"
#include "state.h"
#include "sd_paging.h"
#include "flash_rom.h"
#include "font.h"          /* font8x16 — for the boot ROM-select menu */
#include "nvs.h"
#include "nvs_flash.h"
#include <dirent.h>
#include <sys/stat.h>
#include <strings.h>
#include <cstring>

/* Milestone 3: ROM/NOR are SD-paged (no PSRAM on Cardputer), so the core can
 * boot. Flip to 0 to fall back to the M2 keyboard self-test. */
#define RUN_EMULATOR 1

static const char *TAG = "cardputer-nc1020";

extern WqxRom nc2k_rom;
extern nc2k_states_t nc2k_states;
extern uint8_t nor_info_block[0x100];
extern uint32_t cpu_batch;
void SetKeyWayback(int code_y, int code_x, bool down);
void cold_reset();   /* NC1020 firmware reset (cpu_loop_new.cpp) — file scope so it
                      * keeps C++ linkage; declaring it inside extern "C" app_main
                      * gave it C linkage and an unmangled undefined reference. */
void warm_reset_if_clkoff();

#define LCD_W 160
#define LCD_H 80
static uint8_t  lcd_buf[LCD_W * LCD_H / 8 * 2];

/* Cardputer physical key (x,y) -> NC1020/wqx key code. Layout matches the printed
 * Cardputer keys (computer-like): the top row types digits 1-0, the letter rows
 * are QWERTY, Enter/Space/arrows are themselves. Hold Fn (the Alt key, code 0x00)
 * to turn the top row into the NC1020 app keys (英汉/名片/计算/...) — see extend_kb_map.
 *   y0: `  1    2    3    4    5    6    7    8    9    0    -    =   del
 *   y1: tab q w e r t y u i o p [ ] \   (letters)
 *   y2: fn shift a s d f g h j k l ; ' enter
 *   y3: ctrl opt alt z x c v b n m , . / space                                   */
#define KEY_SIZE 0x40
static const uint8_t kb_map[4][14] = {
    {0x3B, 0x34, 0x35, 0x36, 0x2c, 0x2d, 0x2e, 0x24, 0x25, 0x26, 0x3c, 0x1a, 0x1d, 0x11},
    {0x00, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x18, 0x1c, 0x37, 0x1e, 0x13},
    {0x00, 0x3a, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x19, 0x1a, 0x3d, 0x1d},
    {0x38, 0x3c, 0x39, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x3f, 0x1b, 0x1f, 0x3e}};
/* Fn + top row -> the NC1020 function keys F1..F11, in order, so the keyboard can
 * switch functions/apps:
 *   Fn+1..4 = F1 F2 F3 F4  (插入 删除 查找 修改)
 *   Fn+5..0 = F5..F10       (英汉 名片 计算 行程 资料 时间)
 *   Fn+-    = F11 (网络) ; Fn+= 报时 ; Fn+del 发音 */
static const uint8_t extend_kb_map[14] = {
    0x3B, 0x10, 0x11, 0x12, 0x13, 0x0b, 0x0c, 0x0d, 0x0a, 0x09, 0x08, 0x0e, 0x14, 0x15};
static bool kb_state[KEY_SIZE] = {0};

static void wqx_set_key(uint8_t code, bool down)
{
    if (code == 0x0f) {
        uint8_t *ram_io = nc2k_states.ram_io;
        if (down) { ram_io[0x0b] &= (uint8_t)~1; warm_reset_if_clkoff(); }
        else      { ram_io[0x0b] |= 1; }
        return;
    }
    SetKeyWayback(code % 8, code / 8, down);
}

#define KBD_DEBUG 0   /* logs raw (x,y) of each pressed key + Fn(ex) state */
static void poll_keyboard(bool to_core)
{
    cp_key_t keys[16];
    int n = cp_kbd_scan(keys, 16);

    bool want[KEY_SIZE] = {0};
    bool ex = false;
    for (int i = 0; i < n; i++)
        if (keys[i].y < 4 && keys[i].x < 14 && kb_map[keys[i].y][keys[i].x] == 0x00) { ex = true; break; }
#if KBD_DEBUG
    for (int i = 0; i < n; i++)
        ESP_LOGI(TAG, "rawkey x=%d y=%d -> code=0x%02x  Fn(ex)=%d",
                 keys[i].x, keys[i].y,
                 (keys[i].y < 4 && keys[i].x < 14) ? kb_map[keys[i].y][keys[i].x] : 0xff, ex);
#endif
    for (int i = 0; i < n; i++) {
        if (keys[i].y >= 4 || keys[i].x >= 14) continue;
        uint8_t code = (ex && keys[i].y == 0) ? extend_kb_map[keys[i].x] : kb_map[keys[i].y][keys[i].x];
        if (code != 0x00 && code < KEY_SIZE) want[code] = true;
    }
    for (int i = 0; i < KEY_SIZE; i++) {
        if (want[i] && !kb_state[i]) { ESP_LOGI(TAG, "KEY %02x down", i); if (to_core) wqx_set_key(i, true); }
        else if (!want[i] && kb_state[i]) { if (to_core) wqx_set_key(i, false); }
        kb_state[i] = want[i];
    }
}

#define NAT_W 135          /* panel native width  (= landscape height) */
#define NAT_H 240          /* panel native height (= landscape width)  */

/* Fill the whole NATIVE panel with one rgb565 color. */
static void fill_screen(uint16_t c)
{
    static uint16_t row[NAT_W];
    for (int i = 0; i < NAT_W; i++) row[i] = c;
    for (int ny = 0; ny < NAT_H; ny++) cp_lcd_draw(0, ny, NAT_W, ny + 1, row);
}

/* ── Boot-UI framebuffer (1bpp logical 240x135 landscape, bit=1 -> white) ─────
 * The boot UI draws into this in intuitive landscape coords, then ui_flush()
 * blits it with the SAME native-ROW loop as draw_lcd (the path proven on
 * hardware). Heap-allocated on first use (ui_clear) and freed (ui_free) before
 * the emulator loop, so it costs 0 during gameplay and only ~4KB while a menu is
 * on screen — internal RAM is precious (the FAT/SD mount needs it). */
#define UIFB_SZ (CP_LCD_W * CP_LCD_H / 8)
static uint8_t *s_uifb = NULL;
static void ui_clear(void)
{
    if (!s_uifb) s_uifb = (uint8_t *)malloc(UIFB_SZ);
    if (s_uifb)  memset(s_uifb, 0, UIFB_SZ);
}
static void ui_free(void) { if (s_uifb) { free(s_uifb); s_uifb = NULL; } }
static inline void ui_pixel(int lx, int ly, bool on)
{
    if (!s_uifb || (unsigned)lx >= CP_LCD_W || (unsigned)ly >= CP_LCD_H) return;
    int i = ly * CP_LCD_W + lx;
    if (on) s_uifb[i >> 3] |=  (0x80 >> (i & 7));
    else    s_uifb[i >> 3] &= ~(0x80 >> (i & 7));
}
static void ui_flush(void)
{
    if (!s_uifb) return;
    static uint16_t line[NAT_W];
    for (int ny = 0; ny < NAT_H; ny++) {            /* native row = landscape x (lx) */
        int lx = ny;
        for (int nx = 0; nx < NAT_W; nx++) {        /* native col -> landscape y      */
            int ly = NAT_W - 1 - nx;
            int i = ly * CP_LCD_W + lx;
            bool on = (s_uifb[i >> 3] >> (7 - (i & 7))) & 1;
            line[nx] = on ? 0xFFFF : 0x0000;
        }
        cp_lcd_draw(0, ny, NAT_W, ny + 1, line);
    }
}

/* One-time ROM->flash copy progress bar (called from flash_rom_prepare). */
static void menu_text(int lx, int ly, const char *s, bool sel);   /* fwd decl */
extern "C" void rom_progress(int pct)
{
    static int last = -1;
    if (pct == last) return;
    last = pct;
    ui_clear();
    menu_text(20, 30, "Loading ROM to flash...", false);
    const int bx = 20, bw = CP_LCD_W - 40, by = 58, bh = 16;
    const int fill = bx + bw * pct / 100;
    for (int y = by; y < by + bh; y++)
        for (int x = bx; x < bx + bw; x++) {
            bool border = (y == by || y == by + bh - 1 || x == bx || x == bx + bw - 1);
            bool filled = (x > bx && x < fill && y > by && y < by + bh - 1);
            ui_pixel(x, y, border || filled);
        }
    ui_flush();
}

#define RGB565(r,g,b) ((uint16_t)((((r)&0xF8)<<8)|(((g)&0xFC)<<3)|((b)>>3)))
#define LCD_SRC_W 160
#define LCD_SRC_H 80
bool is_grey_mode();
/* The NC1020 LCD is exactly 160x80 (comm.h SCREEN_WIDTH/HEIGHT; nc2000.cpp copies
 * 1600 bytes = 160*80 1bpp). There is NO separate icon buffer in nc1020 mode — the
 * status segments (英/汉/CAPS/battery) live within the same 160x80 bitmap — so we
 * draw all 160 columns and lose nothing. Layout: 20 bytes/row, MSB-first horizontal.
 * Drawn 1:1 (one NC1020 pixel = one panel pixel), centred, rotated 180deg. 1:1
 * (integer scale) is the only way horizontal lines stay perfectly level: any
 * fractional stretch to fill 240x135 lands some source rows on 1 panel row and some
 * on 2, making lines ragged. 1bpp MSB-first (1=dark) / 2bpp grey. */
/* Draw only the native rectangle [nx0,nx1) x [ny0,ny1). NATIVE 135x240 portrait
 * frame; landscape (lx=ny, ly=nx) nearest-neighbour scale of the 160x80 image,
 * rotated in software. Redrawing only the changed rect (not all 240 rows every
 * time) kills the once-a-second full-screen rewrite that read as flicker. */
static void draw_lcd_rect(const uint8_t *fb, bool grey, int nx0, int ny0, int nx1, int ny1)
{
    static const uint16_t lvl[4] = {
        RGB565(245,245,245), RGB565(180,180,180), RGB565(105,105,105), RGB565(0,0,0) };
    static uint16_t line[NAT_W];
    /* `grey` is captured WITH the frame on Core 0 (not re-read here), so the decode
     * always matches the bytes that were copied — no grey/1bpp mismatch on a mode
     * switch, and Core 1 never reads emulator state. */
    if (nx0 < 0) nx0 = 0;
    if (ny0 < 0) ny0 = 0;
    if (nx1 > NAT_W) nx1 = NAT_W;
    if (ny1 > NAT_H) ny1 = NAT_H;
    if (nx0 >= nx1 || ny0 >= ny1) return;
    for (int ny = ny0; ny < ny1; ny++) {
        int sx = ny * LCD_SRC_W / NAT_H;                 /* source col 0..159 */
        for (int nx = nx0; nx < nx1; nx++) {
            int ly = NAT_W - 1 - nx;
            int sy = ly * LCD_SRC_H / NAT_W;             /* source row 0..79 */
            int pos = sy * LCD_SRC_W + sx, val;
            if (!grey) val = ((fb[pos >> 3] >> (7 - (pos & 7))) & 1) ? 3 : 0;
            else       val = (fb[pos >> 2] >> (6 - (pos & 3) * 2)) & 3;
            line[nx - nx0] = lvl[val];
        }
        cp_lcd_draw(nx0, ny, nx1, ny + 1, line);
    }
}
static inline void draw_lcd(const uint8_t *fb, bool grey) { draw_lcd_rect(fb, grey, 0, 0, NAT_W, NAT_H); }

/* Dump the 160x80 frame to serial as 80x40 ASCII art so the display content can
 * be verified without looking at the panel. ' '=white .=light :=dark #=black. */
static void dump_lcd_ascii(const uint8_t *fb)
{
    const bool grey = is_grey_mode();
    static const char gch[4] = {' ', '.', ':', '#'};
    printf("==== LCD 160x80 (%s) ====\n", grey ? "grey" : "1bpp");
    for (int cy = 0; cy < 40; cy++) {
        char row[81];
        int sr = cy * 2;
        for (int cx = 0; cx < 80; cx++) {
            int sc = cx * 2, pos = sr * 160 + sc, val;
            if (!grey) val = ((fb[pos >> 3] >> (7 - (pos & 7))) & 1) ? 3 : 0;
            else       val = (fb[pos >> 2] >> (6 - (pos & 3) * 2)) & 3;
            row[cx] = gch[val];
        }
        row[80] = 0;
        printf("|%s|\n", row);
    }
}

/* ───────────────────────── Boot ROM-select menu ─────────────────────────
 * Hold G0 at boot to pick a ROM from the SD card. Renders text with font8x16,
 * 180deg-flipped to match the panel. Keys: ; = up, . = down, Enter = select. */
typedef struct { char basepath[80]; char name[48]; } RomEntry;

/* Draw an 8x16 text line at logical landscape (lx,ly) into the boot-UI framebuffer
 * (ui_flush blits it). sel=true = white highlight bar with black text. */
static void menu_text(int lx, int ly, const char *s, bool sel)
{
    for (int cr = 0; cr < 16; cr++) {
        int gy = ly + cr;
        if (gy < 0 || gy >= CP_LCD_H) continue;
        if (sel) for (int x = 0; x < CP_LCD_W; x++) ui_pixel(x, gy, true);   /* white bar */
        for (int ci = 0; s[ci]; ci++) {
            int cx = lx + ci * 8;
            if (cx + 8 > CP_LCD_W) break;
            uint8_t bits = font8x16[(uint8_t)s[ci] * 16 + cr];
            for (int b = 0; b < 8; b++)
                if (bits & (0x80 >> b)) ui_pixel(cx + b, gy, !sel);  /* sel: black text on bar; else white text */
        }
    }
}

static int scan_roms(RomEntry *out, int maxn)
{
    const char *dirs[] = { "/sd", "/sd/roms" };
    int n = 0;
    for (int d = 0; d < 2 && n < maxn; d++) {
        DIR *dir = opendir(dirs[d]);
        if (!dir) continue;
        struct dirent *e;
        while ((e = readdir(dir)) && n < maxn) {
            int len = (int)strlen(e->d_name);
            if (len < 5 || strcasecmp(e->d_name + len - 4, ".rom") != 0) continue;
            char full[96];
            snprintf(full, sizeof full, "%s/%s", dirs[d], e->d_name);
            struct stat st;
            if (stat(full, &st) != 0 || st.st_size != 12 * 1024 * 1024) continue;  /* 12 MB only */
            snprintf(out[n].basepath, sizeof out[n].basepath, "%s/%.*s", dirs[d], len - 4, e->d_name);
            snprintf(out[n].name, sizeof out[n].name, "%.*s", len - 4, e->d_name);
            n++;
        }
        closedir(dir);
    }
    return n;
}

/* Block until a new key is pressed; return its NC1020 code. */
static uint8_t menu_wait_key(void)
{
    static bool prev[KEY_SIZE] = {0};
    for (;;) {
        cp_key_t keys[16];
        int n = cp_kbd_scan(keys, 16);
        bool cur[KEY_SIZE] = {0};
        for (int i = 0; i < n; i++)
            if (keys[i].y < 4 && keys[i].x < 14) {
                uint8_t c = kb_map[keys[i].y][keys[i].x];
                if (c && c < KEY_SIZE) cur[c] = true;
            }
        uint8_t pressed = 0;
        for (int c = 0; c < KEY_SIZE; c++) if (cur[c] && !prev[c]) pressed = c;
        memcpy(prev, cur, sizeof prev);
        if (pressed) return pressed;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* Decide which ROM base path to boot. G0 held -> show the menu; otherwise use the
 * last-selected ROM (NVS). Returns false only if nothing usable is found.
 * *changed is set if the chosen ROM differs from the last one (forces a reload). */
static bool choose_rom(char *outBase, int sz, bool *changed)
{
    esp_err_t nv = nvs_flash_init();
    if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }

    char cur[80] = "";
    nvs_handle_t h;
    if (nvs_open("nc1020", NVS_READONLY, &h) == ESP_OK) {
        size_t l = sizeof cur; nvs_get_str(h, "cur_rom", cur, &l); nvs_close(h);
    }
    bool g0_held = (gpio_get_level(GPIO_NUM_0) == 0);

    /* Normal boot (G0 not held): reuse the last-selected ROM, or fall back to the
     * baked/default ROM if none was ever chosen. The menu only appears with G0. */
    if (!g0_held) {
        if (cur[0]) { snprintf(outBase, sz, "%s", cur); *changed = false; return true; }
        return false;
    }

    /* G0 held: scan + show the ROM menu. */
    static RomEntry roms[24];
    int n = scan_roms(roms, 24);
    ESP_LOGW(TAG, "choose_rom: G0 held — scan_roms found %d 12MB .rom file(s)", n);
    if (n == 0) {
        if (cur[0]) { snprintf(outBase, sz, "%s", cur); *changed = false; return true; }
        return false;
    }
    int sel = 0;
    for (int i = 0; i < n; i++) if (strcmp(roms[i].basepath, cur) == 0) sel = i;

    const int VIS = 6;                              /* visible rows */
    for (;;) {
        int top = sel - VIS / 2;
        if (top > n - VIS) top = n - VIS;
        if (top < 0) top = 0;
        ui_clear();
        menu_text(2, 0, "Select ROM (hold G0):", false);
        for (int i = 0; i < VIS && top + i < n; i++)
            menu_text(6, 18 + i * 16, roms[top + i].name, (top + i) == sel);
        menu_text(2, 119, "; up  . down  Enter  Esc", false);
        ui_flush();
        uint8_t k = menu_wait_key();
        if      (k == 0x1a) sel = (sel - 1 + n) % n;          /* ; = up   */
        else if (k == 0x1b) sel = (sel + 1) % n;              /* . = down */
        else if (k == 0x1d || k == 0x3e) {                    /* Enter/Space = select */
            snprintf(outBase, sz, "%s", roms[sel].basepath);
            *changed = (strcmp(outBase, cur) != 0);
            if (nvs_open("nc1020", NVS_READWRITE, &h) == ESP_OK) { nvs_set_str(h, "cur_rom", outBase); nvs_commit(h); nvs_close(h); }
            return true;
        }
        else if (k == 0x3b) {                                 /* Esc = cancel */
            if (cur[0]) { snprintf(outBase, sz, "%s", cur); *changed = false; return true; }
            return false;
        }
    }
}

static void wqx_configure(const char *base)
{
    nc1020mode = true;
    cpu_loop_version = CPU_RUN3;
    io_version       = IO_V2;
    nc2k_rom = WqxRom();
    nc2k_rom.romPath      = std::string(base) + ".rom";
    nc2k_rom.norFlashPath = std::string(base) + ".nor";
    nc2k_rom.statesPath   = std::string(base) + ".state";
    nor_info_block[8] = 0xfc; nor_info_block[9] = 0x03;
    save_flash_on_exit = true;
    /* Persist the working RAM across reboots so the NC1020 doesn't cold-boot (and
     * ask to format the system) every reset. The state is loaded on boot and saved
     * periodically (see the main loop). First boot has no .state -> one format. */
    enable_load_state = true;
    /* Resume from the SAVED PC (no cpu->reset). A reset would warm-boot and re-init
     * the firmware UI back to the home/"出厂" screen, losing the session (the user
     * saw "a flash of the saved screen, then factory"). Instead load_state clears the
     * clk-off bits (ram_io[0x05]) so the firmware walks out of its saved standby and
     * repaints the saved session in place. set_warm_reset_flag() stays implemented
     * for genuine ON-key warm resets. */
    reset_after_load_state = false;
    init_parameters();
    /* No real-time-clock chip and no network on the Cardputer, so time(NULL) is a
     * meaningless counter (~1970 + uptime). Auto-sync would overwrite both the
     * user-set time AND the time restored from the save with that garbage on every
     * boot. Disable it; seed_initial_rtc_time() (called from app_main AFTER
     * LoadNC2k) writes a one-shot build-time initial clock on a true cold boot
     * (or if a stale .state was read back with ext_reg[0..2]==0). The user can
     * correct it once via the NC1020 menu, and it then persists via the saved
     * state and advances with the emulator. */
    enable_auto_time_sync = false;
    /* The 6502 interpreter runs at ~76% realtime, so the NC1020 RTC (driven by
     * emulated cycles) ran 24% slow and the clock drifted behind. Scale the RTC
     * timebase by 1/0.76 ≈ 1.32 so it ticks at real rate — set the time once in the
     * NC1020 menu and it then tracks the wall clock (approx; drifts a little under
     * heavy load when emulation dips below 76%). */
    rtc_speed = 1.32;
    cpu_batch = 4096;
    init_keyitems();
}

/* Seed NC1020 RTC from the build timestamp on a true cold boot. The Cardputer has
 * no RTC chip and no NTP, so time(NULL) is meaningless — we want the first cold
 * boot to show a clock that's at least in the right year, not 1881 / 00:00:00.
 * The user can correct it once via the NC1020 menu ("时间" -> 设定时间), and
 * the state file then preserves that user-set time across reboots.
 *
 * We write the same fields that sync_time_1020() does (RAM 0x472-0x474 for
 * year/month/day, ext_reg[0..2] for sec/min/hour).
 *
 * IMPORTANT: write nc2k_states.ram[] directly, NOT via Store(). Store() walks
 * the memmap[] page table, and we are called BEFORE init_mem() so memmap[0]
 * is uninitialised — calling Store(0x472,...) here triggers a StoreProhibited
 * Guru Meditation and a reboot loop. Writing the array field directly is safe
 * because the very first thing the CPU does is read RAM 0x000-0x1FFF.
 *
 * Also: only seed if ext_reg[0..2] is all zero (true cold boot or stale save);
 * any non-zero value means the user (or the previous session) has set the clock
 * and we must NOT clobber it. Must run AFTER LoadNC2k so its load_state() read
 * has already populated ext_reg[] from the .state file. */
static void seed_initial_rtc_time(void)
{
    if (nc2k_states.ext_reg[0] != 0 ||
        nc2k_states.ext_reg[1] != 0 ||
        nc2k_states.ext_reg[2] != 0) {
        ESP_LOGI(TAG, "RTC already set (hh=%02d mm=%02d ss=%02d) — not seeding",
                 nc2k_states.ext_reg[2], nc2k_states.ext_reg[1], nc2k_states.ext_reg[0]);
        return;
    }
    /* __DATE__ is "Mmm DD YYYY" (DD may have leading space, e.g. "Jun  3 2026").
     * Parse it; fall back to 2026/01/01 00:00:00 if anything looks off. */
    static const char mon_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    int y = 2026, m = 1, d = 1, hh = 0, mm = 0, ss = 0;
    const char *ds = __DATE__;
    const char *ts = __TIME__;
    char month_str[4] = {0};
    if (ds && strlen(ds) >= 11) {
        memcpy(month_str, ds, 3);
        const char *p = strstr(mon_names, month_str);
        if (p) m = (int)((p - mon_names) / 3) + 1;
        d = atoi(ds + 4);
        y = atoi(ds + 7);
    }
    if (ts && strlen(ts) >= 8) {
        hh = atoi(ts);
        mm = atoi(ts + 3);
        ss = atoi(ts + 6);
    }
    /* Clamp to the NC1020 firmware's range: year byte 0x7A..0x9C = 2003..2031.
     * Older/year>2031 firmware mis-decodes and shows garbage. */
    if (y < 2003) y = 2003;
    if (y > 2031) y = 2031;
    /* Write the firmware-visible fields the same way sync_time_1020() does:
     *   RAM 0x472 = (year - 1900 - 103) + 0x7A  -> 2003..2031
     *   RAM 0x473 = month 0..11
     *   RAM 0x474 = day-1  0..30
     *   ext_reg[0..2] = sec, min, hour
     * See comment above on why we write the array directly, not via Store(). */
    nc2k_states.ram[0x472] = (uint8_t)((y - 1900 - 103) + 0x7A);
    nc2k_states.ram[0x473] = (uint8_t)(m - 1);
    nc2k_states.ram[0x474] = (uint8_t)(d - 1);
    nc2k_states.ext_reg[0] = (uint8_t)ss;
    nc2k_states.ext_reg[1] = (uint8_t)mm;
    nc2k_states.ext_reg[2] = (uint8_t)hh;
    ESP_LOGI(TAG, "seeded initial RTC = %04d-%02d-%02d %02d:%02d:%02d (0x472=0x%02x)",
             y, m, d, hh, mm, ss, (unsigned)(y - 1900 - 103 + 0x7A));
}

/* Full-panel geometry test: paints the whole logical 240x135 so we can see exactly
 * how it lands on the physical panel (rotation / mirror / gap / any wrap-split).
 * Corners get distinct colours, plus a 1px border and a centre cross. */
#define PANEL_TEST 0
#define PT_W NAT_W   /* native portrait panel width  */
#define PT_H NAT_H   /* native portrait panel height */
static void panel_test(void)
{
    const uint16_t WHITE=0xFFFF, RED=0xF800, GREEN=0x07E0, BLUE=0x001F, BLACK=0x0000;
    static uint16_t row[PT_W];
    for (int y = 0; y < PT_H; y++) {
        for (int x = 0; x < PT_W; x++) {
            uint16_t c = BLACK;
            /* native-portrait probe (no swap_xy): RED=TOP edge, BLUE=LEFT edge,
             * white diagonal TL->BR, GREEN centre dot, white border. If the box is
             * square/axis-aligned (not sheared) the addressing is right. */
            if (y < 12)                         c = RED;     /* top edge  */
            if (x < 12)                         c = BLUE;    /* left edge */
            if (x * (PT_H - 1) / (PT_W - 1) == y) c = WHITE; /* diagonal  */
            int cx = x - PT_W/2, cy = y - PT_H/2;
            if (cx > -12 && cx < 12 && cy > -12 && cy < 12) c = GREEN;
            if (x == 0 || x == PT_W-1 || y == 0 || y == PT_H-1) c = WHITE; /* border */
            row[x] = c;
        }
        cp_lcd_draw(0, y, PT_W, y + 1, row);
    }
    for (;;) vTaskDelay(pdMS_TO_TICKS(200));
}

/* ── 6502-JIT feasibility probe ──────────────────────────────────────────────
 * The interpreter runs at ~185 real Xtensa cycles per 6502 instruction (76%%
 * realtime). A JIT would eliminate the DISPATCH (opcode fetch + 256-way switch +
 * PC advance + register reload + loop) but NOT the irreducible MEM+ALU "floor".
 * Measure both on real hardware to estimate the achievable JIT speedup.
 * Must run AFTER LoadNC2k() (memmap must be valid). */
#define JIT_PROBE 0
#if JIT_PROBE
static void bench_jit_feasibility(void) {
    extern uint8_t* memmap[8];
    const int N = 4000000;
    volatile uint32_t sink = 0;

    /* (1) irreducible FLOOR: a representative op's work — 2 operand reads + ALU +
     * NZ flags + a data access, via the real page-table memmap (real cache behaviour).
     * A JIT still pays this. */
    {
        uint16_t a = 0x8000; uint8_t acc = 0, nz = 0; uint32_t s = 0;
        int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < N; i++) {
            uint8_t v0 = memmap[a >> 13][a & 0x1FFF];
            uint8_t v1 = memmap[(a + 1) >> 13][(a + 1) & 0x1FFF];
            acc = (uint8_t)(acc + v0 + v1);
            nz = (uint8_t)((acc == 0 ? 2 : 0) | (acc & 0x80));   /* N/Z flags */
            s += memmap[(a + acc) >> 13][(a + acc) & 0x1FFF] + nz;
            a += 5; if (a > 0xfc00) a = 0x8000;
        }
        int64_t dt = esp_timer_get_time() - t0; sink += s;
        ESP_LOGW(TAG, "[JIT probe] FLOOR (mem+ALU+flags, no dispatch): %.0f real-cyc/op",
                 (double)dt * 1000.0 / N * 240.0 / 1000.0);
    }
    /* (1b) FLOOR but flags written to GLOBAL memory (like the interpreter's mN/mZ in
     * nc2k_states) instead of a local — isolates the cost of global flag state. */
    {
        static volatile uint8_t gN, gZ, gC;   /* stand-in for the interpreter's globals */
        uint16_t a = 0x8000; uint8_t acc = 0; uint32_t s = 0;
        int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < N; i++) {
            uint8_t v0 = memmap[a >> 13][a & 0x1FFF];
            uint8_t v1 = memmap[(a + 1) >> 13][(a + 1) & 0x1FFF];
            acc = (uint8_t)(acc + v0 + v1);
            gZ = (acc == 0); gN = (acc & 0x80) ? 1 : 0; gC = (v0 + v1) > 255;  /* global flags */
            s += memmap[(a + acc) >> 13][(a + acc) & 0x1FFF];
            a += 5; if (a > 0xfc00) a = 0x8000;
        }
        int64_t dt = esp_timer_get_time() - t0; sink += s + gN + gZ + gC;
        ESP_LOGW(TAG, "[JIT probe] FLOOR + GLOBAL flags: %.0f real-cyc/op",
                 (double)dt * 1000.0 / N * 240.0 / 1000.0);
    }
    /* (2) DISPATCH overhead: opcode fetch + a dense 256-entry function-pointer dispatch
     * (like switch(mOpcode)) + PC advance. A JIT eliminates this entirely. */
    {
        static uint8_t prog[8192];
        for (int i = 0; i < 8192; i++) prog[i] = (uint8_t)(i * 7 + 3);   /* pseudo opcodes */
        uint16_t pc = 0; uint32_t s = 0;
        int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < N; i++) {
            uint8_t op = prog[pc & 0x1FFF];
            switch (op) {
                case 0xA5: case 0xA9: s += 1; break;
                case 0x85: case 0x8D: s += 2; break;
                case 0xE8: case 0xC8: s += 3; break;
                case 0xD0: case 0xF0: s += 4; break;
                case 0x4C: case 0x6C: s += 5; break;
                case 0x20: case 0x60: s += 6; break;
                default:               s += op; break;
            }
            pc += 1;
        }
        int64_t dt = esp_timer_get_time() - t0; sink += s;
        ESP_LOGW(TAG, "[JIT probe] DISPATCH (fetch+switch+pc): %.0f real-cyc/op",
                 (double)dt * 1000.0 / N * 240.0 / 1000.0);
    }
    (void)sink;
    ESP_LOGW(TAG, "[JIT probe] interpreter ~185 cyc/op (76%%RT). JIT estimate ~= FLOOR + small; "
                  "speedup ~= 185 / (FLOOR). Compare the two numbers above.");
}
#endif

/* ── JIT profiler: histogram of basic-block entry PCs (bank,pc) ──────────────
 * cpu_run3() calls jit_profile_sample() once per ~512-cycle batch (the PC where a
 * batch starts = a basic-block boundary). Tells us where the hot code is, i.e.
 * which blocks are worth translating and whether the hot set is concentrated. */
#define PROF_N 2048
extern "C" {
volatile bool g_jit_prof_on = false;
static uint32_t prof_key[PROF_N];   /* (bank<<16)|pc; 0 = empty */
static uint32_t prof_cnt[PROF_N];
static uint64_t prof_total = 0;
void jit_profile_sample(uint16_t pc, uint8_t bank) {
    if (!g_jit_prof_on) return;
    uint32_t k = ((uint32_t)bank << 16) | pc; if (!k) k = 1;
    uint32_t h = (k * 2654435761u) & (PROF_N - 1);
    for (int i = 0; i < PROF_N; i++) {
        uint32_t idx = (h + i) & (PROF_N - 1);
        if (prof_key[idx] == k) { prof_cnt[idx]++; prof_total++; return; }
        if (prof_key[idx] == 0) { prof_key[idx] = k; prof_cnt[idx] = 1; prof_total++; return; }
    }
}
}
static void jit_profile_report(void) {
    /* find + log the top 12 hottest (bank,pc) blocks and their % of all samples */
    uint32_t top[12]; for (int i = 0; i < 12; i++) top[i] = 0xFFFFFFFF;  /* store idx */
    uint32_t topc[12] = {0};
    int distinct = 0;
    for (int idx = 0; idx < PROF_N; idx++) {
        if (!prof_key[idx]) continue;
        distinct++;
        uint32_t c = prof_cnt[idx];
        for (int j = 0; j < 12; j++) {
            if (c > topc[j]) {
                for (int m = 11; m > j; m--) { topc[m] = topc[m-1]; top[m] = top[m-1]; }
                topc[j] = c; top[j] = idx; break;
            }
        }
    }
    uint64_t tot = prof_total ? prof_total : 1;
    ESP_LOGW(TAG, "[JIT prof] %llu samples, %d distinct blocks. Top hot blocks:", (unsigned long long)tot, distinct);
    uint32_t topsum = 0;
    for (int j = 0; j < 12 && top[j] != 0xFFFFFFFF; j++) {
        uint32_t k = prof_key[top[j]];
        topsum += topc[j];
        ESP_LOGW(TAG, "    bank=%02x pc=%04x : %lu  (%.1f%%)",
                 (k >> 16) & 0xff, k & 0xffff, (unsigned long)topc[j], 100.0 * topc[j] / tot);
    }
    ESP_LOGW(TAG, "[JIT prof] top-12 = %.0f%% of all time (concentrated=good for JIT)", 100.0 * topsum / tot);
    /* one-shot: dump the bytes of the hot blocks so we can disassemble the 6502 */
    {
        extern uint8_t& Peek16Debug(uint16_t addr);
        static bool dumped = false;
        if (!dumped) {
            dumped = true;
            for (uint16_t base : (uint16_t[]){0x04c0, 0xe310, 0xad28}) {
                char buf[160]; int p = 0;
                for (int i = 0; i < 40; i++)
                    p += snprintf(buf + p, sizeof(buf) - p, "%02x ", Peek16Debug((uint16_t)(base + i)));
                ESP_LOGW(TAG, "[JIT dump] %04x: %s", base, buf);
            }
        }
    }
    /* reset the window so each report reflects only the LAST ~5s (so active use,
     * e.g. a dictionary lookup, isn't drowned out by minutes of idle-loop samples). */
    memset(prof_key, 0, sizeof prof_key);
    memset(prof_cnt, 0, sizeof prof_cnt);
    prof_total = 0;
}

/* ── Display task (Core 1) ───────────────────────────────────────────────────
 * Core 0 snapshots the 160x80 framebuffer (CopyLcdBuffer runs on Core 0, *in
 * sequence* with the 6502, so it never races a write — no torn lcdbuffaddr / no
 * concurrent read of the emulator RAM, which was hanging the device during
 * grey-mode "绘图") and hands the finished frame to this task through a FreeRTOS
 * queue. This task owns a private copy: it touches NO emulator RAM, so the two
 * cores never read/write the same memory. It blocks on the queue (fully yields →
 * feeds the Core-1 idle/​watchdog) and only wakes when a new frame is ready, then
 * does the slow ~14ms SPI stretch-blit. Output stays uniform and the hang is gone. */
/* One self-contained frame handed Core 0 -> Core 1: the pixels AND the grey flag
 * that was in effect when they were captured, so Core 1 decodes them correctly
 * without ever reading emulator state. */
typedef struct { uint8_t grey; uint8_t fb[sizeof(lcd_buf)]; } frame_msg_t;
static QueueHandle_t g_frame_q = NULL;          /* depth-1 mailbox of full frames */
static volatile uint32_t g_disp_draws = 0;      /* diag: actual blits done */
static void display_task(void *arg)
{
    (void)arg;
    static frame_msg_t show;                     /* private — never shared with Core 0 */
    for (;;) {
        if (xQueueReceive(g_frame_q, &show, portMAX_DELAY) == pdTRUE) {
            draw_lcd(show.fb, show.grey != 0);
            g_disp_draws++;
        }
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== cardputer-nc1020 (wqx) M2 self-test ===");

    /* Grab the SD-paging slot pool before LCD/SD bring-up fragments the heap —
     * the 32 KB banks need contiguous internal RAM. */
    bool pool_ok = sdpg_reserve_pool();

    cp_lcd_init();
    cp_lcd_backlight(true);
#if PANEL_TEST
    panel_test();                              /* never returns — geometry probe */
#endif
    fill_screen(0x001F);                       /* blue = LCD alive              */
    vTaskDelay(pdMS_TO_TICKS(300));
    fill_screen(0x07E0);                       /* green = booting               */

    bool sd_ok = cp_sd_init();
    fill_screen(sd_ok ? 0x07E0 : 0xF800);      /* green ok / red SD fail        */

    cp_kbd_init();

    /* G0 button (GPIO0) — press it to force an immediate save (NOR + state). */
    gpio_config_t g0cfg = { .pin_bit_mask = 1ULL << 0, .mode = GPIO_MODE_INPUT,
                            .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&g0cfg);

    /* Pick which ROM to boot: hold G0 at boot for the on-screen menu, otherwise
     * reuse the last-selected ROM. Falls back to /sd/nc1020 if none chosen. */
    static char rombase[80] = "/sd/nc1020";
    bool rom_changed = false;
    if (sd_ok && !choose_rom(rombase, sizeof rombase, &rom_changed))
        ESP_LOGW(TAG, "no ROM selected; trying %s", rombase);
    /* Configure + prepare even without SD: a complete flash image holds the ROM/NOR
     * already (flash_rom_prepare trusts the baked data when the SD .rom is absent),
     * so the device still boots the dictionary card-less. */
    wqx_configure(rombase);
    cpu_batch = 512; // Reduce from 4096 to 512 to prevent watchdog timeouts
    ESP_LOGI(TAG, "configured rom=%s (SD %s, changed=%d)", nc2k_rom.romPath.c_str(),
             sd_ok ? "ok" : "FAIL", (int)rom_changed);

    {
        void rom_progress(int pct);
        flash_rom_prepare(nc2k_rom.romPath.c_str(), rom_progress);
        /* Re-seed NOR only when the ROM was actually re-copied to flash (content
         * changed) — keeps a matching ROM+NOR pair and blanks a stale NOR on switch. */
        nor_flash_prepare(nc2k_rom.norFlashPath.c_str(), 512 * 1024, flash_rom_repopulated());
    }
    /* A newly selected/flashed ROM must not inherit the previous ROM's saved state
     * (it would load garbage and could lock up). Wipe it so the new ROM cold-boots. */
    if (rom_changed) {
        ESP_LOGW(TAG, "ROM changed — clearing stale saved state");
        delete_state("");
    }

#if RUN_EMULATOR
    if (!pool_ok || flash_rom_banks() == 0) {
        ESP_LOGE(TAG, "no ROM available — need an SD card with a .rom, or a complete flash image");
        fill_screen(0xF800);
        for (;;) { poll_keyboard(false); vTaskDelay(pdMS_TO_TICKS(50)); }
    }
    /* crash-loop guard: a bad .state that crashes the firmware would reboot-loop.
     * The bootflag counts consecutive boots that DON'T survive 30 s; only after 3 in
     * a row (a genuine crash loop) do we delete the state. This way a single hardware
     * RST keeps the saved state (loads normally) — only a real crash loop wipes it. */
    int boot_strikes = 0;
    if (sd_ok) {
        FILE* f = fopen("/sd/roms/nc1020.bootflag", "rb");
        if (f) { int c = fgetc(f); boot_strikes = (c > 0) ? c : 0; fclose(f); }
    }
    if (sd_ok && boot_strikes >= 3) {
        ESP_LOGW(TAG, "crash-loop (%d rapid reboots) — deleting bad state file", boot_strikes);
        delete_state("");
        boot_strikes = 0;
        remove("/sd/roms/nc1020.bootflag");
    }
    ESP_LOGW(TAG, "calling LoadNC2k() ...");
    LoadNC2k();   /* SD-paged ROM/NOR (M3) — no PSRAM on Cardputer */
    ESP_LOGW(TAG, "LoadNC2k() returned — emulator loop starting");
    /* One-shot RTC seed from build timestamp. Must run AFTER LoadNC2k so the
     * .state read has populated ext_reg[] (we only seed if it's still 0). */
    seed_initial_rtc_time();
    /* arm the strike counter: record boot_strikes+1. If this boot also fails to
     * survive 30 s (clears it below), the next boot sees the higher count. */
    if (sd_ok && enable_load_state) {
        FILE* f = fopen("/sd/roms/nc1020.bootflag", "wb");
        if (f) { fputc(boot_strikes + 1, f); fclose(f); }
    }
    fill_screen(0x0000);                  /* black; LCD frame is drawn centred */
    ui_free();                            /* release the boot-UI framebuffer (~4KB) */
#if JIT_PROBE
    bench_jit_feasibility();              /* one-shot: estimate JIT speedup potential */
#endif
    int flush_tick = 0, hb = 0;
    uint32_t last_reads = 0;
    /* Dual-core (user design): Core 0 = 6502 + input + save + frame snapshot;
     * Core 1 = SPI stretch-blit. They share NO RAM — frames cross via this queue. */
    g_frame_q = xQueueCreate(1, sizeof(frame_msg_t));   /* depth-1 latest-frame mailbox */
    if (!g_frame_q) {
        ESP_LOGE(TAG, "frame queue alloc failed (%u B) — running without Core-1 display",
                 (unsigned)sizeof(frame_msg_t));
    } else if (xTaskCreatePinnedToCore(display_task, "lcd", 6144, NULL, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "display_task create failed — running without Core-1 display");
        vQueueDelete(g_frame_q);
        g_frame_q = NULL;                                /* snapshot block skips enqueue */
    }
#define KEY_SWEEP_TEST 0     /* interactivity confirmed; physical keyboard now drives it */
    for (;;) {
        fast_forward = false;
        RunTimeSlice(10); // Reduce timeslice from 33 to 10 to prevent watchdog timeouts
#if KEY_SWEEP_TEST
        /* Auto-inject each key code in turn (press ~8 frames, release ~2) to
         * verify the input->CPU->display path without a physical keyboard. */
        {
            static int ks_code = 0x08, ks_t = 0;
            if (ks_t == 1) wqx_set_key(ks_code, true);
            if (++ks_t >= 10) {
                wqx_set_key(ks_code, false);
                ks_t = 0;
                if (++ks_code > 0x3f) ks_code = 0x08;
                ESP_LOGW(TAG, "SWEEP pressing key 0x%02x", ks_code);
            }
        }
#else
        poll_keyboard(true);
#endif
        /* Fn + Del (held together) = SOFT RESET: delete the saved state, then restart
         * the EMULATED NC1020 firmware (cold_reset -> re-inits from its reset vector /
         * 资料整理). Use this to recover from a stuck/corrupt session — it wipes the
         * state so you get a clean slate now and on the next boot. (The hardware RST
         * button just reboots the ESP, which loads the saved state normally — it does
         * NOT delete it.) Edge-detected. Fn = any 0x00-mapped key; Del = (y0,x13). */
        {
            cp_key_t rk[16];
            int rn = cp_kbd_scan(rk, 16);
            bool fn = false, del = false;
            for (int i = 0; i < rn; i++) if (rk[i].y < 4 && rk[i].x < 14) {
                if (kb_map[rk[i].y][rk[i].x] == 0x00) fn = true;
                if (rk[i].y == 0 && rk[i].x == 13)    del = true;
            }
            static bool rst_prev = false;
            if (fn && del && !rst_prev) {
                ESP_LOGW(TAG, "Fn+Del = soft reset: delete state + restart NC1020 firmware");
                sdpg_flush();
                delete_state("");
                cold_reset();
            }
            rst_prev = (fn && del);
        }
#define RESUME_WAKE_TEST 0   /* one-shot: ~2s after boot inject a benign key (中英数
                              * toggle, 0x39) to test whether the firmware wakes from
                              * its post-resume standby and repaints (nonzero -> >0). */
#if RESUME_WAKE_TEST
        {
            static int wk = 0;
            if (wk >= 0) {
                wk++;
                if (wk == 130) { ESP_LOGW(TAG, "WAKE-TEST: injecting key 0x39 down"); wqx_set_key(0x39, true); }
                if (wk == 142) { ESP_LOGW(TAG, "WAKE-TEST: key 0x39 up");            wqx_set_key(0x39, false); wk = -1; }
            }
        }
#endif
        /* Snapshot the frame ON CORE 0 (sequential with the 6502 → no torn read,
         * no cross-core RAM race) at ~30fps and hand it to Core 1 via the queue.
         * Only enqueue non-blank changed frames so a static screen costs nothing
         * and Core 1 stays blocked (idle/watchdog fed). xQueueOverwrite keeps just
         * the latest frame, so a slow blit never backs up a queue. */
        if (g_frame_q) {
            static int64_t last_snap = 0;
            static frame_msg_t snap;
            static uint8_t prev[sizeof(lcd_buf)];
            static uint8_t prev_grey = 0xff;             /* force first enqueue */
            int64_t now = esp_timer_get_time();
            if (now - last_snap >= 33000) {              /* ~30 fps cap */
                last_snap = now;
                /* Capture grey ON CORE 0, alongside the pixels CopyLcdBuffer copies
                 * for that same mode — keeps decode and bytes consistent (finding #1). */
                snap.grey = is_grey_mode() ? 1 : 0;
                if (CopyLcdBuffer(snap.fb)) {
                    /* Live bytes: 1600 in 1bpp, 3200 in grey — only scan/diff those. */
                    size_t live = snap.grey ? sizeof(snap.fb) : (sizeof(snap.fb) / 2);
                    bool blank = true;
                    for (size_t i = 0; i < live; i++) if (snap.fb[i]) { blank = false; break; }
                    if (!blank && (snap.grey != prev_grey ||
                                   memcmp(snap.fb, prev, live) != 0)) {
                        memcpy(prev, snap.fb, live);
                        prev_grey = snap.grey;
                        xQueueOverwrite(g_frame_q, &snap);
                    }
                }
            }
        }

        /* Persist dirty NOR pages to SD ~every 5 s so user data survives a
         * power-off (NC1020 has no clean shutdown). Cheap when nothing changed. */
        if (++flush_tick >= 150) { flush_tick = 0; sdpg_flush(); }

        /* State persistence = **G0 manual save ONLY** (auto-save removed per request).
         * Press G0 to snapshot the ~99 KB RAM state to SD; nothing saves automatically,
         * so the periodic mid-session SD-write freeze is gone (it was also disturbing
         * the image). Remember to press G0 before powering off, or the next boot
         * cold-boots into "是否清除闪存". NOR (documents) still flushes every ~5 s above. */
        {
            static int g0_prev = 1;
            int g0 = gpio_get_level(GPIO_NUM_0);
            if (g0 == 0 && g0_prev == 1) {           /* G0 falling edge = pressed */
                ESP_LOGI(TAG, "G0 pressed — saving state");
                sdpg_flush();
                save_state("");
            }
            g0_prev = g0;
        }
        if (++hb >= 30) {
            hb = 0;
            uint32_t r = sdpg_read_count();
            static uint32_t last_draws = 0; uint32_t dd = g_disp_draws;
            ESP_LOGI(TAG, "heartbeat: cycles=%llu  SD-reads(+%lu)  RTC=%02d:%02d:%02d  draws(+%lu)",
                     (unsigned long long)nc2k_states.cycles, (unsigned long)(r - last_reads),
                     nc2k_states.ext_reg[2], nc2k_states.ext_reg[1], nc2k_states.ext_reg[0],
                     (unsigned long)(dd - last_draws));
            last_reads = r; last_draws = dd;
            static int prof_tick = 0;
            if (++prof_tick >= 12) { prof_tick = 0; jit_profile_report(); }   /* ~every 5s */
            /* after 30 s stable, clear the boot flag — state is good */
            static bool bootflag_cleared = false;
            if (!bootflag_cleared) {
                bootflag_cleared = true;
                remove("/sd/roms/nc1020.bootflag");
            }
        }
        /* We run far below realtime, so don't sleep a full frame (that was pure
         * idle waste). Yield 1 tick to feed the watchdog/idle and run flat out. */
        vTaskDelay(1);
    }
#else
    /* M2: just echo keys to the serial log so the keyboard matrix can be checked. */
    ESP_LOGW(TAG, "M2 self-test running: press keys, watch the log for 'KEY xx down'");
    for (;;) {
        poll_keyboard(false);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
#endif
}
