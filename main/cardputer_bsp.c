#include "cardputer_bsp.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "driver/i2s_std.h"
#include <math.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"

static const char *TAG = "cp_bsp";

/* ── Pins (M5Cardputer) ─────────────────────────────────────────────────── */
#define LCD_SCLK  36
#define LCD_MOSI  35
#define LCD_CS    37
#define LCD_DC    34
#define LCD_RST   33
#define LCD_BL    38

#define SD_SCLK   40
#define SD_MISO   39
#define SD_MOSI   14
#define SD_CS     12

#define LCD_HOST  SPI2_HOST
#define SD_HOST   SPI3_HOST

/* Speaker (NS4168 I2S amp) — M5Cardputer */
#define SPK_BCLK  41
#define SPK_WS    43
#define SPK_DOUT  42
#define SPK_RATE  44100

/* ── Display (ST7789, 240x135 landscape; native 135x240 panel) ──────────── */
static esp_lcd_panel_handle_t s_panel;

esp_lcd_panel_handle_t cp_lcd_init(void)
{
    gpio_config_t bl = { .pin_bit_mask = 1ULL << LCD_BL, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&bl);
    gpio_set_level(LCD_BL, 0);

    spi_bus_config_t bus = {
        .sclk_io_num = LCD_SCLK, .mosi_io_num = LCD_MOSI, .miso_io_num = -1,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = CP_LCD_W * CP_LCD_H * 2 + 16,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_DC, .cs_gpio_num = LCD_CS,
        .pclk_hz = 40 * 1000 * 1000, .lcd_cmd_bits = 8, .lcd_param_bits = 8,
        .spi_mode = 0, .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io));

    esp_lcd_panel_dev_config_t pcfg = {
        .reset_gpio_num = LCD_RST, .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &pcfg, &s_panel));
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, true);          /* ST7789 is inverted    */
    /* Landscape 240x135: swap axes, mirror to taste, and apply the panel's RAM
     * offset (native 135x240 has a 40/52 gap). Tune mirror/gap on hardware. */
    /* NATIVE portrait 135x240 (no swap_xy) — M5GFX's Cardputer panel is 135x240
     * with offset_x=52, offset_y=40. swap_xy sheared the image, so we draw in the
     * native portrait frame and rotate the content in software. */
    esp_lcd_panel_swap_xy(s_panel, false);
    esp_lcd_panel_mirror(s_panel, false, false);
    esp_lcd_panel_set_gap(s_panel, 52, 40);
    esp_lcd_panel_disp_on_off(s_panel, true);
    ESP_LOGI(TAG, "ST7789 init done");
    return s_panel;
}

void cp_lcd_backlight(bool on) { gpio_set_level(LCD_BL, on ? 1 : 0); }

void cp_lcd_draw(int x0, int y0, int x1, int y1, const uint16_t *px)
{
    if (s_panel) esp_lcd_panel_draw_bitmap(s_panel, x0, y0, x1, y1, px);
}

/* ── SD card (SPI) ──────────────────────────────────────────────────────── */
bool cp_sd_init(void)
{
    spi_bus_config_t bus = {
        .sclk_io_num = SD_SCLK, .mosi_io_num = SD_MOSI, .miso_io_num = SD_MISO,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = 8192,   /* one 8KB page per DMA xfer. Bigger wastes
                                    * internal RAM better spent on the paging cache. */
    };
    if (spi_bus_initialize(SD_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        ESP_LOGE(TAG, "SD spi bus init failed");
        return false;
    }

    /* SD-over-SPI cold init is flaky: a card often fails the first probe but mounts
     * on retry. Start at 20 MHz (below the 25 MHz high-speed threshold, so no HS-mode
     * negotiation — 26.8 MHz reliably failed enable_hs_mode_and_check / send_csd on
     * this card), then back off to 10 then 4 MHz. */
    static const int freqs_khz[] = { 20000, 20000, 10000, 4000 };
    esp_vfs_fat_sdmmc_mount_config_t mcfg = {
        .format_if_mount_failed = false, .max_files = 5, .allocation_unit_size = 16 * 1024,
    };
    for (int i = 0; i < (int)(sizeof(freqs_khz) / sizeof(freqs_khz[0])); i++) {
        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        host.slot = SD_HOST;
        host.max_freq_khz = freqs_khz[i];
        sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot.gpio_cs = SD_CS;
        slot.host_id = SD_HOST;
        sdmmc_card_t *card;
        esp_err_t e = esp_vfs_fat_sdspi_mount("/sd", &host, &slot, &mcfg, &card);
        if (e == ESP_OK) {
            ESP_LOGI(TAG, "SD mounted @%dkHz (%lluMB)", freqs_khz[i],
                     ((uint64_t)card->csd.capacity * card->csd.sector_size) >> 20);
            return true;
        }
        ESP_LOGW(TAG, "SD mount @%dkHz failed: %s (try %d/%d)", freqs_khz[i],
                 esp_err_to_name(e), i + 1, (int)(sizeof(freqs_khz) / sizeof(freqs_khz[0])));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    spi_bus_free(SD_HOST);
    ESP_LOGE(TAG, "SD mount failed after retries");
    return false;
}

/* ── Keyboard matrix (74HC138 + GPIO), from M5Cardputer IOMatrix ─────────── */
static const int s_out[3] = { 8, 9, 11 };
static const int s_in[7]  = { 13, 15, 3, 4, 5, 6, 7 };
/* per input-row j: {x for cols 0..3, x for cols 4..7} */
static const uint8_t s_xmap[7][2] = {
    {1,0},{3,2},{5,4},{7,6},{9,8},{11,10},{13,12}};

void cp_kbd_init(void)
{
    for (int i = 0; i < 3; i++) {
        gpio_config_t c = { .pin_bit_mask = 1ULL << s_out[i], .mode = GPIO_MODE_OUTPUT };
        gpio_config(&c); gpio_set_level(s_out[i], 0);
    }
    for (int i = 0; i < 7; i++) {
        gpio_config_t c = { .pin_bit_mask = 1ULL << s_in[i], .mode = GPIO_MODE_INPUT,
                            .pull_up_en = GPIO_PULLUP_ENABLE };
        gpio_config(&c);
    }
}

int cp_kbd_scan(cp_key_t *out, int max)
{
    int n = 0;
    for (int i = 0; i < 8; i++) {
        gpio_set_level(s_out[0], i & 1);
        gpio_set_level(s_out[1], (i >> 1) & 1);
        gpio_set_level(s_out[2], (i >> 2) & 1);
        esp_rom_delay_us(5);
        for (int j = 0; j < 7; j++) {
            if (gpio_get_level(s_in[j]) == 0) {        /* pull-up: pressed = low */
                if (n >= max) return n;
                out[n].x = (i > 3) ? s_xmap[j][1] : s_xmap[j][0];
                int y = (i > 3) ? (i - 4) : i;
                out[n].y = (uint8_t)(3 - y);
                n++;
            }
        }
    }
    return n;
}

/* ── Audio (I2S std TX -> NS4168 speaker; 44100 Hz, 16-bit, mono) ─────────── */
static i2s_chan_handle_t s_tx;

bool cp_audio_init(void)
{
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    if (i2s_new_channel(&cc, &s_tx, NULL) != ESP_OK) { ESP_LOGE(TAG, "i2s_new_channel failed"); return false; }
    i2s_std_config_t std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SPK_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, .bclk = SPK_BCLK, .ws = SPK_WS,
            .dout = SPK_DOUT, .din = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    if (i2s_channel_init_std_mode(s_tx, &std) != ESP_OK) { ESP_LOGE(TAG, "i2s init_std failed"); return false; }
    if (i2s_channel_enable(s_tx) != ESP_OK) { ESP_LOGE(TAG, "i2s enable failed"); return false; }
    ESP_LOGI(TAG, "audio I2S ready (bclk=%d ws=%d dout=%d @%dHz)", SPK_BCLK, SPK_WS, SPK_DOUT, SPK_RATE);
    return true;
}

void cp_audio_write(const int16_t *samples, int n)
{
    if (!s_tx || n <= 0) return;
    size_t wrote = 0;
    i2s_channel_write(s_tx, samples, (size_t)n * sizeof(int16_t), &wrote, portMAX_DELAY);
}

/* Stop the I2S clock so the NS4168 amp drops to standby (no continuous-feed hiss);
 * the audio task calls these around runs of real sound. Enable/disable are only
 * called on a real active<->idle transition, so no rapid toggling. */
void cp_audio_pause(void)  { if (s_tx) i2s_channel_disable(s_tx); }
void cp_audio_resume(void) { if (s_tx) i2s_channel_enable(s_tx); }

/* Short startup tone so the speaker/pins can be confirmed on hardware. Two beeps:
 * 880 Hz then 1320 Hz, ~125 ms each. Generated in a SMALL stack buffer (a big
 * static .bss buffer re-triggered the SD-mount NO_MEM), phase kept continuous. */
void cp_audio_beep_test(void)
{
    if (!s_tx) return;
    int16_t buf[256];
    const int amp = 6000;
    const int freqs[3] = { 880, 1320, 0 };   /* 0 = trailing silence */
    for (int f = 0; f < 3; f++) {
        int total  = (freqs[f] == 0) ? SPK_RATE / 20 : SPK_RATE / 8;   /* 50ms / 125ms */
        int period = (freqs[f] == 0) ? 2 : SPK_RATE / freqs[f];
        int phase  = 0;
        while (total > 0) {
            int n = total < 256 ? total : 256;
            for (int i = 0; i < n; i++) {
                buf[i] = (freqs[f] == 0) ? 0 : ((phase < period / 2) ? amp : -amp);
                if (++phase >= period) phase = 0;
            }
            cp_audio_write(buf, n);
            total -= n;
        }
    }
}
