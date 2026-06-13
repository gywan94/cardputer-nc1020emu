#include "sd_paging.h"
#include "flash_rom.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"

/* Page at 8 KB, not the 32 KB hardware bank: memmap[] already addresses memory
 * in 8 KB pages (memmap[2..5] are the four 8 KB sub-pages of the selected bank,
 * as four independent pointers), and a no-PSRAM Cardputer's heap is too
 * fragmented (largest contiguous block ~72 KB) to hold 32 KB slots. 8 KB slots
 * pack into the fragments and let enough banks stay pinned per bus switch. */
#define SDPG_PAGE    0x2000u   /* 8 KB per slot                                   */
#define SDPG_NSLOTS  10        /* 10*8KB = 80 KB. Max live pages/switch is 6 (4    */
                               /* subpages + nor[0] + nor[bbs]). Was 14 (112KB);   */
                               /* cut to free ~32KB internal RAM for the FAT/SD     */
                               /* mount (it failed with NO_MEM 0x101 after recent   */
                               /* static allocs). ROM+NOR are in flash so SD reads  */
                               /* are ~0 and the paging cache is barely used.       */
#define ROM_PAGES    1536u     /* 12 MB / 8 KB = 384 banks * 4                     */

static const char *TAG = "sdpg";

/* key: ROM page 0..1535 ; NOR page -> 0x10000 | q (q = nor_bank*4 + sub, 0..63) */
#define KEY_EMPTY     (-1)
#define KEY_NOR(q)    ((int)(0x10000 | (unsigned)(q)))
#define IS_NOR_KEY(k) ((k) != KEY_EMPTY && ((k) & 0x10000) != 0)
#define NOR_PAGE(k)   ((uint32_t)((k) & 0xffff))

typedef struct {
    uint8_t *buf;     /* 8 KB resident page                   */
    int      key;     /* KEY_EMPTY or encoded id              */
    uint32_t gen;     /* last-touched switch generation (LRU) */
    bool     dirty;   /* NOR only: needs write-back           */
} slot_t;

static slot_t   s_slot[SDPG_NSLOTS];
static FILE    *s_rom_fp    = NULL;
static FILE    *s_nor_fp    = NULL;
static uint32_t s_nor_banks = 0;
static uint32_t s_gen       = 1;
static bool     s_pool_ok   = false;
static uint32_t s_reads     = 0;   /* count of actual SD page reads (cache misses) */

uint32_t sdpg_read_count(void) { return s_reads; }

/* ------------------------------------------------------------------ file I/O */
static void read_page(FILE *fp, uint32_t offset, uint8_t *dst) {
    memset(dst, 0xff, SDPG_PAGE);           /* short read / no file -> blank flash */
    if (!fp) return;
    if (fseek(fp, (long)offset, SEEK_SET) != 0) return;
    fread(dst, 1, SDPG_PAGE, fp);
    s_reads++;
}

static void write_nor_page(uint32_t q, const uint8_t *src) {
    if (nor_flash_ready()) { nor_flash_write(q * SDPG_PAGE, src, SDPG_PAGE); return; }
    if (!s_nor_fp) return;
    if (fseek(s_nor_fp, (long)(q * SDPG_PAGE), SEEK_SET) != 0) return;
    fwrite(src, 1, SDPG_PAGE, s_nor_fp);
    fflush(s_nor_fp);
}

/* init_rom() volume mapping, bank_idx in [0x80,0xFF]:
 *   v0 -> bank_idx-128 ; v1 -> bank_idx ; v2 -> 128+bank_idx  (each a 32KB bank) */
static int rom_filebank(int volume, uint8_t bank_idx) {
    if (volume == 1) return bank_idx;
    if (volume == 2) return 128 + bank_idx;
    return bank_idx - 128;
}

/* --------------------------------------------------------------- slot lookup */
static int find_slot(int key) {
    for (int i = 0; i < SDPG_NSLOTS; i++)
        if (s_slot[i].key == key) return i;
    return -1;
}

static int pick_victim(void) {
    int best = -1; uint32_t bestgen = 0xffffffffu;
    for (int i = 0; i < SDPG_NSLOTS; i++) {
        if (s_slot[i].key == KEY_EMPTY) return i;
        if (s_slot[i].gen == s_gen) continue;        /* pinned this switch */
        if (s_slot[i].gen < bestgen) { bestgen = s_slot[i].gen; best = i; }
    }
    if (best < 0) {                                  /* every slot pinned: pool too small */
        ESP_LOGE(TAG, "all %d slots pinned in one switch (pool too small)", SDPG_NSLOTS);
        best = 0;
    }
    return best;
}

/* key = ROM page index (is_nor=false) or KEY_NOR(q) (is_nor=true).
 * file_off = byte offset into the matching file (page-aligned). */
static uint8_t *resolve(int key, bool is_nor, uint32_t file_off) {
    int i = find_slot(key);
    if (i >= 0) { s_slot[i].gen = s_gen; return s_slot[i].buf; }

    i = pick_victim();
    if (s_slot[i].dirty && IS_NOR_KEY(s_slot[i].key))
        write_nor_page(NOR_PAGE(s_slot[i].key), s_slot[i].buf);
    s_slot[i].dirty = false;

    if (is_nor && nor_flash_ready())
        nor_flash_read(file_off, s_slot[i].buf, SDPG_PAGE);   /* NOR from flash */
    else
        read_page(is_nor ? s_nor_fp : s_rom_fp, file_off, s_slot[i].buf);

    s_slot[i].key = key;
    s_slot[i].gen = s_gen;
    return s_slot[i].buf;
}

/* -------------------------------------------------------------------- public */
bool sdpg_ready(void) { return s_pool_ok; }

bool sdpg_reserve_pool(void) {
    if (s_pool_ok) return true;
    ESP_LOGW(TAG, "reserve pool: free internal %u, largest block %u (need %d x8KB)",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             SDPG_NSLOTS);
    for (int i = 0; i < SDPG_NSLOTS; i++) {
        s_slot[i].buf = (uint8_t *)heap_caps_malloc(SDPG_PAGE, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
        if (!s_slot[i].buf) {
            ESP_LOGE(TAG, "slot %d/%d alloc failed (largest block now %u) — lower SDPG_NSLOTS",
                     i, SDPG_NSLOTS, (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
            for (int j = 0; j < i; j++) { free(s_slot[j].buf); s_slot[j].buf = NULL; }
            return false;
        }
        s_slot[i].key = KEY_EMPTY; s_slot[i].gen = 0; s_slot[i].dirty = false;
    }
    s_pool_ok = true;
    ESP_LOGI(TAG, "pool reserved: %d slots x8KB = %dKB, free internal now %u",
             SDPG_NSLOTS, SDPG_NSLOTS * 8,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    return true;
}

bool sdpg_init(const char *norPath, uint32_t num_nor_pages) {
    s_nor_banks = num_nor_pages;
    if (!sdpg_reserve_pool()) return false;

    if (nor_flash_ready()) {                 /* NOR backed by flash — no SD .nor */
        ESP_LOGI(TAG, "NOR ready (%u banks, flash-backed)", (unsigned)s_nor_banks);
        return true;
    }

    s_nor_fp = fopen(norPath, "rb+");
    if (!s_nor_fp) {
        s_nor_fp = fopen(norPath, "wb+");        /* create blank (all 0xff) */
        if (s_nor_fp) {
            uint8_t *z = (uint8_t *)malloc(SDPG_PAGE);
            if (z) {
                memset(z, 0xff, SDPG_PAGE);
                for (uint32_t q = 0; q < s_nor_banks * 4; q++) fwrite(z, 1, SDPG_PAGE, s_nor_fp);
                free(z);
            }
            fflush(s_nor_fp);
            ESP_LOGW(TAG, "created blank NOR %s (%u banks)", norPath, (unsigned)s_nor_banks);
        } else {
            ESP_LOGE(TAG, "cannot open/create NOR %s", norPath);
        }
    }
    ESP_LOGI(TAG, "NOR ready (%u banks, paged 8KB)", (unsigned)s_nor_banks);
    return true;
}

bool sdpg_open_rom(const char *romPath) {
    s_rom_fp = fopen(romPath, "rb");
    if (!s_rom_fp) { ESP_LOGE(TAG, "ROM %s not found", romPath); return false; }
    fseek(s_rom_fp, 0, SEEK_END);
    long sz = ftell(s_rom_fp);
    fseek(s_rom_fp, 0, SEEK_SET);
    if (sz != (long)(ROM_PAGES * SDPG_PAGE)) {
        ESP_LOGE(TAG, "ROM size %ld != %u (need 12MB; old format? get a newer rom)",
                 sz, (unsigned)(ROM_PAGES * SDPG_PAGE));
        fclose(s_rom_fp); s_rom_fp = NULL;
        return false;
    }
    ESP_LOGI(TAG, "ROM %s opened (12MB, paged 8KB)", romPath);
    return true;
}

uint8_t *sdpg_rom_sub(int volume, uint8_t bank_idx, int sub) {
    int fb = rom_filebank(volume, bank_idx);
    /* Banks cached in on-chip flash are mmapped — return a direct pointer (no
     * slot, no SD). This is what makes the dictionary fast. */
    uint32_t fbanks = flash_rom_banks();
    if ((uint32_t)fb < fbanks) {
        return (uint8_t *)(flash_rom_ptr() + (uint32_t)fb * (SDPG_PAGE * 4) + (uint32_t)sub * SDPG_PAGE);
    }
    int p = fb * 4 + sub;                       /* tail banks: SD-paged */
    return resolve(p, false, (uint32_t)p * SDPG_PAGE);
}

uint8_t *sdpg_nor_sub(uint8_t nor_bank, int sub) {
    uint32_t q = (uint32_t)nor_bank * 4 + sub;
    return resolve(KEY_NOR(q), true, q * SDPG_PAGE);
}

bool sdpg_ptr_is_nor(const uint8_t *p) {
    if (!s_pool_ok) return false;
    for (int i = 0; i < SDPG_NSLOTS; i++)
        if (IS_NOR_KEY(s_slot[i].key) && p >= s_slot[i].buf && p < s_slot[i].buf + SDPG_PAGE)
            return true;
    return false;
}

void sdpg_mark_dirty_ptr(const uint8_t *p) {
    for (int i = 0; i < SDPG_NSLOTS; i++)
        if (IS_NOR_KEY(s_slot[i].key) && p >= s_slot[i].buf && p < s_slot[i].buf + SDPG_PAGE) {
            s_slot[i].dirty = true; return;
        }
}

void sdpg_nor_mass_erase(void) {
    /* Clear any resident NOR pages first... */
    for (uint32_t q = 0; q < s_nor_banks * 4; q++) {
        int i = find_slot(KEY_NOR(q));
        if (i >= 0) { memset(s_slot[i].buf, 0xff, SDPG_PAGE); s_slot[i].dirty = false; }
    }
    if (nor_flash_ready()) { nor_flash_erase_all(); }   /* ...then erase the backing */
    else if (s_nor_fp) {
        uint8_t *z = (uint8_t *)malloc(SDPG_PAGE);
        if (z) {
            memset(z, 0xff, SDPG_PAGE);
            for (uint32_t q = 0; q < s_nor_banks * 4; q++) {
                fseek(s_nor_fp, (long)(q * SDPG_PAGE), SEEK_SET);
                fwrite(z, 1, SDPG_PAGE, s_nor_fp);
            }
            fflush(s_nor_fp);
            free(z);
        }
    }
    ESP_LOGI(TAG, "NOR mass-erased (%u banks)", (unsigned)s_nor_banks);
}

void sdpg_switch_begin(void) { s_gen++; }

void sdpg_flush(void) {
    bool any = false;
    for (int i = 0; i < SDPG_NSLOTS; i++)
        if (s_slot[i].dirty && IS_NOR_KEY(s_slot[i].key)) {
            write_nor_page(NOR_PAGE(s_slot[i].key), s_slot[i].buf);
            s_slot[i].dirty = false; any = true;
        }
    if (any) ESP_LOGI(TAG, "NOR flushed to SD");
}
