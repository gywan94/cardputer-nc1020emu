#include "flash_rom.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BANK 0x8000u   /* 32 KB hardware bank */

static const char *TAG = "flashrom";
static const void              *s_map   = NULL;
static esp_partition_mmap_handle_t s_mh;
static uint32_t                 s_banks = 0;
static bool                     s_repop = false;   /* did this boot re-copy the ROM? */

uint32_t       flash_rom_banks(void)       { return s_map ? s_banks : 0; }
const uint8_t *flash_rom_ptr(void)         { return (const uint8_t *)s_map; }
bool           flash_rom_repopulated(void) { return s_repop; }

/* A cheap signature of the SD ROM (size class + first 16 bytes) so a swapped
 * .rom triggers a re-copy instead of serving stale flash. */
static uint32_t rom_signature(FILE *fp, uint32_t cap_banks) {
    uint32_t sig = 0xC1020000u ^ cap_banks;
    if (fp) {
        uint8_t hdr[16];
        fseek(fp, 0, SEEK_SET);
        size_t n = fread(hdr, 1, sizeof(hdr), fp);
        for (size_t i = 0; i < n; i++) sig = sig * 131u + hdr[i];
    }
    return sig;
}

static bool nvs_sig_ok(uint32_t sig) {
    nvs_handle_t h;
    if (nvs_open("nc1020", NVS_READONLY, &h) != ESP_OK) return false;
    uint32_t v = 0;
    esp_err_t e = nvs_get_u32(h, "romsig", &v);
    nvs_close(h);
    return e == ESP_OK && v == sig;
}
static void nvs_sig_set(uint32_t sig) {
    nvs_handle_t h;
    if (nvs_open("nc1020", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, "romsig", sig);
    nvs_commit(h);
    nvs_close(h);
}

bool flash_rom_prepare(const char *romPath, void (*progress)(int)) {
    /* NVS must be ready for the populated-marker. */
    esp_err_t nv = nvs_flash_init();
    if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    const esp_partition_t *p = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "romdata");
    if (!p) { ESP_LOGE(TAG, "no 'romdata' partition"); return false; }
    uint32_t cap_banks = p->size / BANK;          /* 222 */

    FILE *fp = fopen(romPath, "rb");
    if (!fp) ESP_LOGW(TAG, "%s not open (tail/populate from SD unavailable)", romPath);

    uint32_t sig = rom_signature(fp, cap_banks);

    if (!nvs_sig_ok(sig)) {
      if (!fp) {
        /* No SD ROM — a complete flash image may already hold romdata. Trust it
         * if bank 0 isn't blank (all 0xff); otherwise there is nothing to run. */
        uint8_t probe[64];
        esp_partition_read(p, 0, probe, sizeof(probe));
        bool blank = true;
        for (unsigned i = 0; i < sizeof(probe); i++) if (probe[i] != 0xff) { blank = false; break; }
        if (blank) { ESP_LOGE(TAG, "romdata empty and no SD ROM to populate"); return false; }
        ESP_LOGW(TAG, "romdata pre-filled (no SD) — trusting embedded image");
        nvs_sig_set(sig);
      } else {
        ESP_LOGW(TAG, "populating romdata: %u banks (%.1f MB) from SD — one-time, ~1 min",
                 (unsigned)cap_banks, cap_banks * 32.0 / 1024.0);
        /* Copy in 4 KB chunks: the SD-paging pool has already fragmented the heap,
         * so a 32 KB buffer won't allocate, but 4 KB always will. */
        const uint32_t CHUNK = 4096u;
        uint8_t *buf = (uint8_t *)malloc(CHUNK);
        if (!buf) {
            ESP_LOGE(TAG, "copy buffer alloc failed (largest free %u)",
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
            fclose(fp); return false;
        }
        bool ok = true;
        for (uint32_t b = 0; b < cap_banks && ok; b++) {
            if (esp_partition_erase_range(p, (size_t)b * BANK, BANK) != ESP_OK) {
                ESP_LOGE(TAG, "erase bank %u failed", (unsigned)b); ok = false; break;
            }
            for (uint32_t off = 0; off < BANK; off += CHUNK) {
                fseek(fp, (long)b * BANK + off, SEEK_SET);
                size_t n = fread(buf, 1, CHUNK, fp);
                if (n < CHUNK) memset(buf + n, 0xff, CHUNK - n);
                if (esp_partition_write(p, (size_t)b * BANK + off, buf, CHUNK) != ESP_OK) {
                    ESP_LOGE(TAG, "write bank %u+%u failed", (unsigned)b, (unsigned)off); ok = false; break;
                }
            }
            if (progress) progress((int)((b + 1) * 100 / cap_banks));
            if ((b & 3) == 0) {
                if ((b & 15) == 0) {
                    ESP_LOGI(TAG, "  ... %u/%u banks", (unsigned)b, (unsigned)cap_banks);
                }
                vTaskDelay(1);   /* feed the watchdog / let other tasks run */
            }
        }
        free(buf);
        if (!ok) { fclose(fp); return false; }
        nvs_sig_set(sig);
        s_repop = true;                  /* ROM content changed -> NOR must follow */
        ESP_LOGW(TAG, "romdata populated & marked");
      }
    } else {
        ESP_LOGI(TAG, "romdata already populated (sig 0x%08x)", (unsigned)sig);
    }
    if (fp) fclose(fp);

    esp_err_t e = esp_partition_mmap(p, 0, p->size, ESP_PARTITION_MMAP_DATA, &s_map, &s_mh);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "mmap %u bytes failed (%s) — ROM stays SD-paged",
                 (unsigned)p->size, esp_err_to_name(e));
        s_map = NULL;
        return false;
    }
    s_banks = cap_banks;
    ESP_LOGI(TAG, "romdata mmapped: %u banks at %p — dictionary now direct-read",
             (unsigned)s_banks, s_map);
    return true;
}

/* ───────────────────────── NOR in flash ───────────────────────────────── */
static const esp_partition_t *s_nor_part = NULL;

bool nor_flash_ready(void) { return s_nor_part != NULL; }

bool nor_flash_prepare(const char *sdNorPath, uint32_t nor_size, bool force_reseed) {
    const esp_partition_t *p = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x41, "nordata");
    if (!p) { ESP_LOGW(TAG, "no 'nordata' partition — NOR stays on SD"); return false; }
    if (nor_size > p->size) nor_size = p->size;

    /* Seed once: if not yet initialised (or the ROM changed -> force_reseed),
     * import the SD .nor (if any), else keep the flash contents. NVS "norinit"
     * guards against re-seeding on every boot. */
    nvs_handle_t h;
    uint32_t done = 0;
    if (nvs_open("nc1020", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, "norinit", &done); nvs_close(h);
    }
    if (force_reseed) done = 0;
    if (!done) {
        FILE *fp = fopen(sdNorPath, "rb");
        if (fp) {
            /* Seed from the SD .nor: erase then import. */
            ESP_LOGW(TAG, "seeding nordata (%u KB) from %s", (unsigned)(nor_size / 1024), sdNorPath);
            if (esp_partition_erase_range(p, 0, p->size) != ESP_OK) {
                ESP_LOGE(TAG, "nordata erase failed"); fclose(fp); return false;
            }
            uint8_t *buf = (uint8_t *)malloc(4096);
            if (buf) {
                for (uint32_t off = 0; off < nor_size; off += 4096) {
                    size_t n = fread(buf, 1, 4096, fp);
                    if (n == 0) break;
                    if (n < 4096) memset(buf + n, 0xff, 4096 - n);
                    esp_partition_write(p, off, buf, 4096);
                }
                free(buf);
            }
            fclose(fp);
            ESP_LOGW(TAG, "nordata seeded from SD");
        } else if (force_reseed) {
            /* ROM changed but no matching .nor on SD: the existing nordata belongs
             * to the OLD ROM and would corrupt the new one, so blank it (the new
             * ROM then initialises a fresh NOR — one "format" prompt). */
            ESP_LOGW(TAG, "nordata: ROM changed, no .nor — erasing to blank NOR");
            esp_partition_erase_range(p, 0, p->size);
        } else {
            /* Initial seed, no SD .nor: keep the flash contents as-is — a complete
             * image keeps its baked NOR; a blank (0xff) partition is a blank NOR. */
            ESP_LOGW(TAG, "nordata: no SD .nor, using flash contents as-is");
        }
        if (nvs_open("nc1020", NVS_READWRITE, &h) == ESP_OK) {
            nvs_set_u32(h, "norinit", 1); nvs_commit(h); nvs_close(h);
        }
    }
    s_nor_part = p;
    ESP_LOGI(TAG, "NOR in flash ready (%u KB partition)", (unsigned)(p->size / 1024));
    return true;
}

void nor_flash_read(uint32_t off, uint8_t *dst, uint32_t len) {
    if (!s_nor_part || off + len > s_nor_part->size) { memset(dst, 0xff, len); return; }
    if (esp_partition_read(s_nor_part, off, dst, len) != ESP_OK) memset(dst, 0xff, len);
}

void nor_flash_write(uint32_t off, const uint8_t *src, uint32_t len) {
    if (!s_nor_part || off + len > s_nor_part->size) return;
    /* Fast path — skip the erase when the page only CLEARS bits. Flash programming can
     * turn a 1 into a 0 without erasing; only setting a bit back to 1 needs the (~tens
     * of ms) erase. The emulated NOR byte-program is `&=` (bit-clear only), so a flushed
     * page is almost always a subset of what's already in flash → program-only, no erase.
     * Only a real NOR erase command (memset 0xff → sets bits) takes the slow path. Kills
     * the periodic write-back stall + flash wear. Safe: flash encryption is OFF (re-
     * programming already-written cells to a subset value is well-defined unencrypted). */
    static uint8_t *cur = NULL;     /* scratch for the current page; lazily heap-alloc'd */
    static uint32_t cap = 0;        /* AFTER SD mount, so it can't reopen the boot-time   */
    if (!cur) {                     /* NO_MEM mount failure that shrinking the slot pool   */
        cap = len;                  /* was meant to avoid. */
        cur = (uint8_t *)heap_caps_malloc(cap, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    }
    if (cur && len <= cap &&
        esp_partition_read(s_nor_part, off, cur, len) == ESP_OK) {
        bool needs_erase = false, differs = false;
        for (uint32_t i = 0; i < len; i++) {
            if (src[i] != cur[i]) {
                differs = true;
                if (src[i] & (uint8_t)~cur[i]) { needs_erase = true; break; }  /* a 0->1 bit */
            }
        }
        if (!needs_erase) {
            if (differs) esp_partition_write(s_nor_part, off, src, len);   /* program-only */
            return;                                                        /* identical: skip */
        }
    }
    /* Real erase needed (or scratch unavailable): the original safe path. */
    esp_partition_erase_range(s_nor_part, off, len);
    esp_partition_write(s_nor_part, off, src, len);
}

void nor_flash_erase_all(void) {
    if (s_nor_part) esp_partition_erase_range(s_nor_part, 0, s_nor_part->size);
}
