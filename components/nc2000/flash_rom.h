/*
 * flash_rom — cache the hot first ~6.9 MB of the 12 MB NC1020 ROM in the
 * on-chip flash "romdata" partition and memory-map it for direct, cache-speed
 * reads. The dictionary lives in these low banks, so this removes the SD paging
 * thrash that made lookups crawl. The remaining tail banks stay SD-paged.
 *
 * On first boot the partition is empty, so it is populated once from the SD
 * /sd/nc1020.rom (~1 min, NVS marks it done); later boots skip straight to mmap.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Populate (first boot only) the romdata partition from romPath, then mmap it.
 * progress(pct) is called 0..100 during the one-time copy (may be NULL).
 * Safe to call once early in app_main(). Returns true if the mmap is live. */
bool flash_rom_prepare(const char *romPath, void (*progress)(int pct));

/* Number of ROM file banks (32 KB each) served from the flash mmap (0 if off). */
uint32_t flash_rom_banks(void);

/* Base pointer of the mmapped romdata (bank b at base + b*0x8000). NULL if off. */
const uint8_t *flash_rom_ptr(void);

/* True if flash_rom_prepare() re-copied the ROM this boot (ROM actually changed).
 * Pass to nor_flash_prepare() so the NOR is re-seeded to match the new ROM. */
bool flash_rom_repopulated(void);

/* ── NOR (read-write user data) backed by the on-chip "nordata" partition ──
 * The 512 KB NOR lives in flash instead of the SD .nor file: page reads come
 * from flash, the slot cache holds the writable working set, dirty pages are
 * erased+written back here. On first use it is seeded from sdNorPath if that
 * file exists (so existing user data carries over), else left blank (0xff). */
bool nor_flash_prepare(const char *sdNorPath, uint32_t nor_size, bool force_reseed);
bool nor_flash_ready(void);
void nor_flash_read(uint32_t off, uint8_t *dst, uint32_t len);   /* page-in        */
void nor_flash_write(uint32_t off, const uint8_t *src, uint32_t len); /* erase+write */
void nor_flash_erase_all(void);                                  /* NOR mass erase */

#ifdef __cplusplus
}
#endif
