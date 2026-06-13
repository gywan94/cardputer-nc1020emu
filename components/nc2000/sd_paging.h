/*
 * sd_paging — SD-backed bank cache for the wqx core on a no-PSRAM Cardputer.
 *
 * The Cardputer (StampS3) has 8 MB flash and NO PSRAM, so neither the 12 MB ROM
 * nor the 512 KB NOR can be held in RAM. This keeps a small pool of 32 KB slots
 * (one per hardware bank) in internal RAM and pages banks in from the .rom /.nor
 * files on demand. ROM is read-only; NOR is written back (dirty slots flushed to
 * the .nor file on eviction, on sdpg_flush(), and on mass-erase).
 *
 * The CPU fast path (CPU_PEEK in w65c02.h) reads memmap[] pointers directly, so
 * a bank must stay resident as long as memmap points at it. memmap is only
 * re-pointed inside super_switch(); sdpg_switch_begin() (called at the top of
 * super_switch) bumps a generation so every bank resolved during one switch is
 * pinned and cannot be evicted by another resolve in the same switch.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Allocate the 32 KB slot pool. Call this as early as possible in app_main()
 * (before LCD/SD bring-up) so the contiguous internal RAM is still free.
 * Safe to call again (no-op once reserved). Returns false on OOM. */
bool     sdpg_reserve_pool(void);

/* Reserve the pool (if not already) and open/create the read-write NOR file.
 * num_nor_pages is the NOR image size in 32 KB banks (0x10 for nc1020). */
bool     sdpg_init(const char *norPath, uint32_t num_nor_pages);

/* True once the slot pool is allocated (emulator can run). */
bool     sdpg_ready(void);

/* Open + size-check the read-only 12 MB ROM file (nc1020/pc1000 only). */
bool     sdpg_open_rom(const char *romPath);

/* Resolve one 8 KB sub-page (sub 0..3 = offset sub*0x2000) of a ROM bank
 * (volume 0/1/2, hw bank_idx in [0x80,0xFF]) to a resident slot. Read-only. */
uint8_t *sdpg_rom_sub(int volume, uint8_t bank_idx, int sub);

/* Resolve one 8 KB sub-page (sub 0..3) of NOR bank (0..num_nor_pages-1). */
uint8_t *sdpg_nor_sub(uint8_t nor_bank, int sub);

/* True if p points inside a slot currently holding a NOR page. */
bool     sdpg_ptr_is_nor(const uint8_t *p);

/* Mark the NOR slot containing p dirty (call after writing NOR through memmap). */
void     sdpg_mark_dirty_ptr(const uint8_t *p);

/* Erase every NOR bank to 0xFF and persist (NOR mass erase). */
void     sdpg_nor_mass_erase(void);

/* Begin a new bus-switch generation (pin guard). Call at top of super_switch(). */
void     sdpg_switch_begin(void);

/* Flush all dirty NOR slots to the .nor file. Cheap when nothing is dirty. */
void     sdpg_flush(void);

/* Total number of SD page reads (cache misses) since boot — for profiling. */
uint32_t sdpg_read_count(void);

#ifdef __cplusplus
}
#endif
