/*
 * Tab5 port stubs — replaces the NC2000 desktop layer (SDL window/timer, DSP
 * audio chip, debug console, UART, disassembler, NekoDriver, cmd) with no-ops
 * so the platform-independent 6502 core compiles + links on ESP-IDF.
 * Real audio/UART can be wired to the Tab5 BSP later; for now they're silent.
 */
#include <stdint.h>
#include <string>
#include "esp_timer.h"
#include "dsp/dsp.h"
#include "comm.h"
#include "state.h"

extern nc2k_states_t nc2k_states;

/* The global DSP/sound-chip instance the core links against (io_new.cpp does
 * `extern Dsp dsp;`). Stubbed silent for now. */
Dsp dsp;

/* ---- SDL shim ---- */
extern "C" uint32_t SDL_GetTicks(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
extern "C" uint64_t SDL_GetTicks64(void) { return (uint64_t)(esp_timer_get_time() / 1000); }
extern "C" void SDL_SetWindowTitle(struct SDL_Window *w, const char *t) { (void)w; (void)t; }

/* ---- sound: Phase 1 = beeper -> cross-core ring -> Core1 I2S (main.cpp) ----
 * DSP voice playback stays silent (Phase 2; needs ~100% realtime to sound right). */
extern "C" void audio_push_samples(const int16_t *s, int n);   /* main.cpp ring */

/* The firmware drives a 1-bit beeper level; emit square-wave samples in EMULATED-
 * cycle time (count = cycle-delta scaled to BEEPER_AUDIO_HZ) so the PITCH is right
 * regardless of the ~70% emulation rate. count is capped and the ring drops on
 * full, so a huge cycle jump (e.g. right after load_state) can't flood Core 0.
 * Core-0 only: both entry points run inside the 6502 execution path. */
static long long s_beep_cycle = 0;
static int       s_beep_value = 0;

static void beeper_emit(int next) {
    long long cur = (long long)nc2k_states.cycles;
    long long s0 = s_beep_cycle * (long long)BEEPER_AUDIO_HZ / (long long)CYCLES_SECOND;
    long long s1 = cur          * (long long)BEEPER_AUDIO_HZ / (long long)CYCLES_SECOND;
    s_beep_cycle = cur;
    long long count = s1 - s0;
    if (count < 0) count = 0;
    if (count > BEEPER_AUDIO_HZ / 10) count = BEEPER_AUDIO_HZ / 10;   /* cap ~100ms */
    if (count > 0) {
        int16_t buf[128];
        /* NekoDriverIO passes a = (value&0x80)?128:-1, so map by sign to a clean
         * bipolar +/-7000 square (no DC offset). */
        int16_t v = (s_beep_value > 0) ? 7000 : -7000;
        long long left = count;
        while (left > 0) {
            int n = left < 128 ? (int)left : 128;
            for (int i = 0; i < n; i++) buf[i] = v;
            audio_push_samples(buf, n);
            left -= n;
        }
    }
    s_beep_value = next;
}

void beeper_on_io_write(int v) { beeper_emit(v); }
void post_cpu_run_sound_handling() { beeper_emit(s_beep_value); }
void reset_dsp() {}
void dsp_move(int len) { (void)len; }
void write_data_to_dsp(uint8_t a, uint8_t b) { (void)a; (void)b; }
bool sound_busy(void) { return false; }

/* ---- IV / UART: NOT stubbed — the real iv_uart.cpp is compiled. ----
 * History of why: the IV queue is the RTC interrupt system. A stubbed
 * peek_iv()==0 (=IV_2HZ) fired a spurious wake every cycle (sleep/wake loop,
 * never boots); a stubbed peek_iv()==IV_NONE with put_iv() a no-op swallowed
 * the 2Hz RTC tick, so the firmware slept (clk off) and NEVER woke — black
 * screen after 保存/清除flash, keys only flashing a frame via warm-reset.
 * The firmware NEEDS the real put_iv/peek_iv/RCR0/RCR1 wiring to sleep+wake. */

/* ---- console / cmd / debug (no interactive console on device) ---- */
bool console_on = false;
bool dummy_io_for_read(uint16_t addr, uint8_t &value) { (void)addr; (void)value; return false; }
bool dummy_io_for_write(uint16_t addr, uint8_t value) { (void)addr; (void)value; return false; }
bool is_nc2600_rom() { return false; }
/* Real implementation (was a no-op stub). The wqx firmware decides cold-vs-warm
 * boot at reset by: lda io_timer0_val; ora io_timer1_val; beq cold_start. So as
 * long as ram_io[2] or ram_io[3] is non-zero it takes the WARM path — it resumes
 * the saved session and repaints instead of running the cold-boot "资料整理中 /
 * 是否清除闪存" format flow. Must be called before warm_reset()'s cpu->reset(). */
void set_warm_reset_flag() {
    nc2k_states.ram_io[2] = 1;
    nc2k_states.ram_io[3] = 1;
}
void handle_cmd(std::string str) { (void)str; }
std::string get_message() { return std::string(); }
/* MUST return NULL (not ""): cpu_run3 does `if(peek_message()) split_s(...)[0]`,
 * and split_s("") yields an empty vector — indexing [0] would deref NULL. */
char* peek_message() { return nullptr; }

/* ---- disassembler (debug only) ---- */
std::string disassemble2(uint16_t a) { (void)a; return std::string(); }
std::string disassemble_next(unsigned char* c, uint16_t a) { (void)c; (void)a; return std::string(); }
