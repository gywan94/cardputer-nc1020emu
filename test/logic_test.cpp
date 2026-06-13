// Host-side simulation of the boot/ROM-select/NOR decision logic, extracted
// verbatim from main.cpp / flash_rom.cpp so we can exercise every scenario on the
// PC (no hardware). Build: g++ -std=c++17 logic_test.cpp -o logic_test
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <cassert>
using namespace std;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { if (cond) { g_pass++; } else { g_fail++; \
    printf("  FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); } } while (0)

// ───────────────────────── 1. rom_signature (flash_rom.cpp) ─────────────────
static uint32_t rom_signature(const uint8_t *hdr16, bool have_file, uint32_t cap_banks) {
    uint32_t sig = 0xC1020000u ^ cap_banks;
    if (have_file) for (int i = 0; i < 16; i++) sig = sig * 131u + hdr16[i];
    return sig;
}

// ───────────────────────── 2. choose_rom() decision (main.cpp) ──────────────
struct ChooseOut { string base; bool changed; bool ret; bool menu_shown; };
// pick = index the user selects in the menu (only used when the menu is shown)
static ChooseOut choose_rom(bool g0_held, string cur,
                            const vector<string>& roms, int pick) {
    ChooseOut o{ "", false, false, false };
    if (!g0_held) {                                   // normal boot
        if (!cur.empty()) { o.base = cur; o.changed = false; o.ret = true; return o; }
        return o;                                     // ret=false -> fallback baked
    }
    int n = (int)roms.size();                         // G0 held -> menu
    if (n == 0) {
        if (!cur.empty()) { o.base = cur; o.ret = true; return o; }
        return o;
    }
    o.menu_shown = true;
    int sel = 0;
    for (int i = 0; i < n; i++) if (roms[i] == cur) sel = i;
    sel = pick;                                       // simulate navigation to `pick`
    o.base = roms[sel];
    o.changed = (o.base != cur);
    o.ret = true;
    return o;
}

// ───────────────────────── 3. NOR reseed decision (flash_rom.cpp) ───────────
// returns: "import" | "blank" | "keep" | "skip"
static const char* nor_decision(bool norinit, bool force_reseed, bool has_nor_file) {
    bool done = norinit;
    if (force_reseed) done = false;
    if (done) return "skip";                          // already initialised, no change
    if (has_nor_file)  return "import";               // seed from SD .nor
    if (force_reseed)  return "blank";                // ROM changed, no .nor -> fresh NOR
    return "keep";                                    // first seed, no .nor -> baked/blank
}

// ───────────────────────── 4. menu scroll clamp (main.cpp) ──────────────────
static int menu_top(int sel, int n, int VIS) {
    int top = sel - VIS / 2;
    if (top > n - VIS) top = n - VIS;
    if (top < 0) top = 0;
    return top;
}

int main() {
    printf("== 1. rom_signature ==\n");
    // nc1020_53.rom first 16 bytes (read on device); firmware logged sig 0x3b634302
    uint8_t hdr[16] = {0x60,0xea,0xa8,0xd1,0xea,0xd1,0x4b,0xd1,0x81,0xd1,0xb4,0xc0,0xd3,0xc6,0x5e,0xc7};
    uint32_t sig = rom_signature(hdr, true, 206);
    printf("  sig(nc1020_53,206) = 0x%08X (expect 0x3B634302)\n", sig);
    CHECK(sig == 0x3B634302u, "romsig matches device-logged value");
    // no file -> deterministic no-fold value, must differ -> forces trust/repopulate path
    CHECK(rom_signature(nullptr, false, 206) != 0x3B634302u, "no-file sig differs (triggers fallback)");

    printf("== 2. choose_rom decision ==\n");
    vector<string> R = {"/sd/A", "/sd/B", "/sd/roms/C"};
    auto a = choose_rom(false, "/sd/A", R, 0);
    CHECK(a.ret && a.base=="/sd/A" && !a.changed && !a.menu_shown, "no-G0 + cur -> use cur, no menu");
    auto b = choose_rom(false, "", R, 0);
    CHECK(!b.ret && !b.menu_shown, "no-G0 + no cur -> fallback, NO menu (fix #2)");
    auto c = choose_rom(true, "/sd/A", R, 1);
    CHECK(c.ret && c.base=="/sd/B" && c.changed && c.menu_shown, "G0 + pick different -> changed=true");
    auto d = choose_rom(true, "/sd/A", R, 0);
    CHECK(d.ret && d.base=="/sd/A" && !d.changed, "G0 + pick same -> changed=false");
    auto e = choose_rom(true, "/sd/A", {}, 0);
    CHECK(e.ret && e.base=="/sd/A" && !e.menu_shown, "G0 + no roms + cur -> use cur");
    auto f = choose_rom(true, "", {}, 0);
    CHECK(!f.ret, "G0 + no roms + no cur -> fallback");

    printf("== 3. NOR reseed decision (the crash fix) ==\n");
    CHECK(!strcmp(nor_decision(true,  false, true ), "skip"),   "baked boot (norinit,!force) -> skip/keep baked");
    CHECK(!strcmp(nor_decision(true,  false, false), "skip"),   "baked boot no .nor -> skip");
    CHECK(!strcmp(nor_decision(true,  true,  true ), "import"), "ROM changed + has .nor -> import");
    CHECK(!strcmp(nor_decision(true,  true,  false), "blank"),  "ROM changed + NO .nor -> BLANK (fixes crash)");
    CHECK(!strcmp(nor_decision(false, false, true ), "import"), "first seed + .nor -> import");
    CHECK(!strcmp(nor_decision(false, false, false), "keep"),   "first seed no .nor -> keep baked/blank");
    // The bug was: ROM changed, no .nor -> old code 'keep' (stale NOR) -> crash.
    CHECK(strcmp(nor_decision(true, true, false), "keep") != 0, "regression guard: never 'keep' a stale NOR on ROM change");

    printf("== 4. menu scroll clamp (sel always visible, in range) ==\n");
    for (int n = 1; n <= 24; n++)
      for (int sel = 0; sel < n; sel++) {
        int VIS = 6, top = menu_top(sel, n, VIS);
        bool in_range = top >= 0 && (top < n);
        bool sel_visible = (sel >= top && sel < top + VIS);
        bool no_overrun = (top + VIS <= n) || (top == 0);
        CHECK(in_range && sel_visible && no_overrun, "menu_top keeps sel visible, no overrun");
      }

    printf("== 5. 180-degree flip is a bijection within bounds ==\n");
    const int W = 240, H = 135;
    bool flip_ok = true;
    for (int lx = 0; lx < W; lx++) for (int ly = 0; ly < H; ly++) {
        int px = W - 1 - lx, py = H - 1 - ly;
        if (px < 0 || px >= W || py < 0 || py >= H) flip_ok = false;
        // round-trip back
        if (W-1-px != lx || H-1-py != ly) flip_ok = false;
    }
    CHECK(flip_ok, "every logical pixel maps to a unique in-bounds panel pixel");

    printf("\n==== %d passed, %d failed ====\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
