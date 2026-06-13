// Host simulation of draw_lcd() (main.cpp) — verifies the full-160, 1:1, 180deg
// mapping keeps horizontal lines horizontal, covers the panel without overlap/gap,
// and never reads/writes out of bounds. No hardware. g++ -std=c++17 draw_test.cpp
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
using namespace std;

static int g_pass = 0, g_fail = 0;
#define CHECK(c,m) do{ if(c) g_pass++; else { g_fail++; printf("  FAIL: %s (%d)\n",m,__LINE__);} }while(0)

// ── constants mirrored verbatim from main.cpp ───────────────────────────────
static const int CP_LCD_W = 240, CP_LCD_H = 135;
static const int LCD_SRC_W = 160, LCD_SRC_H = 80, LCD_DRAW_W = 160;

// One emitted scanline from draw_lcd: a horizontal run [x0,x0+w) on panel row prow,
// carrying `w` source pixels (already column-reversed for the 180deg flip).
struct Span { int prow, x0, w; vector<int> srccol; int srcrow; };

// Faithful re-implementation of draw_lcd's geometry (values, not colors).
static vector<Span> draw_lcd_sim(const uint8_t* fb, bool grey, bool* oob) {
    const int X0 = (CP_LCD_W - LCD_DRAW_W) / 2;   // 40
    const int Y0 = (CP_LCD_H - LCD_SRC_H) / 2;    // 27
    *oob = false;
    vector<Span> out;
    for (int r = 0; r < LCD_SRC_H; r++) {
        const int base = r * LCD_SRC_W;
        Span s; s.prow = CP_LCD_H - 1 - (Y0 + r); s.x0 = X0; s.w = LCD_DRAW_W; s.srcrow = r;
        s.srccol.assign(LCD_DRAW_W, 0);
        for (int j = 0; j < LCD_DRAW_W; j++) {
            int pos = base + j;                    // source col 0..159
            if (pos < 0 || pos >= LCD_SRC_W * LCD_SRC_H) *oob = true;     // fb pixel index
            int byteIdx = grey ? (pos >> 2) : (pos >> 3);
            int bufBytes = LCD_SRC_W * LCD_SRC_H / 8 * (grey ? 2 : 1);
            if (byteIdx < 0 || byteIdx >= bufBytes) *oob = true;          // fb byte index
            (void)fb;
            s.srccol[LCD_DRAW_W - 1 - j] = j;      // 180deg: panel x carries source col j reversed
        }
        if (s.prow < 0 || s.prow >= CP_LCD_H) *oob = true;
        if (s.x0 < 0 || s.x0 + s.w > CP_LCD_W) *oob = true;
        out.push_back(s);
    }
    return out;
}

int main() {
    // synthetic framebuffer: alternating dark/light horizontal lines (1bpp)
    uint8_t fb[LCD_SRC_W * LCD_SRC_H / 8];
    for (int r = 0; r < LCD_SRC_H; r++)
        for (int c = 0; c < LCD_SRC_W; c++) {
            int pos = r * LCD_SRC_W + c, bit = (r & 1);  // every odd row dark
            if (bit) fb[pos >> 3] |= (0x80 >> (pos & 7));
            else     fb[pos >> 3] &= ~(0x80 >> (pos & 7));
        }

    bool oob = false;
    auto spans = draw_lcd_sim(fb, false, &oob);

    printf("== 1. no out-of-bounds fb read / panel write ==\n");
    CHECK(!oob, "all fb indices and panel coords in range");

    printf("== 2. exactly 80 spans, one per source row ==\n");
    CHECK((int)spans.size() == LCD_SRC_H, "80 emitted scanlines");

    printf("== 3. each source row -> exactly one panel row (lines stay level) ==\n");
    // Count panel rows touched and ensure 1:1: no source row shares a panel row,
    // no source row spans 2 panel rows (the ragged-line failure mode of scaling).
    int rowcount[CP_LCD_H]; memset(rowcount, 0, sizeof(rowcount));
    bool one_to_one = true;
    for (auto& s : spans) { rowcount[s.prow]++; if (s.w != LCD_SRC_W) one_to_one = false; }
    int touched = 0, doubled = 0;
    for (int y = 0; y < CP_LCD_H; y++) { if (rowcount[y]) touched++; if (rowcount[y] > 1) doubled++; }
    CHECK(one_to_one, "every span is exactly 160px wide (1:1, no horizontal stretch)");
    CHECK(touched == LCD_SRC_H, "80 distinct panel rows used (no two source rows collide)");
    CHECK(doubled == 0, "no panel row written twice (no source row split across rows)");

    printf("== 4. panel rows are contiguous 28..107, centred ==\n");
    int ymin = 999, ymax = -1;
    for (auto& s : spans) { if (s.prow < ymin) ymin = s.prow; if (s.prow > ymax) ymax = s.prow; }
    CHECK(ymin == 28 && ymax == 107, "vertical band 28..107 (centred 80 of 135)");
    bool contiguous = true;
    for (int y = ymin; y <= ymax; y++) if (rowcount[y] != 1) contiguous = false;
    CHECK(contiguous, "no gaps in the vertical band");

    printf("== 5. horizontal lines remain horizontal ==\n");
    // A dark source row must produce a span on a single panel row whose pixels are
    // ALL dark — a level line. Check parity is preserved per panel row.
    bool lines_level = true;
    for (auto& s : spans) {
        int sr = s.srcrow; int expect = (sr & 1);   // 1 = dark
        for (int j = 0; j < s.w; j++) {
            int srccol = s.srccol[j]; int pos = sr * LCD_SRC_W + srccol;
            int val = (fb[pos >> 3] >> (7 - (pos & 7))) & 1;
            if (val != expect) lines_level = false;  // a pixel breaking the row = slant/noise
        }
    }
    CHECK(lines_level, "each panel row is uniformly the source row's value (perfectly level)");

    printf("== 6. 180deg column flip is a bijection over 0..159 ==\n");
    bool bij = true; vector<int> seen(LCD_DRAW_W, 0);
    for (auto& s : spans) {
        if (s.srcrow != 0) continue;
        for (int x = 0; x < s.w; x++) { int sc = s.srccol[x]; if (sc<0||sc>=160) bij=false; else seen[sc]++; }
    }
    for (int i = 0; i < LCD_DRAW_W; i++) if (seen[i] != 1) bij = false;
    CHECK(bij, "all 160 source columns map to exactly one panel x each");

    printf("== 7. full width used, centred 40..199 ==\n");
    int xmin = 999, xmax = -1;
    for (auto& s : spans) { if (s.x0 < xmin) xmin = s.x0; if (s.x0 + s.w - 1 > xmax) xmax = s.x0 + s.w - 1; }
    CHECK(xmin == 40 && xmax == 199, "horizontal band 40..199 (full 160, centred)");

    printf("\n==== %d passed, %d failed ====\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
