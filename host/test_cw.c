/* Checks the CW beacon tables and keying (firmware/main/cw_lut.h).
 *   gcc -O2 -o /tmp/test_cw test_cw.c -lm && /tmp/test_cw [out.bin [nco_hz [amp [text]]]]
 * Prints PASS / FAIL per check; the optional dump holds one beacon cycle of int16 I,Q pairs at CW_FS (the DAC words as the loop in
 * main.c makes them, 2 ms per unit so that a cycle stays short) for a spectrum check. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#pragma GCC diagnostic ignored "-Wunused-function"          /* the SSB part of the header */
#include "../firmware/main/cw_lut.h"

#define CW_FS 2000000.0

static int fails;
static void check(int ok, const char *what) {
    printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    fails += !ok;
}

static int32_t total(const int32_t *r, uint32_t n, int sign) {
    int32_t s = 0;
    for (uint32_t k = 0; k < n; ++k)
        if ((r[k] > 0) == (sign > 0)) s += r[k] > 0 ? r[k] : -r[k];
    return s;
}

int main(int argc, char **argv) {
    int32_t runs[1024];
    /* "PARIS " is 50 units: 22 on, 28 off (with the closing word gap) */
    uint32_t n = cw_build_runs(runs, 1024, "PARIS", 1, 0);
    check(n > 0 && total(runs, n, 1) == 22 && total(runs, n, -1) == 28, "PARIS = 50 units, 22 key down");
    check(runs[0] == 1 && runs[1] == -1 && runs[2] == 3, "P starts dit, gap, dah");
    for (uint32_t k = 1; k < n; ++k)
        if ((runs[k] > 0) == (runs[k - 1] > 0)) { check(0, "runs alternate"); break; }
    /* word gap and carrier: "E E" = dit, 7 gap, dit; then 7 gap, carrier 100, 7 gap */
    n = cw_build_runs(runs, 1024, "E E", 2, 100);
    check(n == 6 && runs[0] == 2 && runs[1] == -14 && runs[2] == 2 && runs[3] == -14 && runs[4] == 100 && runs[5] == -14, "word gap, carrier, scaled unit");
    check(cw_build_runs(runs, 1024, " #  ", 1, 0) == 0, "nothing to send: 0");
    check(cw_build_runs(runs, 1024, "#", 1, 50) == 2, "carrier only");
    check(cw_build_runs(runs, 4, "VVV", 1, 0) == 0, "too many runs: 0");
    check(cw_build_runs(runs, 1024, "vvv de ok1cdj", 1, 0) == cw_build_runs(runs, 1024, "VVV DE OK1CDJ", 1, 0), "lower case");

    /* ramp: 0 .. amp, monotonic, half at R / 2 */
    int32_t ramp[401];
    cw_ramp(ramp, 400, 100.0f);
    int mono = 1;
    for (int k = 1; k <= 400; ++k) mono &= ramp[k] >= ramp[k - 1];
    check(ramp[0] == 0 && ramp[400] == 25600 && abs(ramp[200] - 12800) <= 1 && mono, "raised-cosine ramp");

    /* words: full amplitude reaches amp, DC pattern averages to the fraction */
    static int16_t ci[CW_SIN_N], cq[CW_SIN_N];
    cw_sin_tables(ci, cq, 1.0f, 0.0f);
    int32_t di[CW_BLOCK], dq[CW_BLOCK];
    cw_dc_pattern(di, 2.3f);
    cw_dc_pattern(dq, -1.75f);
    double si = 0, sq = 0;
    for (uint32_t j = 0; j < CW_BLOCK; ++j) { si += di[j]; sq += dq[j]; }
    check(fabs(si / CW_BLOCK - 2.3) <= 1.0 / CW_BLOCK && fabs(sq / CW_BLOCK + 1.75) <= 1.0 / CW_BLOCK, "DC pattern mean");
    int imax = 0, qmax = 0;
    for (uint32_t k = 0; k < CW_SIN_N; ++k) {
        const uint32_t w = cw_word(480 * 256, k << (32 - CW_SIN_BITS), ci, cq, 0, 0);
        const int i = (int)((w & 0x3FF) ^ 0x200) - 512, q = (int)(((w >> 10) & 0x3FF) ^ 0x200) - 512;
        if (abs(i) > imax) imax = abs(i);
        if (abs(q) > qmax) qmax = abs(q);
    }
    check(imax == 480 && qmax == 480, "amplitude 480 without overflow");
    cw_sin_tables(ci, cq, 1.3f, 40.0f);                          /* the largest correction QPSKT accepts: still no overflow */
    qmax = 0;
    for (uint32_t k = 0; k < CW_SIN_N; ++k) {
        const uint32_t w = cw_word(380 * 256, k << (32 - CW_SIN_BITS), ci, cq, 0, 0);
        const int q = (int)(((w >> 10) & 0x3FF) ^ 0x200) - 512;
        if (abs(q) > qmax) qmax = abs(q);
    }
    check(qmax == 494, "Q gain 1.3 at amplitude 380");

    if (argc > 1) {                                              /* one cycle as the loop makes it */
        const double nco = argc > 2 ? atof(argv[2]) : 20000.0;
        const float amp = argc > 3 ? (float)atof(argv[3]) : 100.0f;
        const char *text = argc > 4 ? argv[4] : "VVV";
        const uint32_t unit = (uint32_t)(0.002 * CW_FS / CW_BLOCK + 0.5), R = (uint32_t)(0.0005 * CW_FS / CW_BLOCK + 0.5);
        n = cw_build_runs(runs, 1024, text, (int32_t)unit, (int32_t)(10 * unit));
        int32_t *rt = malloc((R + 1) * sizeof(int32_t));
        cw_ramp(rt, R, amp);
        cw_sin_tables(ci, cq, 1.0f, 0.0f);
        cw_dc_pattern(di, 0.4f);
        cw_dc_pattern(dq, -0.3f);
        const uint32_t inc = (uint32_t)(int32_t)llround(nco / CW_FS * 4294967296.0);
        FILE *f = fopen(argv[1], "wb");
        uint32_t ph = 0, e = 0, nsamp = 0;
        for (uint32_t r = 0; r < n; ++r)
            for (int32_t b = 0; b < abs(runs[r]); ++b) {
                if (runs[r] > 0) { if (e < R) ++e; } else if (e) --e;
                for (uint32_t j = 0; j < CW_BLOCK; ++j) {
                    const uint32_t w = cw_word(rt[e], ph, ci, cq, di[j], dq[j]);
                    ph += inc;
                    const int16_t iq[2] = {(int16_t)((int)((w & 0x3FF) ^ 0x200) - 512), (int16_t)((int)(((w >> 10) & 0x3FF) ^ 0x200) - 512)};
                    fwrite(iq, sizeof(iq), 1, f);
                    ++nsamp;
                }
            }
        fclose(f);
        free(rt);
        printf("wrote %u samples at %.0f S/s, NCO %.3f Hz, %u runs\n", nsamp, CW_FS, nco, n);
    }
    printf("%s\n", fails ? "FAILED" : "all passed");
    return fails != 0;
}
