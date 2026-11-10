/* Checks the SSB path of the CW beacon firmware (firmware/main/cw_lut.h): the loop's integer interpolation and NCO rotation.
 *   gcc -O2 -o /tmp/test_ssb test_ssb.c -lm && /tmp/test_ssb out.bin [tone1_hz [tone2_hz [amp [nco_hz]]]]
 * Feeds the analytic signal of one tone (tone2 0) or two equal tones at 62.5 kS/s, as the host sends it (int16, the sum at
 * 0.9 of full scale), through the arithmetic of ssb_loop() in main.c for 0.5 s and writes the int16 I, Q pairs of the DAC at
 * 2 MS/s (the words decoded) for a spectrum check; prints PASS / FAIL for the amplitude. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#pragma GCC diagnostic ignored "-Wunused-function"          /* the CW part of the header */
#include "../firmware/main/cw_lut.h"

#define FS 2000000.0
#define IN (FS / SSB_INTERP)

static int dac(uint32_t w, int field) {
    return (int)(((w >> (10 * field)) & 0x3FF) ^ 0x200) - 512;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: test_ssb out.bin [tone1 [tone2 [amp [nco]]]]\n"); return 2; }
    const double f1 = argc > 2 ? atof(argv[2]) : 1000.0, f2 = argc > 3 ? atof(argv[3]) : 0.0;
    const int amp = argc > 4 ? atoi(argv[4]) : 100;
    const double nco = argc > 5 ? atof(argv[5]) : 20000.0;
    static int16_t c[SSB_SIN_N], s[SSB_SIN_N], sp[SSB_SIN_N], cp[SSB_SIN_N];
    ssb_tables(c, s, sp, cp, 1.0f, 0.0f);
    int32_t di[CW_BLOCK], dq[CW_BLOCK];
    cw_dc_pattern(di, 0.0f);
    cw_dc_pattern(dq, 0.0f);
    const uint32_t inc = (uint32_t)(int32_t)llround(nco / FS * 4294967296.0);
    const uint32_t npairs = (uint32_t)(0.5 * IN);
    FILE *f = fopen(argv[1], "wb");
    int32_t u = 0, v = 0, du = 0, dv = 0;
    uint32_t phase = 0;
    int peak = 0;
    const double a = (f2 > 0 ? 0.45 : 0.9) * 32767.0;
    for (uint32_t n = 0; n < npairs; ++n) {
        /* the host's pair n: the analytic signal (USB: e^(+j w t)) */
        const double t = n / IN;
        double re = a * cos(2 * M_PI * f1 * t), im = a * sin(2 * M_PI * f1 * t);
        if (f2 > 0) { re += a * cos(2 * M_PI * f2 * t); im += a * sin(2 * M_PI * f2 * t); }
        const int32_t ti = (int16_t)lround(re), tq = (int16_t)lround(im);
        du = (ti * amp - u) / (int32_t)SSB_INTERP;           /* as ssb_loop(): the steps to the new pair */
        dv = (tq * amp - v) / (int32_t)SSB_INTERP;
        for (uint32_t k = 0; k < SSB_INTERP; ++k) {
            u += du;
            v += dv;
            phase += inc;
            const uint32_t w = ssb_word(u, v, phase, c, s, sp, cp, di[k & (CW_BLOCK - 1u)], dq[k & (CW_BLOCK - 1u)]);
            const int16_t iq[2] = {(int16_t)dac(w, 0), (int16_t)dac(w, 1)};
            if (abs(iq[0]) > peak) peak = abs(iq[0]);
            fwrite(iq, sizeof(iq), 1, f);
        }
    }
    fclose(f);
    const int ok = peak <= amp + 1 && peak >= amp * 0.85;
    if (f2 > 0) printf("%s peak %d codes for AMP %d (two tones %.0f + %.0f Hz, NCO %.0f Hz), %u pairs\n", ok ? "PASS" : "FAIL", peak, amp, f1, f2, nco, npairs);
    else printf("%s peak %d codes for AMP %d (one tone %.0f Hz, NCO %.0f Hz), %u pairs\n", ok ? "PASS" : "FAIL", peak, amp, f1, nco, npairs);
    return !ok;
}
