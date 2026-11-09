/* CW beacon: Morse keying, envelope ramp and NCO tables (portable: also compiled on the PC by host/test_cw.c).
 *
 * The keying is a list of runs in blocks of CW_BLOCK DAC samples: > 0 key down, < 0 key up, the length in blocks. One cycle is the
 * text (PARIS timing: dit 1 unit, dah 3, gap inside a character 1, between characters 3, between words 7), a word gap, the long
 * carrier, a word gap; the beacon repeats it. The envelope follows the key one step per block along a raised-cosine ramp of
 * R blocks: it rises from key down and falls from key up, so the 50 % points keep the length of the element.
 *
 * NCO: a 32-bit phase accumulator, the top CW_SIN_BITS bits index two tables (Q13, I = cos, Q = g sin(x + phi) with the I/Q gain
 * and phase correction of the beacon baked in); I = (a * cos) >> 21 with the amplitude a in Q8 DAC codes. DAC word: I bits 9:0,
 * Q bits 19:10, 10-bit two's complement. */
#ifndef CW_LUT_H
#define CW_LUT_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#define CW_BLOCK     16u                 /* DAC samples per envelope step */
#define CW_SIN_BITS  12u
#define CW_SIN_N     (1u << CW_SIN_BITS)
#define CW_TEXT_MAX  64u

/* International Morse code: '.' dit, '-' dah; NULL for characters without one (they are left out) */
static const char *cw_code(char c) {
    static const char *const az[26] = {".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..", "--",
                                       "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--.."};
    static const char *const d09[10] = {"-----", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----."};
    if (c >= 'a' && c <= 'z') c -= 'a' - 'A';
    if (c >= 'A' && c <= 'Z') return az[c - 'A'];
    if (c >= '0' && c <= '9') return d09[c - '0'];
    switch (c) {
    case '/': return "-..-.";
    case '?': return "..--..";
    case '.': return ".-.-.-";
    case ',': return "--..--";
    case '=': return "-...-";
    case '-': return "-....-";
    case '+': return ".-.-.";
    default: return 0;
    }
}

/* Appends a run (same sign: merged). Returns 0 when the list is full. */
static int cw_run_add(int32_t *runs, uint32_t max, uint32_t *n, int32_t len) {
    if (!len) return 1;
    if (*n && (runs[*n - 1] > 0) == (len > 0)) { runs[*n - 1] += len; return 1; }
    if (*n == max) return 0;
    runs[(*n)++] = len;
    return 1;
}

/* One beacon cycle: text, word gap, carrier (carrier blocks, 0: none), word gap (without text: carrier, word gap). unit: blocks per dit. Returns the number of runs,
 * 0 when they do not fit or there is nothing to send. Spaces are word gaps; characters without a code are left out. */
static uint32_t cw_build_runs(int32_t *runs, uint32_t max, const char *text, int32_t unit, int32_t carrier) {
    uint32_t n = 0;
    int ok = 1, gap = 0;                                       /* gap: units of key up owed before the next element */
    for (const char *p = text; *p && ok; ++p) {
        if (*p == ' ') { if (n) gap = 7; continue; }
        const char *c = cw_code(*p);
        if (!c) continue;
        for (; *c && ok; ++c) {
            ok = cw_run_add(runs, max, &n, -gap * unit) && cw_run_add(runs, max, &n, (*c == '-' ? 3 : 1) * unit);
            gap = 1;
        }
        gap = 3;
    }
    if (!ok) return 0;
    if (carrier > 0) ok = (!n || cw_run_add(runs, max, &n, -7 * unit)) && cw_run_add(runs, max, &n, carrier);
    ok = ok && cw_run_add(runs, max, &n, -7 * unit);
    if (!ok) return 0;
    for (uint32_t k = 0; k < n; ++k)
        if (runs[k] > 0) return n;
    return 0;
}

/* Raised-cosine ramp, R + 1 entries: amplitude in Q8 DAC codes for envelope step 0 (off) .. R (full). */
static void cw_ramp(int32_t *t, uint32_t R, float amp) {
    const float pi = 3.14159265f;
    for (uint32_t k = 0; k <= R; ++k) t[k] = (int32_t)lroundf(amp * 256.0f * 0.5f * (1.0f - cosf(pi * (float)k / (float)R)));
}

/* NCO tables, Q13: ci = cos x, cq = g sin(x + phi) (the beacon's I/Q correction Q' = g (Q cos phi + I sin phi)). */
static void cw_sin_tables(int16_t *ci, int16_t *cq, float g, float phi_deg) {
    const float pi = 3.14159265f, ph = phi_deg * pi / 180.0f;
    for (uint32_t k = 0; k < CW_SIN_N; ++k) {
        const float x = 2.0f * pi * (float)k / (float)CW_SIN_N;
        ci[k] = (int16_t)lroundf(8192.0f * cosf(x));
        cq[k] = (int16_t)lroundf(8192.0f * g * sinf(x + ph));
    }
}

/* DC correction spread over the CW_BLOCK samples of a block by error feedback: whole codes, the mean exact to 1 / CW_BLOCK code. */
static void cw_dc_pattern(int32_t *d, float dc) {
    float e = 0.0f;
    for (uint32_t j = 0; j < CW_BLOCK; ++j) {
        d[j] = (int32_t)lroundf(dc + e);
        e += dc - (float)d[j];
    }
}

/* The DAC word of one sample (the loop in main.c computes the same inline). */
static inline uint32_t cw_word(int32_t a, uint32_t phase, const int16_t *ci, const int16_t *cq, int32_t di, int32_t dq) {
    const uint32_t k = phase >> (32u - CW_SIN_BITS);
    const int32_t i = ((a * ci[k] + (1 << 20)) >> 21) + di, q = ((a * cq[k] + (1 << 20)) >> 21) + dq;
    return ((uint32_t)i & 0x3FFu) | (((uint32_t)q & 0x3FFu) << 10);
}

#endif
