/*
 * trig.c - sin and cos together, sqrt, atan2 and angle wrapping.  See trig.h.
 *
 * Three of the five are written for THIS program rather than taken from
 * fmath.h, because the swarm spent a third of its time in them:
 *
 *   t_sincos  one call for both.  The airframe wants the sin AND the cos of
 *             every angle it has; two f_sin() calls cost 12 multiplies and
 *             9 adds, this costs 7 and 5 - and is more exact.
 *   t_sqrt    whole-number arithmetic, no float operation at all: 330 cycles
 *             where three Newton steps (three float divisions) took 1030,
 *             and the answer is exact.
 *   t_wrap    f_wrap(), but "is it in range already?" is asked of the bits.
 *
 * A float's bits (sign, exponent, fraction - softfloat.c draws the picture)
 * are read through a union.  Two facts about them do most of the work here:
 * clear the sign bit and you have |x|; and the bit patterns of positive
 * floats sort in the same order as the numbers, so "|x| <= pi" is one
 * integer compare, where a float compare is a ~25-cycle library call.
 */
#include <stdint.h>
#include "trig.h"
#include "fmath.h"

typedef union { float f; uint32_t u; } bits_t;

#define SIGN_BIT     0x80000000u
#define PI_4_BITS    0x3F490FDBu          /* the bits of the float pi/4   */
#define PI_3_4_BITS  0x4016CBE4u          /*                    3 pi/4    */
#define PI_BITS      0x40490FDBu          /*                    pi        */

float t_atan2(float y, float x) { return f_atan2(y, x); }

/* into -pi .. pi.  Nearly every angle here is in range already. */
float t_wrap(float a)
{
    bits_t v = { a };
    return (v.u & ~SIGN_BIT) <= PI_BITS ? a : f_wrap(a);
}

/* ---- sin and cos, together ---------------------------------------------------
 * 1. Fold the angle: sin is odd and cos is even, so work on |a|; then take
 *    off the nearest multiple of pi/2, leaving x in -pi/4 .. pi/4.
 * 2. On that short range short polynomials are enough.  These are not
 *    Taylor's (best at 0, worst at the ends) but MINIMAX fits, which spread
 *    the error evenly: sin to x^5 is good to 1.0e-6, cos to x^6 to 1.1e-7.
 *    (f_sin's Taylor series to x^9, on its range of +-pi/2: 3.7e-6.)
 * 3. Unfold: a quarter turn swaps the two, a half turn flips both signs. */
#define S1  (-0.16662835f)
#define S2  ( 0.008152995f)
#define C1  (-0.49999896f)
#define C2  ( 0.041656293f)
#define C3  (-0.0013597824f)

void t_sincos(float a, float *sn, float *cs)
{
    bits_t v = { a };
    uint32_t neg = v.u & SIGN_BIT;
    v.u ^= neg;                                    /* |a|                      */
    if (v.u > PI_BITS) {                           /* more than half a turn    */
        v.f = f_wrap(a);
        neg = v.u & SIGN_BIT;
        v.u ^= neg;
    }
    int quarters = 0;
    if (v.u > PI_4_BITS) {
        if (v.u <= PI_3_4_BITS) { v.f -= F_PI_2;  quarters = 1; }
        else                    { v.f -= F_PI;    quarters = 2; }
    }
    float x = v.f, x2 = x * x;
    bits_t s = { x * (1.0f + x2 * (S1 + x2 * S2)) };
    bits_t c = { 1.0f + x2 * (C1 + x2 * (C2 + x2 * C3)) };
    if (quarters == 1) {                           /* sin(x + pi/2) =  cos x   */
        bits_t t = s;  s = c;  c = t;              /* cos(x + pi/2) = -sin x   */
        c.u ^= SIGN_BIT;
    } else if (quarters == 2) {                    /* a half turn: both flip   */
        s.u ^= SIGN_BIT;  c.u ^= SIGN_BIT;
    }
    s.u ^= neg;                                    /* sin(-a) = -sin a         */
    *sn = s.f;  *cs = c.f;
}

/* ---- sqrt, in whole numbers ----------------------------------------------------
 * x = m * 2^e with m a 24-bit whole number.  Make e even (double m if it is
 * odd) and sqrt(x) = sqrt(m) * 2^(e/2): only the root of a whole number is
 * left, and that is the schoolbook method - bring down two bits of m at a
 * time, try the next bit of the root, keep it if it fits.  26 steps give the
 * 24 bits of the answer and what is needed to round it.  The result is the
 * correctly rounded square root, the same on the chip and on the PC.
 * x <= 0 gives 0 (as f_sqrt does). */
float t_sqrt(float x)
{
    bits_t v = { x };
    if ((int32_t)v.u <= 0)   { return 0.0f; }
    if (v.u >= 0x7F800000u)  { return x; }         /* inf, NaN                 */
    int32_t e = (int32_t)(v.u >> 23) - 150;        /* x = m * 2^e              */
    uint32_t m = v.u & 0x007FFFFFu;
    if (e > -150) { m |= 0x00800000u; }
    else { e++;  while (!(m & 0x00800000u)) { m <<= 1;  e--; } }   /* subnormal */
    if (e & 1) { m <<= 1;  e--; }

    /* The root of m * 2^26, a 52-bit number whose low 20 bits are all 0.
     * First its top 32 bits, m << 6, all at once: 16 bits of root, in the
     * tightest form of the method (`one` is the bit being tried, already
     * squared; rem ends as what is left over). */
    uint32_t rem = m << 6, root = 0;
    for (uint32_t one = 1u << 30; one; one >>= 2) {
        uint32_t trial = root + one;
        root >>= 1;
        if (rem >= trial) { rem -= trial;  root += one; }
    }
    /* Then ten more bits, bringing down two zeros a step. */
    for (int k = 0; k < 10; k++) {
        rem <<= 2;
        uint32_t trial = (root << 2) | 1u;
        root <<= 1;
        if (rem >= trial) { rem -= trial;  root |= 1u; }
    }
    /* root = sqrt(m) * 2^13, 25 or 26 bits.  Keep 24; the next bit down says
     * which way to round (a square root is never exactly halfway). */
    e = e / 2 + 137;                               /* the result's exponent field - 1 */
    if (root >> 25) { root = (root >> 2) + ((root >> 1) & 1u);  e++; }
    else            { root = (root >> 1) + (root & 1u); }
    v.u = ((uint32_t)e << 23) + root;              /* root's top bit adds the 1 */
    return v.f;
}
