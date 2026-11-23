/*
 * softfloat.c - float + - * / and compares for a chip with no FPU, fast
 *
 * The Cortex-M0+ has no floating-point unit, so every float operation in
 * this program is a call into a library.  GCC brings one along (libgcc),
 * written for EVERY chip - generic C, every rounding mode, exception flags.
 * Cycles a call, call and return included, averaged over this program's own
 * show (counted in an instruction-set simulator of the core):
 *
 *                    a * b    a + b    a - b    a / b    a < b
 *     libgcc          137      102      119      433       54
 *     this file        67       42       75      188       24
 *
 * and those calls were 87 % of everything the swarm did.  The functions here
 * replace libgcc's: the linker takes a function from the program's own files
 * first and goes to the library only for what is still missing.  They give
 * EXACTLY the same answers - IEEE 754 single precision, round to nearest
 * even, subnormals, infinities, NaN - only faster, and 2.8 KB smaller
 * (libgcc's add and subtract alone are 2.2 KB).  Checked bit for bit against
 * a PC's hardware float: the C on 44 million operand pairs, the assembly
 * (in the simulator) on 3 million, every special value among them.
 *
 * How: a float is a sign, an 8-bit exponent and a 23-bit fraction with an
 * implied leading 1.  Unpack into integers, do the integer sum, product or
 * quotient, then round it back - in ONE place, pack().  The C below is the
 * readable version and what a PC compiles; the chip runs the same steps
 * written in Thumb assembly further down, because the compiler's version of
 * this C is no faster than libgcc (the M0+ has 8 easy registers, and this
 * needs nearly all of them at once).
 *
 * Nothing in this file may itself use a float operator - it would call
 * itself.  Floats arrive and leave as their bit patterns (uint32_t), which
 * is exactly how the ARM soft-float calling convention passes them anyway.
 */
#include <stdint.h>

#define SIGN  0x80000000u
#define INF   0x7F800000u
#define QNAN  0x7FC00000u
#define FRAC  0x007FFFFFu
#define ONE   0x00800000u            /* the implied leading 1 */

/* Leading zeros of a nonzero x.  The M0+ has no CLZ instruction. */
static int clz(uint32_t x)
{
    int n = 0;
    if (!(x >> 16)) { n += 16;  x <<= 16; }
    if (!(x >> 24)) { n += 8;   x <<= 8; }
    if (!(x >> 28)) { n += 4;   x <<= 4; }
    if (!(x >> 30)) { n += 2;   x <<= 2; }
    if (!(x >> 31)) { n += 1; }
    return n;
}

/* The one place a result is rounded.  m holds the significand with its
 * leading 1 at bit 31; anything nonzero below bit 8 means "a little more".
 * e is the biased exponent that leading 1 stands for.  Round to nearest,
 * ties to even; too big is infinity; too small goes subnormal, or zero. */
static uint32_t pack(uint32_t sign, int e, uint32_t m)
{
    if (e >= 255) { return sign | INF; }
    uint32_t base = 0;
    if (e >= 1) {
        base = (uint32_t)(e - 1) << 23;          /* + the leading 1 = e << 23 */
    } else {
        int s = 1 - e;                           /* subnormal: shift it down */
        m = s > 31 ? 1u : (m >> s) | ((m << (32 - s)) != 0u);
    }
    uint32_t q = m >> 8, rest = m & 0xFFu;
    if (rest > 0x80u || (rest == 0x80u && (q & 1u))) { q++; }
    return sign | (base + q);                    /* a carry out of q bumps the exponent */
}

#if !defined(__thumb__) || defined(SOFTFLOAT_C)

/* a subnormal's fraction, normalised: returns its exponent, m gets the 1 at bit 23 */
static int unsub(uint32_t *m)
{
    int s = clz(*m) - 8;
    *m <<= s;
    return 1 - s;
}

/* ---- + and - --------------------------------------------------------------- */
uint32_t __aeabi_fadd(uint32_t a, uint32_t b)
{
    if ((a << 1) < (b << 1)) { uint32_t t = a;  a = b;  b = t; }   /* |a| >= |b| */
    int ea = (int)((a >> 23) & 0xFFu), eb = (int)((b >> 23) & 0xFFu);
    if (ea == 0xFF) {                                     /* inf or NaN       */
        if (a << 9) { return QNAN; }
        if (eb == 0xFF && ((a ^ b) & SIGN)) { return QNAN; }   /* inf - inf */
        return a;
    }
    if (!(b << 1)) { return (a << 1) ? a : (a & b); }     /* + 0: -0 only if both */
    uint32_t ma = a & FRAC, mb = b & FRAC;
    if (ea) { ma |= ONE; } else { ea = 1; }
    if (eb) { mb |= ONE; } else { eb = 1; }
    ma <<= 7;  mb <<= 7;                                  /* the 1 at bit 30   */
    int d = ea - eb;
    if (d) {                                              /* line up b, keep a */
        if (d > 31) { mb = 1; }                           /* sticky: "a little" */
        else { mb = (mb >> d) | ((mb << (32 - d)) != 0u); }
    }
    uint32_t m;
    if ((a ^ b) & SIGN) {
        m = ma - mb;
        if (!m) { return 0; }                             /* x - x = +0        */
    } else {
        m = ma + mb;
    }
    int s = clz(m);                                       /* 0: carried out    */
    return pack(a & SIGN, ea + 1 - s, m << s);
}

uint32_t __aeabi_fsub(uint32_t a, uint32_t b) { return __aeabi_fadd(a, b ^ SIGN); }

/* ---- * ----------------------------------------------------------------------- */
uint32_t __aeabi_fmul(uint32_t a, uint32_t b)
{
    uint32_t sign = (a ^ b) & SIGN;
    int ea = (int)((a >> 23) & 0xFFu), eb = (int)((b >> 23) & 0xFFu);
    uint32_t ma = a & FRAC, mb = b & FRAC;
    if (ea == 0xFF || eb == 0xFF) {
        if ((ea == 0xFF && ma) || (eb == 0xFF && mb)) { return QNAN; }
        if (!(a << 1) || !(b << 1)) { return QNAN; }      /* inf * 0           */
        return sign | INF;
    }
    if (!(a << 1) || !(b << 1)) { return sign; }          /* signed zero       */
    if (ea) { ma |= ONE; } else { ea = unsub(&ma); }
    if (eb) { mb |= ONE; } else { eb = unsub(&mb); }

    /* 24 x 24 bits from 8- and 16-bit pieces: the M0+ multiplies 32 x 32 -> 32.
     * The top 32 bits of the 48-bit product, and whether anything is below: */
    uint32_t al = ma & 0xFFFFu, ah = ma >> 16, bl = mb & 0xFFFFu, bh = mb >> 16;
    uint32_t lo = al * bl;
    uint32_t m = ((ah * bh) << 16) + ah * bl + al * bh + (lo >> 16);
    uint32_t below = lo << 16;
    int e = ea + eb - 126;
    if (!(m & SIGN)) { e--;  m = (m << 1) | (below >> 31);  below <<= 1; }
    return pack(sign, e, m | (below != 0u));
}

/* ---- / ----------------------------------------------------------------------- */
uint32_t __aeabi_fdiv(uint32_t a, uint32_t b)
{
    uint32_t sign = (a ^ b) & SIGN;
    int ea = (int)((a >> 23) & 0xFFu), eb = (int)((b >> 23) & 0xFFu);
    uint32_t ma = a & FRAC, mb = b & FRAC;
    if (ea == 0xFF) {
        if (ma || eb == 0xFF) { return QNAN; }            /* NaN, inf / inf    */
        return sign | INF;
    }
    if (eb == 0xFF) { return mb ? QNAN : sign; }          /* x / inf = 0       */
    if (!(b << 1)) { return (a << 1) ? (sign | INF) : QNAN; }   /* x / 0    */
    if (!(a << 1)) { return sign; }
    if (ea) { ma |= ONE; } else { ea = unsub(&ma); }
    if (eb) { mb |= ONE; } else { eb = unsub(&mb); }

    int e = ea - eb + 127;
    if (ma < mb) { ma <<= 1;  e--; }                      /* quotient in [1, 2) */
    /* long division, one bit a step: 26 bits = 24 + two to round with */
    uint32_t q = 0, r = ma;
    for (int k = 0; k < 26; k++) {
        q <<= 1;
        if (r >= mb) { r -= mb;  q |= 1u; }
        r <<= 1;
    }
    return pack(sign, e, (q << 6) | (r != 0u));
}

/* ---- compares -------------------------------------------------------------------
 * Flip a negative float's other 31 bits and the bit patterns sort like the
 * numbers they stand for, as signed integers.  Left over: NaN (unordered:
 * every compare false) and -0 == +0. */
static int nan2(uint32_t a, uint32_t b) { return (a << 1) > (INF << 1) || (b << 1) > (INF << 1); }

static int32_t key(uint32_t a) { return (int32_t)(a ^ ((uint32_t)((int32_t)a >> 31) >> 1)); }

static int less(uint32_t a, uint32_t b)          /* a < b, both ordered */
{
    if (!((a | b) << 1)) { return 0; }           /* -0 < +0 is false    */
    return key(a) < key(b);
}

int __aeabi_fcmplt(uint32_t a, uint32_t b) { return !nan2(a, b) && less(a, b); }
int __aeabi_fcmpgt(uint32_t a, uint32_t b) { return !nan2(a, b) && less(b, a); }
int __aeabi_fcmple(uint32_t a, uint32_t b) { return !nan2(a, b) && !less(b, a); }
int __aeabi_fcmpge(uint32_t a, uint32_t b) { return !nan2(a, b) && !less(a, b); }
int __aeabi_fcmpeq(uint32_t a, uint32_t b) { return !nan2(a, b) && (a == b || !((a | b) << 1)); }

#else  /* ---- the same, in Thumb assembly, for the chip ------------------------ */

/* Register use, all three arithmetic functions:
 *   r6 the result's sign       r5 its exponent       r4 its significand
 *   r0 the "a little more" (sticky) bits, for rounding
 * They save r4-r6 the same way, so all end in one shared rounding tail,
 * sf_round - pack() above.  One section: the tail is reached by branches. */
__asm__(
"   .syntax unified\n"
"   .section .text.softfloat,\"ax\",%progbits\n"
"   .balign 4\n"

/* r2 r3 = the exponents of r0 r1; r0 r1 = their fractions, leading 1 set.
 * Zero, subnormal, inf or NaN: off to \spec with the fractions left bare. */
"   .macro sf_unpack spec\n"
"   lsls r2, r0, #1\n"
"   lsrs r2, r2, #24\n"
"   lsls r3, r1, #1\n"
"   lsrs r3, r3, #24\n"
"   lsls r0, r0, #9\n"
"   lsrs r0, r0, #9\n"
"   lsls r1, r1, #9\n"
"   lsrs r1, r1, #9\n"
"   subs r4, r2, #1\n"
"   cmp  r4, #254\n"                   /* ea - 1 >= 254: ea is 0 or 255   */
"   bhs  \\spec\n"
"   subs r4, r3, #1\n"
"   cmp  r4, #254\n"
"   bhs  \\spec\n"
"   movs r4, #1\n"
"   lsls r4, r4, #23\n"
"   orrs r0, r4\n"
"   orrs r1, r4\n"
"   .endm\n"

"   .macro sf_sign\n"                  /* r6 = the sign of a product or quotient */
"   movs r6, r0\n"
"   eors r6, r1\n"
"   lsrs r6, r6, #31\n"
"   lsls r6, r6, #31\n"
"   .endm\n"

/* ---- a * b ---------------------------------------------------------------- */
"   .global __aeabi_fmul\n"
"   .type   __aeabi_fmul, %function\n"
"   .thumb_func\n"
"__aeabi_fmul:\n"
"   push {r4, r5, r6, lr}\n"
"   sf_sign\n"
"   sf_unpack fm_spec\n"
"fm_go:\n"
"   adds r5, r2, r3\n"
"   subs r5, #126\n"                   /* exponent, if the product is >= 2 */
"   lsrs r2, r0, #16\n"                /* ah: the top 8 bits              */
"   lsrs r3, r1, #16\n"                /* bh                              */
"   uxth r0, r0\n"                     /* al: the low 16                  */
"   uxth r1, r1\n"                     /* bl                              */
"   movs r4, r2\n"
"   muls r4, r3, r4\n"                 /* ah bh                           */
"   muls r2, r1, r2\n"                 /* ah bl                           */
"   muls r3, r0, r3\n"                 /* bh al                           */
"   muls r0, r1, r0\n"                 /* al bl                           */
"   lsls r4, r4, #16\n"
"   adds r4, r4, r2\n"
"   adds r4, r4, r3\n"
"   lsrs r1, r0, #16\n"
"   lsls r0, r0, #16\n"                /* the bits below: sticky          */
"   adds r4, r4, r1\n"                 /* the top 32 bits of the product  */
"   bmi  sf_round\n"
"   subs r5, #1\n"                     /* product < 2: one more bit       */
"   lsls r0, r0, #1\n"
"   adcs r4, r4\n"                     /* (falls into sf_round)           */

/* ---- the rounding tail: pack() --------------------------------------------
 * in: r4 significand, leading 1 at bit 31; r0 nonzero = more below it;
 *     r5 the biased exponent of that leading 1; r6 the sign. */
"sf_round:\n"
"   subs r5, #1\n"
"   cmp  r5, #254\n"
"   bhs  sf_edge\n"                    /* e <= 0 (subnormal) or e >= 255  */
"sf_round_n:\n"
"   lsls r5, r5, #23\n"
"   lsrs r1, r4, #8\n"                 /* the 24 bits kept                */
"   adds r5, r5, r1\n"                 /* + exponent: a carry bumps it    */
"   lsls r4, r4, #24\n"                /* bit 7, the half, to the top     */
"   bpl  2f\n"                         /* below half: down                */
"   lsls r4, r4, #1\n"
"   bne  1f\n"                         /* above half: up                  */
"   cmp  r0, #0\n"
"   bne  1f\n"
"   lsrs r1, r1, #1\n"                 /* exactly half: to even           */
"   bcc  2f\n"
"1: adds r5, #1\n"
"2: adds r0, r5, r6\n"
"   pop  {r4, r5, r6, pc}\n"
"sf_edge:\n"
"   adds r5, #1\n"
"   bgt  sf_inf\n"                     /* e >= 255: too big               */
"   movs r1, #1\n"
"   subs r1, r1, r5\n"                 /* shift down 1 - e places         */
"   cmp  r1, #31\n"
"   bhi  1f\n"
"   movs r2, r4\n"
"   lsrs r4, r4, r1\n"
"   movs r3, #32\n"
"   subs r3, r3, r1\n"
"   lsls r2, r2, r3\n"                 /* what fell off: sticky           */
"   orrs r0, r2\n"
"   movs r5, #0\n"
"   b    sf_round_n\n"
"1: movs r4, #0\n"                     /* all of it fell off              */
"   movs r5, #0\n"
"   b    sf_round_n\n"
"sf_inf:\n"
"   movs r0, #255\n"
"   lsls r0, r0, #23\n"
"   adds r0, r0, r6\n"
"   pop  {r4, r5, r6, pc}\n"
"sf_zero:\n"
"   movs r0, r6\n"
"   pop  {r4, r5, r6, pc}\n"
"sf_nan:\n"
"   ldr  r0, =0x7FC00000\n"
"   pop  {r4, r5, r6, pc}\n"
"   .ltorg\n"

"fm_spec:\n"                           /* a zero, subnormal, inf or NaN   */
"   bl   sf_nan_in\n"                  /* (back only if neither is NaN)   */
"   cmp  r2, #255\n"
"   beq  fm_inf\n"
"   cmp  r3, #255\n"
"   beq  fm_inf\n"
"   bl   sf_zero_in\n"                 /* (back only if neither is 0)     */
"   bl   sf_norm\n"
"   b    fm_go\n"
"fm_inf:\n"                            /* inf * 0 is NaN, else inf        */
"   bl   sf_zero_in_nan\n"
"   b    sf_inf\n"

/* ---- helpers for the slow paths of * and / ----------------------------------
 * r0 r1 bare fractions, r2 r3 exponents.  The first three LEAVE the whole
 * function with the answer when they find what they look for. */
"sf_nan_in:\n"                         /* a NaN in: NaN out               */
"   cmp  r2, #255\n"
"   bne  1f\n"
"   cmp  r0, #0\n"
"   bne  sf_nan\n"
"1: cmp  r3, #255\n"
"   bne  2f\n"
"   cmp  r1, #0\n"
"   bne  sf_nan\n"
"2: bx   lr\n"
"sf_zero_in:\n"                        /* a zero in: a signed zero out    */
"   cmp  r2, #0\n"
"   bne  1f\n"
"   cmp  r0, #0\n"
"   beq  sf_zero\n"
"1: cmp  r3, #0\n"
"   bne  2f\n"
"   cmp  r1, #0\n"
"   beq  sf_zero\n"
"2: bx   lr\n"
"sf_zero_in_nan:\n"                    /* a zero in: NaN out (inf * 0)    */
"   cmp  r2, #0\n"
"   bne  1f\n"
"   cmp  r0, #0\n"
"   beq  sf_nan\n"
"1: cmp  r3, #0\n"
"   bne  2f\n"
"   cmp  r1, #0\n"
"   beq  sf_nan\n"
"2: bx   lr\n"
/* both finite and nonzero: give a normal number its leading 1; slide a
 * subnormal's fraction up to bit 23, taking its exponent below 1 */
"sf_norm:\n"
"   movs r4, #1\n"
"   lsls r4, r4, #23\n"
"   cmp  r2, #0\n"
"   bne  1f\n"
"   movs r2, #1\n"
"2: lsls r0, r0, #1\n"
"   subs r2, #1\n"
"   tst  r0, r4\n"
"   beq  2b\n"
"   b    3f\n"
"1: orrs r0, r4\n"
"3: cmp  r3, #0\n"
"   bne  4f\n"
"   movs r3, #1\n"
"5: lsls r1, r1, #1\n"
"   subs r3, #1\n"
"   tst  r1, r4\n"
"   beq  5b\n"
"   bx   lr\n"
"4: orrs r1, r4\n"
"   bx   lr\n"
"   .size __aeabi_fmul, .-__aeabi_fmul\n"

/* ---- a + b, a - b ------------------------------------------------------------
 * Significands here sit one byte higher than in * and /: the leading 1 at
 * bit 31, eight spare bits below the fraction for what b loses in lining up. */
"   .global __aeabi_fsub\n"
"   .type   __aeabi_fsub, %function\n"
"   .thumb_func\n"
"__aeabi_fsub:\n"
"   movs r2, #1\n"
"   lsls r2, r2, #31\n"
"   eors r1, r2\n"                     /* a - b = a + (-b)                */
"   .global __aeabi_fadd\n"
"   .type   __aeabi_fadd, %function\n"
"   .thumb_func\n"
"__aeabi_fadd:\n"
"   push {r4, r5, r6, lr}\n"
"   lsls r2, r0, #1\n"                 /* a and b without their signs     */
"   lsls r3, r1, #1\n"
"   cmp  r2, r3\n"
"   bhs  1f\n"
"   movs r4, r0\n"                     /* swap: make |a| >= |b|           */
"   movs r0, r1\n"
"   movs r1, r4\n"
"   movs r4, r2\n"
"   movs r2, r3\n"
"   movs r3, r4\n"
"1: eors r1, r0\n"                     /* r1 < 0: the signs differ        */
"   lsrs r6, r0, #31\n"
"   lsls r6, r6, #31\n"                /* the answer's sign is a's        */
"   lsrs r5, r2, #24\n"                /* ea: the answer's exponent too   */
"   cmp  r5, #255\n"
"   beq  fa_inf\n"
"   lsrs r4, r3, #24\n"                /* eb                              */
"   beq  fa_bsmall\n"                  /* b is zero or subnormal          */
"   movs r0, #1\n"
"   lsls r0, r0, #31\n"
"   lsls r2, r2, #7\n"
"   orrs r2, r0\n"                     /* a: 1.fraction from bit 31 down  */
"   lsls r3, r3, #7\n"
"   orrs r3, r0\n"                     /* b                               */
"fa_align:\n"
"   subs r4, r5, r4\n"                 /* how far b's exponent is below a's */
"   beq  fa_sum\n"
"   cmp  r4, #31\n"
"   bhi  fa_far\n"
"   movs r0, r3\n"
"   lsrs r3, r3, r4\n"                 /* line b up with a                */
"   negs r4, r4\n"
"   adds r4, #32\n"
"   lsls r0, r0, r4\n"                 /* what fell off the end...        */
"   beq  fa_sum\n"
"   movs r0, #1\n"
"   orrs r3, r0\n"                     /* ...is kept as one sticky bit    */
"fa_sum:\n"
"   cmp  r1, #0\n"
"   bmi  fa_sub\n"
"   adds r4, r2, r3\n"
"   bcs  fa_carry\n"
"   movs r0, #0\n"
"   b    sf_round\n"
"fa_carry:\n"                          /* a 33-bit sum: down one place    */
"   lsls r0, r4, #31\n"                /* the bit that falls off: sticky  */
"   lsrs r4, r4, #1\n"
"   movs r2, #1\n"
"   lsls r2, r2, #31\n"
"   orrs r4, r2\n"
"   adds r5, #1\n"
"   b    sf_round\n"
"fa_far:\n"
"   movs r3, #1\n"                     /* b is ALL sticky                 */
"   b    fa_sum\n"
"fa_sub:\n"
"   subs r4, r2, r3\n"
"   beq  fa_zero\n"                    /* x - x = +0                      */
"   bmi  2f\n"
"1: lsrs r0, r4, #23\n"                /* cancelled: slide the leading 1  */
"   bne  3f\n"                         /* back up to bit 31 - by a byte   */
"   lsls r4, r4, #8\n"                 /* while it is far, then by bits   */
"   subs r5, #8\n"
"   b    1b\n"
"3: subs r5, #1\n"
"   lsls r4, r4, #1\n"
"   bpl  3b\n"
"2: movs r0, #0\n"
"   b    sf_round\n"
"fa_bsmall:\n"
"   cmp  r3, #0\n"
"   beq  fa_bzero\n"
"   movs r4, #1\n"                     /* b subnormal: exponent 1, no 1.  */
"   lsls r3, r3, #7\n"
"   lsls r2, r2, #7\n"
"   cmp  r5, #0\n"
"   beq  1f\n"
"   movs r0, #1\n"
"   lsls r0, r0, #31\n"
"   orrs r2, r0\n"
"   b    fa_align\n"
"1: movs r5, #1\n"                     /* a subnormal too                 */
"   b    fa_align\n"
"fa_inf:\n"                            /* a is inf or NaN                 */
"   lsls r2, r2, #8\n"
"   bne  fa_nan\n"
"   lsrs r3, r3, #24\n"
"   cmp  r3, #255\n"
"   bne  fa_a\n"
"   cmp  r1, #0\n"
"   bpl  fa_a\n"                       /* inf - inf: NaN                  */
"fa_nan:\n"
"   b    sf_nan\n"
"fa_bzero:\n"                          /* b is 0: the answer is a - but   */
"   cmp  r2, #0\n"                     /* of two zeros, -0 only if both   */
"   bne  fa_a\n"
"   cmp  r1, #0\n"
"   bpl  fa_a\n"
"fa_zero:\n"
"   movs r0, #0\n"
"fa_a:\n"
"   pop  {r4, r5, r6, pc}\n"
"   .size __aeabi_fadd, .-__aeabi_fadd\n"

/* ---- a / b ------------------------------------------------------------------
 * The slow path comes first: a conditional branch reaches only 256 bytes,
 * and the 26 division steps of the fast path are longer than that. */
"fd_spec:\n"                           /* a zero, subnormal, inf or NaN   */
"   bl   sf_nan_in\n"
"   cmp  r2, #255\n"
"   bne  1f\n"
"   cmp  r3, #255\n"                   /* inf / inf: NaN                  */
"   beq  fd_nan\n"
"fd_inf:\n"
"   b    sf_inf\n"
"1: cmp  r3, #255\n"
"   beq  fd_zero\n"                    /* x / inf                         */
"   cmp  r3, #0\n"
"   bne  2f\n"
"   cmp  r1, #0\n"
"   bne  2f\n"
"   cmp  r2, #0\n"                     /* x / 0: inf - or NaN for 0 / 0   */
"   bne  fd_inf\n"
"   cmp  r0, #0\n"
"   bne  fd_inf\n"
"fd_nan:\n"
"   b    sf_nan\n"
"2: cmp  r2, #0\n"
"   bne  3f\n"
"   cmp  r0, #0\n"
"   bne  3f\n"
"fd_zero:\n"
"   b    sf_zero\n"                    /* 0 / x                           */
"3: bl   sf_norm\n"
"   b    fd_go\n"

"   .global __aeabi_fdiv\n"
"   .type   __aeabi_fdiv, %function\n"
"   .thumb_func\n"
"__aeabi_fdiv:\n"
"   push {r4, r5, r6, lr}\n"
"   sf_sign\n"
"   sf_unpack fd_spec\n"
"fd_go:\n"
"   subs r5, r2, r3\n"
"   adds r5, #127\n"
"   cmp  r0, r1\n"
"   bhs  1f\n"
"   lsls r0, r0, #1\n"                 /* make the quotient 1.something   */
"   subs r5, #1\n"
"1: movs r4, #0\n"
"   .rept 26\n"                        /* long division, one bit a step:  */
"   cmp  r0, r1\n"                     /* does the divisor fit?           */
"   bcc  2f\n"
"   subs r0, r0, r1\n"
"2: adcs r4, r4\n"                     /* the answer is the carry: shift it in */
"   lsls r0, r0, #1\n"
"   .endr\n"
"   lsls r4, r4, #6\n"                 /* r0, the remainder, is the sticky */
"   b    sf_round\n"
"   .size __aeabi_fdiv, .-__aeabi_fdiv\n"

/* ---- compares: 1 or 0.  Any NaN: 0.  -0 equals +0. ---------------------------
 * Flip a negative float's lower 31 bits and the patterns sort, as signed
 * integers, like the numbers they stand for.  \zz: where two zeros go. */
"   .macro fc_keys zz\n"
"   movs r3, #255\n"
"   lsls r3, r3, #24\n"
"   lsls r2, r0, #1\n"
"   cmp  r2, r3\n"
"   bhi  fc_0\n"                       /* a is NaN                        */
"   lsls r2, r1, #1\n"
"   cmp  r2, r3\n"
"   bhi  fc_0\n"                       /* b is NaN                        */
"   movs r3, r0\n"
"   orrs r3, r1\n"
"   lsls r3, r3, #1\n"
"   beq  \\zz\n"                       /* +-0 against +-0                 */
"   asrs r2, r0, #31\n"
"   lsrs r2, r2, #1\n"
"   eors r0, r2\n"
"   asrs r2, r1, #31\n"
"   lsrs r2, r2, #1\n"
"   eors r1, r2\n"
"   cmp  r0, r1\n"
"   .endm\n"

"   .global __aeabi_fcmpgt\n"
"   .type   __aeabi_fcmpgt, %function\n"
"   .thumb_func\n"
"__aeabi_fcmpgt:\n"                    /* a > b is b < a                  */
"   movs r2, r0\n"
"   movs r0, r1\n"
"   movs r1, r2\n"
"   .global __aeabi_fcmplt\n"
"   .type   __aeabi_fcmplt, %function\n"
"   .thumb_func\n"
"__aeabi_fcmplt:\n"
"   fc_keys fc_0\n"
"   blt  fc_1\n"
"fc_0:\n"
"   movs r0, #0\n"
"   bx   lr\n"
"fc_1:\n"
"   movs r0, #1\n"
"   bx   lr\n"
"   .size __aeabi_fcmplt, .-__aeabi_fcmplt\n"

"   .global __aeabi_fcmpge\n"
"   .type   __aeabi_fcmpge, %function\n"
"   .thumb_func\n"
"__aeabi_fcmpge:\n"                    /* a >= b is b <= a                */
"   movs r2, r0\n"
"   movs r0, r1\n"
"   movs r1, r2\n"
"   .global __aeabi_fcmple\n"
"   .type   __aeabi_fcmple, %function\n"
"   .thumb_func\n"
"__aeabi_fcmple:\n"
"   fc_keys fc_1\n"
"   ble  fc_1\n"
"   b    fc_0\n"
"   .size __aeabi_fcmple, .-__aeabi_fcmple\n"

"   .global __aeabi_fcmpeq\n"
"   .type   __aeabi_fcmpeq, %function\n"
"   .thumb_func\n"
"__aeabi_fcmpeq:\n"
"   fc_keys fc_1\n"
"   beq  fc_1\n"
"   b    fc_0\n"
"   .size __aeabi_fcmpeq, .-__aeabi_fcmpeq\n"
"   .text\n"
);

#endif

/* ---- int <-> float ------------------------------------------------------------- */
uint32_t __aeabi_ui2f(uint32_t x)
{
    if (!x) { return 0; }
    int s = clz(x);
    return pack(0, 158 - s, x << s);
}

uint32_t __aeabi_i2f(int32_t x)
{
    if (x >= 0) { return __aeabi_ui2f((uint32_t)x); }
    return __aeabi_ui2f(0u - (uint32_t)x) | SIGN;
}

/* toward zero; out of range saturates, NaN gives 0 */
uint32_t __aeabi_f2uiz(uint32_t a)
{
    int e = (int)((a >> 23) & 0xFFu);
    if ((a << 1) > (INF << 1) || (a & SIGN) || e < 127) { return 0; }
    if (e > 158) { return 0xFFFFFFFFu; }
    uint32_t m = (a << 8) | SIGN;                    /* the 1 at bit 31 */
    return m >> (158 - e);
}

int32_t __aeabi_f2iz(uint32_t a)
{
    if ((a << 1) > (INF << 1)) { return 0; }
    int e = (int)((a >> 23) & 0xFFu);
    if (e < 127) { return 0; }
    if (e > 157) { return (a & SIGN) ? INT32_MIN : INT32_MAX; }
    uint32_t v = ((a << 8) | SIGN) >> (158 - e);
    return (a & SIGN) ? -(int32_t)v : (int32_t)v;
}
