/*
 * trig.h - sin, cos, sqrt, atan2 and angle wrapping, compiled ONCE
 *
 * fmath.h (lesson 10) makes them `static inline`, and the compiler pastes a
 * copy into every place they are called.  This lesson calls them from five
 * files and dozens of places - the copies alone pushed the firmware past the
 * chip's 32 KB of flash.  Here each is compiled once, in trig.c, and called:
 * a call costs a few cycles, the function itself hundreds (soft float).
 * (Lesson 14 met the same problem; its fm.c is the same cure.)
 *
 * And because they are this program's own, they can be the versions that
 * suit it: sin and cos in ONE call, a square root with no float arithmetic
 * in it.  trig.c says how, and what it bought.
 *
 * f_clamp and f_abs stay inline from fmath.h: they are smaller than a call.
 */
#ifndef TRIG_H
#define TRIG_H

void  t_sincos(float a, float *sin_a, float *cos_a);   /* any angle, good to 1e-6 */
float t_sqrt(float x);              /* exact (correctly rounded); x <= 0 gives 0  */
float t_atan2(float y, float x);
float t_wrap(float a);              /* into -pi .. pi */

#endif
