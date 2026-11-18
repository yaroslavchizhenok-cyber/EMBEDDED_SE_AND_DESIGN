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
 * f_clamp and f_abs stay inline from fmath.h: they are smaller than a call.
 */
#ifndef TRIG_H
#define TRIG_H

float t_sin(float a);
float t_cos(float a);
float t_sqrt(float x);
float t_atan2(float y, float x);
float t_wrap(float a);              /* into -pi .. pi */

#endif
