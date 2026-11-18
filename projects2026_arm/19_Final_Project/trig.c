/*
 * trig.c - one copy each of fmath.h's sin, cos, sqrt, atan2 and wrap.  See trig.h.
 */
#include "trig.h"
#include "fmath.h"

float t_sin(float a)              { return f_sin(a); }
float t_cos(float a)              { return f_cos(a); }
float t_sqrt(float x)             { return f_sqrt(x); }
float t_atan2(float y, float x)   { return f_atan2(y, x); }
float t_wrap(float a)             { return f_wrap(a); }
