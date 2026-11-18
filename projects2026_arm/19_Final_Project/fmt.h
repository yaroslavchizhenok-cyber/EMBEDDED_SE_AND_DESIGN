/*
 * fmt.h - a small snprintf, because the real one did not fit
 *
 * newlib-nano's vsnprintf() costs about 2.5 KB of flash once it has pulled
 * in its helpers (and a malloc it never uses here).  This firmware ran out
 * of its 32 KB, and printing integers and strings needs far less.  So:
 *
 *   conversions  %d %i %u %x %X %s %c %%
 *   length       l            (%ld %lu: this chip's long is 32 bits like int)
 *   flags        -  left-align        0  pad with zeros
 *   width        a number:   %5d  %-8s  %02X  %3lu
 *
 * No floats - none of this firmware's output needs them: metres are printed
 * as centimetres split into "2.35" by the callers.  Anything else in a
 * format is copied as it is.  The result is always NUL-terminated, and the
 * return value is the length written (not, as in C99, the length that would
 * have been written).
 *
 * Pure C: host/ compiles it too.
 */
#ifndef FMT_H
#define FMT_H

#include <stdarg.h>
#include <stddef.h>

int fmt_v(char *buf, size_t size, const char *f, va_list ap);
int fmt(char *buf, size_t size, const char *f, ...) __attribute__((format(printf, 3, 4)));

#endif
