/*
 * fmt.c - the small snprintf.  See fmt.h.
 */
#include "fmt.h"

typedef struct { char *p; size_t room; int n; } out_t;   /* room keeps 1 for the NUL */

static void put(out_t *o, char c)
{
    if (o->room > 1u) { *o->p++ = c;  o->room--;  o->n++; }
}

static void pad(out_t *o, int count, char c)
{
    while (count-- > 0) { put(o, c); }
}

/* the digits of v in base 10 or 16, right to left into tmp; returns how many */
static int digits(char *tmp, unsigned long v, unsigned base, int upper)
{
    const char *d = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    int k = 0;
    do { tmp[k++] = d[v % base];  v /= base; } while (v);
    return k;
}

int fmt_v(char *buf, size_t size, const char *f, va_list ap)
{
    out_t o = { buf, size, 0 };
    if (size == 0u) { return 0; }
    for (; *f; f++) {
        if (*f != '%') { put(&o, *f);  continue; }
        f++;
        int left = 0, zero = 0, width = 0, lng = 0;
        for (;; f++) {
            if (*f == '-')      { left = 1; }
            else if (*f == '0') { zero = 1; }
            else                { break; }
        }
        while (*f >= '0' && *f <= '9') { width = width * 10 + (*f++ - '0'); }
        if (*f == 'l') { lng = 1;  f++; }
        if (!*f) { break; }

        char tmp[12];
        const char *s = tmp;
        int len = 0, neg = 0;
        switch (*f) {
        case 'd': case 'i': {
            long v = lng ? va_arg(ap, long) : va_arg(ap, int);
            unsigned long u = v < 0 ? 0ul - (unsigned long)v : (unsigned long)v;
            neg = v < 0;
            len = digits(tmp, u, 10u, 0);
            break;
        }
        case 'u': case 'x': case 'X': {
            unsigned long v = lng ? va_arg(ap, unsigned long) : va_arg(ap, unsigned);
            len = digits(tmp, v, *f == 'u' ? 10u : 16u, *f == 'X');
            break;
        }
        case 'c':
            tmp[0] = (char)va_arg(ap, int);
            len = 1;
            break;
        case 's':
            s = va_arg(ap, const char *);
            if (!s) { s = "(null)"; }
            while (s[len]) { len++; }
            break;
        default:                                    /* %% and anything unknown */
            put(&o, *f);
            continue;
        }

        int body = len + neg;
        int fill = width > body ? width - body : 0;
        if (!left && !(zero && *f != 's')) { pad(&o, fill, ' '); }
        if (neg) { put(&o, '-'); }
        if (!left && zero && *f != 's') { pad(&o, fill, '0'); }
        if (*f == 's') { for (int k = 0; k < len; k++) { put(&o, s[k]); } }
        else           { for (int k = len - 1; k >= 0; k--) { put(&o, tmp[k]); } }
        if (left) { pad(&o, fill, ' '); }
    }
    *o.p = '\0';
    return o.n;
}

int fmt(char *buf, size_t size, const char *f, ...)
{
    va_list ap;
    va_start(ap, f);
    int n = fmt_v(buf, size, f, ap);
    va_end(ap);
    return n;
}
