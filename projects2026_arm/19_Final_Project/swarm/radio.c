/*
 * radio.c - frames to lines and back.  See radio.h.
 */
#include <string.h>
#include "radio.h"
#include "proto.h"
#include "../fmt.h"

int bcn_encode(const bcn_t *b, char *buf, size_t size)
{
    char body[RADIO_LINE];
    int n = fmt(body, sizeof body, "BCN,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%u,%ld,%ld,%u,%u",
                (unsigned)b->id, (unsigned)b->seq, (long)b->x, (long)b->y, (long)b->z,
                (long)b->vx, (long)b->vy, (long)b->vz, (unsigned)b->bat, (unsigned)b->mode,
                (unsigned)b->nn, (unsigned)b->type, (unsigned)b->hop, (long)b->gx, (long)b->gy,
                (unsigned)b->gs, (unsigned)b->ra);
    return fmt(buf, size, "$%s*%02X\n", body, (unsigned)proto_checksum(body, (size_t)n));
}

/* One comma-separated integer field; *p is moved past it and its comma. */
static int field(const char **p, const char *end, int32_t *out)
{
    const char *s = *p;
    int neg = 0, digits = 0;
    int32_t v = 0;
    if (s < end && (*s == '-' || *s == '+')) { neg = *s == '-';  s++; }
    while (s < end && *s >= '0' && *s <= '9') {
        if (v > 100000000) { return -1; }               /* no field is this big */
        v = v * 10 + (*s++ - '0');
        digits++;
    }
    if (!digits || (s < end && *s != ',')) { return -1; }
    if (s < end) { s++; }                                /* the comma            */
    *p = s;
    *out = neg ? -v : v;
    return 0;
}

int frame_ints(const char *line, const char *name, int32_t *out, int max)
{
    const char *body;
    size_t len, nl = strlen(name);
    if (proto_check(line, &body, &len) != PROTO_OK) { return BCN_E_FRAME; }
    if (len < nl + 1 || memcmp(body, name, nl) != 0 || body[nl] != ',') { return BCN_E_NAME; }
    const char *p = body + nl + 1, *end = body + len;
    int n = 0;
    while (p < end) {
        if (n >= max || field(&p, end, &out[n]) != 0) { return BCN_E_FIELDS; }
        n++;
    }
    return n;
}

int bcn_decode(const char *line, bcn_t *b)
{
    int32_t f[17];
    int n = frame_ints(line, "BCN", f, 17);
    if (n < 0) { return n; }
    if (n != 17) { return BCN_E_FIELDS; }
    if (f[0] < 1 || f[0] > (int32_t)RADIO_MAX_ID || f[1] < 0 || f[1] > 65535 ||
        f[8] < 0 || f[8] > 100 || f[9] < 0 || f[9] > 255 || f[10] < 0 || f[10] > 255 ||
        f[11] < 0 || f[11] > 255 || f[12] < 0 || f[12] > 255 || f[15] < 0 || f[15] > 65535 ||
        f[16] < 0 || f[16] > 255) {
        return BCN_E_RANGE;
    }
    *b = (bcn_t){
        .id = (uint8_t)f[0], .seq = (uint16_t)f[1],
        .x = f[2], .y = f[3], .z = f[4], .vx = f[5], .vy = f[6], .vz = f[7],
        .bat = (uint8_t)f[8], .mode = (uint8_t)f[9], .nn = (uint8_t)f[10],
        .type = (uint8_t)f[11], .hop = (uint8_t)f[12], .gx = f[13], .gy = f[14], .gs = (uint16_t)f[15],
        .ra = (uint8_t)f[16],
    };
    return BCN_OK;
}
