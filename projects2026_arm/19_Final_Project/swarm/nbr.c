/*
 * nbr.c - one drone's neighbour table.  See nbr.h.
 */
#include <string.h>
#include "nbr.h"

void nbr_init(nbrs_t *t) { memset(t, 0, sizeof *t); }

const nbr_t *nbr_find(const nbrs_t *t, uint8_t id)
{
    for (unsigned i = 0; i < t->n; i++) {
        if (t->e[i].id == id) { return &t->e[i]; }
    }
    return 0;
}

static float dist2(v3_t a, v3_t b)
{
    float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

static void fill(nbr_t *r, const bcn_t *b, uint32_t now)
{
    r->mode = b->mode;  r->bat = b->bat;  r->nn = b->nn;  r->type = b->type;  r->hop = b->hop;
    r->gs = b->gs;  r->ra = b->ra;
    r->gx = (int16_t)b->gx;  r->gy = (int16_t)b->gy;
    r->seq  = b->seq;   r->heard_ms = now;
    r->pos  = (v3_t){ (float)b->x * 0.01f,  (float)b->y * 0.01f,  (float)b->z * 0.01f };
    r->vel  = (v3_t){ (float)b->vx * 0.01f, (float)b->vy * 0.01f, (float)b->vz * 0.01f };
    r->rx++;
}

int nbr_heard(nbrs_t *t, uint8_t self, v3_t me, const bcn_t *b, uint32_t now, uint8_t *evicted)
{
    *evicted = 0;
    if (b->id == self) { return NBR_SELF; }

    for (unsigned i = 0; i < t->n; i++) {
        nbr_t *r = &t->e[i];
        if (r->id != b->id) { continue; }
        int16_t ahead = (int16_t)(uint16_t)(b->seq - r->seq);
        if (ahead <= 0 && ahead > -20) { t->stale++;  return NBR_OLD; }   /* repeat */
        if (ahead > 0 && ahead <= 50) {
            for (int k = 1; k < ahead; k++) {            /* the beacons in the gap */
                r->qf = (uint16_t)(r->qf - (r->qf >> 3));
                r->missed++;
            }
        }
        /* (a jump back of 20 or more, or ahead by more than 50: it restarted,
         * or was out of range for a while - start counting again from here) */
        r->qf = (uint16_t)(r->qf + ((65535u - r->qf) >> 3));
        r->q  = (uint8_t)((r->qf * 100u + 32767u) / 65535u);
        fill(r, b, now);
        return NBR_SEEN;
    }

    nbr_t *r;
    if (t->n < NBR_MAX) {
        r = &t->e[t->n++];
    } else {                                             /* full: keep the nearest */
        r = &t->e[0];
        for (unsigned i = 1; i < t->n; i++) {
            if (dist2(t->e[i].pos, me) > dist2(r->pos, me)) { r = &t->e[i]; }
        }
        v3_t at = { (float)b->x * 0.01f, (float)b->y * 0.01f, (float)b->z * 0.01f };
        if (dist2(at, me) >= dist2(r->pos, me)) { t->full++;  return NBR_FAR; }
        *evicted = r->id;
        t->evictions++;
    }
    memset(r, 0, sizeof *r);
    r->id = b->id;
    r->q = 100u;  r->qf = 65535u;
    fill(r, b, now);
    t->joins++;
    return NBR_NEW;
}

int nbr_expire(nbrs_t *t, uint32_t now, uint8_t *gone, int max)
{
    int k = 0;
    for (unsigned i = 0; i < t->n;) {
        if (now - t->e[i].heard_ms >= NBR_TIMEOUT_MS) {
            if (k < max) { gone[k] = t->e[i].id; }
            k++;
            t->losses++;
            t->e[i] = t->e[--t->n];                      /* last row fills the hole */
        } else {
            i++;
        }
    }
    return k < max ? k : max;
}
