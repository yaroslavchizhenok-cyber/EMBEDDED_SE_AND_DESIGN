/*
 * tasks.c - the auction with no auctioneer.  See tasks.h.
 */
#include <string.h>
#include "tasks.h"
#include "../trig.h"

void tasks_reset(tasks_t *k, int n, const float *xy)
{
    uint16_t keep = k->my_stamp;                 /* stamps keep counting: never reused */
    memset(k, 0, sizeof *k);
    k->my_stamp = keep;
    k->mine = -1;
    if (n < 0) { n = 0; }
    if (n > (int)TASK_MAX) { n = TASK_MAX; }
    k->n = (uint8_t)n;
    for (int i = 0; i < n; i++) {
        k->t[i].x = xy[2 * i];  k->t[i].y = xy[2 * i + 1];
        k->t[i].bid = TASK_INF;
    }
}

uint16_t tasks_cost(const task_t *t, v3_t me, float bat)
{
    float dx = t->x - me.x, dy = t->y - me.y;
    float c = t_sqrt(dx * dx + dy * dy) * 10.0f;          /* dm */
    float used = 100.0f - (bat > 100.0f ? 100.0f : bat < 0.0f ? 0.0f : bat);
    c += used * (float)TASK_BAT_DM;
    return c > 60000.0f ? 60000u : (uint16_t)c;
}

uint16_t tasks_age_ds(const task_t *t, uint32_t now)
{
    uint32_t a = (now - t->seen_ms) / 100u;
    return a > 9999u ? 9999u : (uint16_t)a;
}

static int beats(uint16_t b, uint8_t w, uint16_t vb, uint8_t vw)
{
    return b < vb || (b == vb && w < vw);
}

void tasks_merge(tasks_t *k, uint8_t self, int i, uint8_t w, uint16_t b, uint16_t s,
                 uint16_t age_ds, uint8_t d, uint32_t now)
{
    if (i < 0 || i >= k->n) { return; }
    task_t *t = &k->t[i];
    if (d) {                                       /* done is done, for everyone */
        if (!t->done && k->mine == i) { k->mine = -1; }
        t->done = 1u;
        return;
    }
    if (t->done || w == 0u) { return; }
    uint32_t age_ms = (uint32_t)age_ds * 100u;
    if (age_ms >= TASK_STALE_MS) { return; }       /* a dead claim: never takes over */
    uint32_t seen = now - age_ms;
    if (w == self) { return; }                     /* I know my own claims best     */
    if (w == t->winner) {                          /* news of the same claim        */
        if ((int16_t)(uint16_t)(s - t->stamp) > 0) {
            t->stamp = s;  t->bid = b;  t->seen_ms = seen;
        }
        return;
    }
    if (t->winner == 0u || t->bid == TASK_INF || beats(b, w, t->bid, t->winner)) {
        if (t->winner == self && k->mine == i) { k->mine = -1;  k->losses++; }   /* outbid */
        t->winner = w;  t->bid = b;  t->stamp = s;  t->seen_ms = seen;
    }
}

void tasks_expire(tasks_t *k, uint8_t self, uint32_t now)
{
    for (unsigned i = 0; i < k->n; i++) {
        task_t *t = &k->t[i];
        if (t->done || t->winner == 0u || t->winner == self) { continue; }
        if (now - t->seen_ms >= TASK_STALE_MS) {
            t->winner = 0u;  t->bid = TASK_INF;
            k->voided++;
        }
    }
}

int tasks_bid(tasks_t *k, uint8_t self, v3_t me, float bat, uint32_t now)
{
    if (k->mine >= 0) { return k->mine; }
    int best = -1;
    uint16_t best_c = TASK_INF;
    for (unsigned i = 0; i < k->n; i++) {
        task_t *t = &k->t[i];
        if (t->done) { continue; }
        uint16_t c = tasks_cost(t, me, bat);
        int open = t->winner == 0u || t->bid == TASK_INF || t->winner == self;
        if (!open && !((uint32_t)c + TASK_MARGIN_DM < t->bid)) { continue; }
        if (c < best_c) { best = (int)i;  best_c = c; }
    }
    if (best >= 0) {
        task_t *t = &k->t[best];
        t->winner = self;  t->bid = best_c;  t->stamp = ++k->my_stamp;  t->seen_ms = now;
        k->mine = (int8_t)best;
        k->claims++;
    }
    return k->mine;
}

void tasks_refresh(tasks_t *k, uint8_t self, v3_t me, float bat, uint32_t now)
{
    if (k->mine < 0) { return; }
    task_t *t = &k->t[k->mine];
    if (t->winner != self) { k->mine = -1;  return; }
    uint16_t c = tasks_cost(t, me, bat);
    if (c < t->bid) { t->bid = c; }                 /* closer now: a stronger hold */
    t->stamp = ++k->my_stamp;
    t->seen_ms = now;
}

void tasks_release(tasks_t *k, uint8_t self, uint32_t now)
{
    if (k->mine < 0) { return; }
    task_t *t = &k->t[k->mine];
    if (t->winner == self) { t->bid = TASK_INF;  t->stamp = ++k->my_stamp;  t->seen_ms = now; }
    k->mine = -1;
}

void tasks_finish(tasks_t *k, int i)
{
    if (i < 0 || i >= k->n) { return; }
    k->t[i].done = 1u;
    if (k->mine == i) { k->mine = -1; }
}

int tasks_all_done(const tasks_t *k)
{
    for (unsigned i = 0; i < k->n; i++) { if (!k->t[i].done) { return 0; } }
    return 1;
}
