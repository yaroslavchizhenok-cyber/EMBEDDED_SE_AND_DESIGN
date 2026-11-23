/*
 * cover.c - the survey map.  See cover.h.
 */
#include <string.h>
#include "cover.h"

void cover_init(cover_t *c, float x0, float y0, float x1, float y1)
{
    memset(c, 0, sizeof *c);
    c->target = -1;
    if (x1 < x0) { float t = x0;  x0 = x1;  x1 = t; }
    if (y1 < y0) { float t = y0;  y0 = y1;  y1 = t; }
    float w = x1 - x0, h = y1 - y0;
    if (w < 0.5f || h < 0.5f) { return; }                /* no area: no survey */
    int nx = (int)(w / COVER_CELL + 0.999f), ny = (int)(h / COVER_CELL + 0.999f);
    if (nx < 1) { nx = 1; }
    if (ny < 1) { ny = 1; }
    if (nx > (int)COVER_N) { nx = COVER_N; }
    if (ny > (int)COVER_N) { ny = COVER_N; }
    c->nx = (uint8_t)nx;  c->ny = (uint8_t)ny;
    c->x0 = x0;  c->y0 = y0;
    c->cw = w / (float)nx;  c->ch = h / (float)ny;
}

int cover_cells(const cover_t *c) { return c->nx * c->ny; }

int cover_has(const cover_t *c, int cell)
{
    if (cell < 0 || cell >= cover_cells(c)) { return 0; }
    return cell < 32 ? (int)((c->lo >> cell) & 1u) : (int)((c->hi >> (cell - 32)) & 1u);
}

void cover_set(cover_t *c, int cell)
{
    if (cell < 0 || cell >= cover_cells(c)) { return; }
    if (cell < 32) { c->lo |= 1u << cell; } else { c->hi |= 1u << (cell - 32); }
}

void cover_merge(cover_t *c, uint32_t hi, uint32_t lo)
{
    int n = cover_cells(c);
    uint32_t mlo = n >= 32 ? 0xFFFFFFFFu : (n > 0 ? (1u << n) - 1u : 0u);
    uint32_t mhi = n >= 64 ? 0xFFFFFFFFu : (n > 32 ? (1u << (n - 32)) - 1u : 0u);
    c->lo |= lo & mlo;
    c->hi |= hi & mhi;
}

int cover_count(const cover_t *c)
{
    int k = 0;
    for (int i = 0; i < cover_cells(c); i++) { k += cover_has(c, i); }
    return k;
}

int cover_done(const cover_t *c) { return c->nx && cover_count(c) == cover_cells(c); }

v3_t cover_centre(const cover_t *c, int cell, float z)
{
    int i = cell % c->nx, j = cell / c->nx;
    return (v3_t){ c->x0 + ((float)i + 0.5f) * c->cw, c->y0 + ((float)j + 0.5f) * c->ch, z };
}

int cover_cell_at(const cover_t *c, float x, float y)
{
    if (!c->nx) { return -1; }
    float fx = (x - c->x0) / c->cw, fy = (y - c->y0) / c->ch;
    if (fx < 0.0f || fy < 0.0f) { return -1; }
    int i = (int)fx, j = (int)fy;
    if (i >= c->nx || j >= c->ny) { return -1; }
    return j * c->nx + i;
}

int cover_see(cover_t *c, v3_t at)
{
    int cell = cover_cell_at(c, at.x, at.y);
    if (cell < 0 || cover_has(c, cell)) { return 0; }
    v3_t m = cover_centre(c, cell, at.z);
    float dx = m.x - at.x, dy = m.y - at.y;
    if (dx * dx + dy * dy > COVER_SEE * COVER_SEE) { return 0; }
    cover_set(c, cell);
    return 1;
}

static float d2(float ax, float ay, float bx, float by) { return (ax - bx) * (ax - bx) + (ay - by) * (ay - by); }

int cover_choose(cover_t *c, uint8_t self, v3_t me, const nbrs_t *nb, uint8_t mapper_type)
{
    int best = -1, fallback = -1;
    float best_d = 1e30f, fall_d = 1e30f;
    for (int cell = 0; cell < cover_cells(c); cell++) {
        if (cover_has(c, cell)) { continue; }
        v3_t m = cover_centre(c, cell, 0.0f);
        float mine = d2(m.x, m.y, me.x, me.y);
        if (cell == c->target) { mine *= 0.8f; }         /* keep what I chose: no dithering */
        int taken = 0;
        for (unsigned k = 0; k < nb->n && !taken; k++) {
            const nbr_t *o = &nb->e[k];
            if (o->type != mapper_type || o->mode != 2u) { continue; }
            if (cover_cell_at(c, (float)o->gx * 0.1f, (float)o->gy * 0.1f) != cell) { continue; }
            float theirs = d2(m.x, m.y, o->pos.x, o->pos.y);
            if (theirs < mine || (theirs == mine && o->id < self)) { taken = 1; }
        }
        if (!taken && mine < best_d) { best = cell;  best_d = mine; }
        if (mine < fall_d) { fallback = cell;  fall_d = mine; }
    }
    c->target = (int8_t)(best >= 0 ? best : fallback);
    return c->target;
}
