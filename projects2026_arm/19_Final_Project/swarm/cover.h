/*
 * cover.h - the survey: a shared map that nobody owns
 *
 * A SURVEY mission names a rectangle.  Each drone cuts it into the same grid
 * (at most COVER_N x COVER_N cells, none wider than COVER_CELL m), so every
 * cell has the same number on every drone.  A MAPPER's camera covers a cell
 * when it flies within COVER_SEE m of its centre; it sets that cell's bit.
 * Bits are only ever SET, so two copies merge by OR, in any order, as often
 * as they like, and all copies end up the same - a grow-only set, the
 * simplest conflict-free replicated data type.  Each drone gossips its 64
 * bits once a second ($MAP), and the map spreads hop by hop.
 *
 * Which cell next?  Each mapper picks, every half second, the nearest cell
 * not yet covered - skipping a cell another mapper is heading for if that
 * mapper is nearer to it (its beacon says where it is heading).  That is
 * the whole coverage "plan": nothing divides the area up, the drones spread
 * over it because each one avoids what its neighbours are about to do.
 *
 * Pure C, no hardware: host/ compiles it too.
 */
#ifndef COVER_H
#define COVER_H

#include <stdint.h>
#include "nbr.h"

#define COVER_N     8u        /* cells per side, at most: 64 bits             */
#define COVER_CELL  5.0f      /* m: cells are at most this wide               */
#define COVER_SEE   1.5f      /* m from a cell's centre: the camera covers it */

typedef struct {
    float    x0, y0, cw, ch;  /* the area's corner, a cell's size, m          */
    uint8_t  nx, ny;          /* 0: no survey                                  */
    uint32_t hi, lo;          /* the covered cells: bit (j * nx + i)          */
    int8_t   target;          /* the cell I am heading for, or -1             */
} cover_t;

void  cover_init(cover_t *c, float x0, float y0, float x1, float y1);
int   cover_cells(const cover_t *c);
int   cover_count(const cover_t *c);
int   cover_has(const cover_t *c, int cell);
void  cover_set(cover_t *c, int cell);
void  cover_merge(cover_t *c, uint32_t hi, uint32_t lo);
int   cover_done(const cover_t *c);
v3_t  cover_centre(const cover_t *c, int cell, float z);
int   cover_cell_at(const cover_t *c, float x, float y);      /* -1: outside */
int   cover_see(cover_t *c, v3_t at);                         /* 1: a new cell */
/* The next cell for a mapper at `me` with id `self`, given its neighbours. */
int   cover_choose(cover_t *c, uint8_t self, v3_t me, const nbrs_t *nb, uint8_t mapper_type);

#endif
