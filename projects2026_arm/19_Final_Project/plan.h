/*
 * plan.h - a show in, a timed track for every drone out
 *
 * What happens on a laptop the day before a real light show, done here by
 * the chip at power-up.  Nothing flies: the plan decides WHERE every drone
 * must be at every moment, and proves no two are ever planned closer than
 * `sep`.  The show, stage by stage:
 *
 *   pads      on the ground, motors armed                1 s
 *   takeoff   straight up to `takeoff` m
 *   move/hold MOVE to the next formation, HOLD it        per formation
 *   move/home MOVE back above the pads (any drone to any pad), hold 1 s
 *   land      each drone's own LAND mode: straight down
 *
 * A MOVE is planned in three steps:
 *   1. WHO GOES WHERE: the assignment with the smallest sum of SQUARED
 *      distances (the Hungarian algorithm).  With that choice, straight paths
 *      flown in step do not cross (Turpin, Michael, Kumar - "CAPT", 2014) if
 *      both formations are spaced about 1.4 x sep or more.
 *   2. HOW FAST: one trapezoidal speed profile (speed up at amax, cruise at
 *      vmax, brake at amax) for the longest path; every other drone flies the
 *      same profile scaled to its own distance, so all arrive together.
 *   3. CHECK: every pair, every 0.2 s.  A pair planned closer than `sep`
 *      is repaired: one drone flies over or under the other (up, across,
 *      down).  If no repair works, the show is refused.
 *
 * The plan is small - the positions at each formation, not a track per
 * moment: 13 spots x 5 drones x 12 B.  plan_sample() works out any moment from them.
 *
 * Pure C, no hardware.
 */
#ifndef PLAN_H
#define PLAN_H

#include <stdint.h>
#include "contract.h"
#include "show.h"


#define PLAN_SPOTS   (SHOW_MAX_FORMS + 3)            /* pads, up, forms, home    */
#define PLAN_STAGES  (2 * (SHOW_MAX_FORMS + 1) + 3)  /* pads, takeoff, ..., land */
#define PLAN_DT_CHECK 0.2f                           /* s between checks - see plan.c */

enum { ST_PADS, ST_TAKEOFF, ST_MOVE, ST_HOLD, ST_HOME, ST_LAND, ST_COUNT };

typedef struct { float L, vp, ta, T; } profile_t;    /* see plan.c, profile_*() */

typedef struct {
    uint8_t   kind;          /* ST_*                                              */
    uint8_t   from, to;      /* spot indices: a hold has from == to               */
    int8_t    form;          /* the formation it moves to or holds, -1 if none    */
    float     t0, T;         /* s                                                 */
    profile_t prof;          /* moves only                                        */
} stage_t;

typedef struct {
    const show_t *show;
    uint8_t  n, n_stages, n_spots, repairs;
    v3_t     spot[PLAN_SPOTS][SHOW_MAX_DRONES];    /* where each drone is at each spot  */
    int8_t   lift[PLAN_SPOTS][SHOW_MAX_DRONES];    /* move spot k -> k+1: fly over (+)
                                                or under (-) by this many DECIMETRES,
                                                0 = straight (a byte, not a float:
                                                RAM is the scarce thing here)      */
    stage_t  st[PLAN_STAGES];
    float    closest, closest_t;             /* the closest planned pass, off the ground */
    uint8_t  ci, cj;
    char     why[80];                        /* why it was refused                */
} plan_t;

/* Called after each move is checked: done of total.  Planning takes seconds
 * in Wokwi, and a silent board looks like a dead one. */
typedef void (*plan_progress_fn)(int done, int total);

int            plan_build(plan_t *p, const show_t *s, plan_progress_fn progress);  /* 0, or -1: p->why */
float          plan_duration(const plan_t *p);
const stage_t *plan_stage(const plan_t *p, float t, int *index);
void           plan_sample(const plan_t *p, const stage_t *s, int i, float t, v3_t *pos, v3_t *vel);
void           plan_sample_all(const plan_t *p, const stage_t *s, float t, v3_t *pos, v3_t *vel);   /* [n] */
v3_t           plan_stage_end(const plan_t *p, const stage_t *s, int i);
const char    *plan_stage_name(const plan_t *p, const stage_t *s);   /* "heart", "pads" */
const char    *plan_kind_name(uint8_t kind);                         /* "move", "hold"  */
void           plan_shape(const form_t *f, int n, v3_t *out);        /* a formation's slots */
void           plan_assign(const v3_t *from, const v3_t *to, int n, uint8_t *who);   /* Hungarian */

#endif
