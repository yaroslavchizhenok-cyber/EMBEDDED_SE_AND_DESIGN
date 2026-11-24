/*
 * steer.h - how one drone moves among others: a few local rules
 *
 * No drone is told where to fly around the others.  Each one works out its
 * own velocity, 20 times a second, from where it wants to go and from what
 * its neighbours' beacons say - nothing else.  The group's shape is not
 * written anywhere: it EMERGES from these rules (Reynolds, "Flocks, herds
 * and schools", 1987, plus a collision-avoidance rule on top):
 *
 *   separation   closer than STEER_SEP to a neighbour: push away, with the
 *                square of how close - gentle at first, then very hard
 *   avoidance    LOOK AHEAD: from both velocities, where will the two of
 *                us be closest in the next 3 s?  If closer than STEER_AVOID,
 *                move so that point opens up.  Head-on, both turn RIGHT -
 *                the same rule on both drones makes them pass, not dance.
 *                A drone with a conflict ahead also BRAKES: its goal speed
 *                shrinks with how close the predicted pass is
 *   goal         toward where I want to be, slowing as I arrive.  A goal
 *                the whole flock shares (GOTO) stops pulling STEER_SLACK
 *                short of it - otherwise every drone presses to the same
 *                point and only separation holds them apart
 *   obstacles    the ones this drone's rangefinder has seen (agent.c): kept
 *                OBS_SEP from their surface, the same look-ahead as for a
 *                drone (one that never moves), and when the goal lies behind
 *                one - pulling straight into it - SLIDE round it, always to
 *                the right of the way in: the head-on rule again, so a drone
 *                pressed against an obstacle cannot stall there
 *   cohesion     toward my neighbours' centre       (flocking only)
 *   alignment    toward my neighbours' mean velocity (flocking only)
 *
 * SAFETY FIRST: separation and avoidance get the speed budget first, and
 * the goal only what is left of STEER_VMAX - a drone in a crowd slows down
 * rather than push through.
 *
 * A neighbour's position is its last beacon moved on by its velocity for
 * the beacon's age: a beacon is up to 200 ms old, and at 3 m/s that is 0.6 m.
 *
 * Pure C, no hardware: host/ compiles it too.
 */
#ifndef STEER_H
#define STEER_H

#include <stdint.h>
#include "nbr.h"

#define STEER_VMAX   3.0f      /* m/s                                           */
#define STEER_SEP    2.8f      /* m: separation starts (under the 3 m between pads,
                                  so drones can land side by side)               */
#define STEER_AVOID  3.0f      /* m: a predicted pass closer than this is avoided */
#define STEER_SLACK  3.0f      /* m: a SHARED goal (GOTO) only pulls from further out */
#define STEER_HORIZ  3.0f      /* s: how far ahead it looks                     */
#define OBS_SEP      2.0f      /* m kept from an obstacle's surface             */
#define OBS_MAX      6u        /* obstacles a drone keeps in mind               */

/* An obstacle: a round column, taller than any lane.  dm, so 6 fit in 48 B. */
typedef struct {
    int16_t  x, y;             /* its centre, dm                                */
    uint8_t  id, r;            /* the world's number for it; its radius, dm     */
    uint32_t seen_ms;          /* when the rangefinder last saw it (my clock)   */
} obst_t;

typedef struct {
    v3_t  me, vel;             /* my estimate                                   */
    int   has_goal;
    v3_t  goal;
    float k_coh, k_ali;        /* 0 unless flocking                             */
    float slack;               /* m: the goal pulls only from beyond this       */
    float vmax;                /* <= STEER_VMAX                                  */
    const obst_t *obs;         /* the obstacles in mind                         */
    uint8_t nobs;
} steer_in_t;

typedef struct {
    v3_t  v;                   /* the velocity to fly, m/s (z = 0)              */
    float closest;             /* m, to the nearest neighbour now (99: none)    */
    uint8_t avoiding;          /* a predicted conflict shaped v                 */
} steer_out_t;

void steer(const steer_in_t *in, const nbrs_t *nb, uint32_t now_ms, steer_out_t *out);

#endif
