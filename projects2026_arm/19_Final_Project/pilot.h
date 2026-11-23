/*
 * pilot.h - one drone's flight computer, written so there can be many
 *
 * Lesson 14's flight.c keeps ONE drone in a file-level `static fc_t fc`, and
 * every function reaches for it.  That is fine on a chip that flies one
 * aircraft and impossible for a swarm.  Here every function takes the pilot
 * it works on - `pilot_t *p` - and the file holds no state at all, so a swarm
 * is just an array of pilot_t.  (This is the usual C way to write an
 * "object": the struct is the object, the pointer is `this`.)
 *
 * The pilot does less than lesson 14's: no missions, no geofence, no
 * parameters over the link.  A light-show drone does not decide where to go.
 * The ground station hands it a MOVING setpoint - a goal position and the
 * goal's velocity - several times a second, and the pilot's whole job is to
 * be there on time:
 *
 *     want_v = goal_v  +  k_pos * (goal - where I think I am)
 *              ^^^^^^ feed-forward:   ^^^^^^^^^^^^^^^^^^^^^^ feedback:
 *              fly with the goal       close the remaining gap
 *
 * Without the feed-forward term a moving goal is always chased from behind,
 * a fixed distance late (the lag is goal_v / k_pos) - in a formation every
 * drone would trail its slot.  Slide-style explanation in ../README.md.
 */
#ifndef PILOT_H
#define PILOT_H

#include <stdint.h>
#include "contract.h"

#define PILOT_PAST      12u          /* belief history, > the GPS lag in steps */
#define PILOT_STALE_MS  1000u        /* no new goal for this long -> HOVER     */

enum { PM_IDLE, PM_FOLLOW, PM_HOVER, PM_LAND, PM_COUNT };      /* modes       */
enum { PW_START, PW_CMD, PW_STALE, PW_FRESH, PW_BATTERY, PW_LANDED, PW_COUNT };
enum { PILOT_OK = 0, PILOT_E_MODE, PILOT_E_GPS };              /* results     */

typedef struct {
    float k_pos;         /* 1/s    position gap -> velocity correction          */
    float k_vel;         /* 1/s    velocity gap -> acceleration                 */
    float k_int;         /* 1/s^2  its integral: learns the wind                */
    float k_alt, k_climb, k_climb_int;   /* the same three, vertically          */
    float fix_speed;     /* m/s    biggest horizontal correction on top of goal_v */
    float tilt;          /* rad    acceleration limit = g tan(tilt)             */
    float gps_a, gps_b;  /*        the estimator's GPS gains (alpha, beta)      */
    float low_charge;    /* %      below this: land where you are               */
} pilot_tune_t;

typedef struct {
    /* ---- belief: where the pilot THINKS it is ------------------------- */
    v3_t   est, est_v;
    float  heading;
    float  past_x[PILOT_PAST], past_y[PILOT_PAST];
    uint8_t past_at, fixes;
    /* ---- the goal the ground station gave, moved on between updates --- */
    v3_t   goal, goal_v;
    uint32_t goal_ms;                  /* when the last goal arrived          */
    /* ---- loop memory --------------------------------------------------- */
    v3_t   sum;                        /* velocity-loop integrators           */
    uint8_t mode, why;
    uint32_t now_ms, mode_ms, quiet_ms;
    float  charge;
    cmd_t  last;                       /* the last command, for telemetry     */
    pilot_tune_t k;
    /* ---- worked out from k by pilot_init(), not every step: a tan costs
     * this chip ~1100 cycles.  Change k?  Call pilot_retune() after. --- */
    float  reach;                      /* m/s^2, g tan(tilt)                  */
    float  cap, cap_z;                 /* the integrators' anti-windup limits */
} pilot_t;

void        pilot_init(pilot_t *p);
void        pilot_retune(pilot_t *p);                 /* after changing p->k  */
int         pilot_arm(pilot_t *p);                    /* IDLE -> FOLLOW       */
int         pilot_land(pilot_t *p);                   /* -> LAND, then IDLE   */
void        pilot_goal(pilot_t *p, v3_t at, v3_t vel);
void        pilot_step(pilot_t *p, const meas_t *m, cmd_t *out);   /* PILOT_HZ */
const char *pilot_mode_name(uint8_t mode);

#endif
