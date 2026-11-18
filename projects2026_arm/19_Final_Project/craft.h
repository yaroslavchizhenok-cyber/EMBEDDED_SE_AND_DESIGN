/*
 * craft.h - one simulated drone, and the sky the whole swarm shares
 *
 * The same physics as lesson 14's world.c, rewritten for many drones:
 *   - a point mass pushed by a thrust vector that roll and pitch tilt;
 *   - thrust, tilt and yaw rate FOLLOW the pilot's command through lags;
 *   - rotor drag on the velocity relative to the air;
 *   - the air = a STEADY wind that every drone feels the same (sky_t) plus
 *     GUSTS that each drone meets on its own (its own random stream);
 *   - a battery that drains with the 1.5th power of the thrust;
 *   - GPS at 10 Hz, 200 ms late, with an error that wanders slowly;
 *     a noisy baro; an accelerometer with a per-drone bias.
 *
 * What is NOT modelled: drones do not feel each other's downwash, and two
 * drones may pass through one another.  The planner and the score treat
 * "too close" as the failure - as a real show's safety case does.
 */
#ifndef CRAFT_H
#define CRAFT_H

#include <stdint.h>
#include "contract.h"

#define FIX_EVERY   5u               /* pilot steps per GPS fix: 10 Hz        */
#define FIX_LAG     10u              /* pilot steps of GPS latency: 200 ms    */

typedef struct {
    float speed;                     /* m/s, the steady part                  */
    float toward;                    /* rad, the direction it blows TOWARD    */
} sky_t;

/* How good the GPS receivers are.  Plain GPS wanders by tens of cm and
 * lesson 14 flies with it; show drones carry RTK GPS (a base station on the
 * ground sends corrections) and are good to a couple of cm - without it a
 * formation cannot be held much tighter than a metre. */
typedef struct {
    float wander;                    /* m, the slow part of the error         */
    float jitter;                    /* m, the jumpy part                     */
} gps_grade_t;

#define GPS_PLAIN  ((gps_grade_t){ 0.40f, 0.10f })
#define GPS_RTK    ((gps_grade_t){ 0.02f, 0.02f })

typedef struct {
    /* ---- the truth ----------------------------------------------------- */
    v3_t   pos, vel, acc;
    float  bank, nose, heading, spin;  /* roll, pitch, yaw (rad), yaw rate     */
    float  lift;                       /* m/s^2, what the rotors give now      */
    float  gust_x, gust_y;             /* this drone's own gust, m/s           */
    float  air_x, air_y;               /* the air it flies in, m/s             */
    float  charge;                     /* percent                              */
    float  hardest_landing;            /* m/s                                  */
    uint8_t grounded;
    /* ---- its sensors' private state ------------------------------------ */
    float  bias_x, bias_y;             /* accelerometer bias, m/s^2            */
    float  drift_x, drift_y;           /* GPS wander, m                        */
    gps_grade_t gps;                   /* its receiver                         */
    float  trail_x[FIX_LAG + 1], trail_y[FIX_LAG + 1];  /* where it was       */
    uint8_t trail_at;
    uint32_t samples;
    uint32_t t_ms;
    uint32_t dice;                     /* random state                         */
} craft_t;

void craft_init(craft_t *c, uint32_t seed, float x, float y);
void craft_step(craft_t *c, const sky_t *sky, const cmd_t *cmd, float dt);
void craft_sense(craft_t *c, meas_t *m);

#endif
