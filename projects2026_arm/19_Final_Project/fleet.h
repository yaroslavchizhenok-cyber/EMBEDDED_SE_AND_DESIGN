/*
 * fleet.h - N drones, one clock, one sky
 *
 * Every drone is a craft_t (its airframe, craft.c) and a pilot_t (its
 * flight computer, pilot.c).  fleet_step() moves ALL of them on by one
 * airframe tick of 10 ms, so no drone can run ahead of another - the
 * lock-step a real light show gets from GPS time.  Pilots run on every
 * second tick (50 Hz).
 *
 * The chip holds FLEET_MAX drones (about 520 B of RAM each); the PC build
 * (host/) raises it with -DFLEET_MAX=64.
 *
 * Pure C, no hardware: the firmware and host/ compile the same file.
 */
#ifndef FLEET_H
#define FLEET_H

#include <stdint.h>
#include "contract.h"

#ifndef FLEET_MAX
#define FLEET_MAX 5
#endif

/* One drone as the outside world sees it.  Floats in SI units.
 * host/fleet.py mirrors this struct field for field - change both. */
typedef struct {
    uint32_t t_ms;
    uint8_t  mode, why, grounded, fixes;
    float    x, y, z;            /* TRUTH: where the airframe is               */
    float    vx, vy, vz;
    float    ex, ey, ez;         /* BELIEF: where its pilot thinks it is       */
    float    gx, gy, gz;         /* the goal the pilot is flying to right now  */
    float    roll, pitch, yaw;   /* rad, actual                                */
    float    lift;               /* m/s^2, actual                              */
    float    charge;             /* percent                                    */
    float    air_x, air_y;       /* the air around it, m/s                     */
    float    hardest_landing;    /* m/s                                        */
} fleet_drone_t;

int      fleet_init(int n, uint32_t seed);       /* n drones at (0,0); n, or -1 */
int      fleet_count(void);
void     fleet_place(int i, float x, float y);   /* on its pad, before arming   */
void     fleet_wind(float speed, float toward_rad);
void     fleet_gps(float wander, float jitter);  /* every receiver, m           */
int      fleet_arm(int i);                       /* 0 ok, 1 wrong mode, 2 no GPS */
int      fleet_land(int i);
void     fleet_target(int i, v3_t at, v3_t vel);
void     fleet_step(void);                       /* every drone, one 10 ms tick */
uint32_t fleet_time_ms(void);
int      fleet_view(int i, fleet_drone_t *out);
v3_t     fleet_where(int i);                     /* truth, cheaply              */
uint8_t  fleet_mode(int i);
uint8_t  fleet_grounded(int i);

#endif
