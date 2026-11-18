/*
 * contract.h - what one drone's airframe and its pilot hand each other
 *
 * The swarm keeps lesson 14's two-tier split: the airframe (craft.c) only
 * ever produces a meas_t, the pilot (pilot.c) only ever produces a cmd_t.
 * The pilot never reads the airframe's truth - one drone of the swarm flies
 * on late GPS and a biased accelerometer, exactly like lesson 14's one drone.
 *
 * Frame: x = east, y = north, z = up, metres, origin = the show's origin
 * (the middle of the launch pads).  Heading is counter-clockwise from +x.
 *
 * Pure C, no hardware.  The swarm runs on the PC (swarm.dll, called from
 * Python), but nothing here would stop it compiling for the chip.
 */
#ifndef CONTRACT_H
#define CONTRACT_H

#include <stdint.h>

#define AIR_HZ      100u                      /* airframe steps per second    */
#define AIR_DT      (1.0f / (float)AIR_HZ)
#define PILOT_HZ    50u                       /* pilot steps per second       */
#define PILOT_DT    (1.0f / (float)PILOT_HZ)
#define G0          9.81f

typedef struct { float x, y, z; } v3_t;

/* What the sensors say, once per pilot step. */
typedef struct {
    uint32_t t_ms;
    v3_t     acc;            /* world-frame acceleration, gravity removed, m/s^2 */
    float    heading;        /* compass, rad                                      */
    float    baro;           /* altitude, m, noisy                                */
    uint8_t  fix;            /* 1 when a GPS fix arrived on this step (10 Hz)     */
    float    fix_x, fix_y;   /* the fix, m - describing the drone 200 ms AGO      */
    float    charge;         /* battery, percent                                  */
} meas_t;

/* What the pilot asks of the motors.  The attitude loop (lesson 13) is
 * assumed to deliver the tilt; the airframe models it as a lag. */
typedef struct {
    uint8_t  live;           /* 0: motors stopped                                 */
    float    lift;           /* specific thrust, m/s^2 (9.81 = hover)             */
    float    bank, nose;     /* roll and pitch, rad                               */
    float    turn;           /* yaw rate, rad/s                                   */
} cmd_t;

#endif
