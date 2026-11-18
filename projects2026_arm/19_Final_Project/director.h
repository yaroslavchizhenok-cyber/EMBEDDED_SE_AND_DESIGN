/*
 * director.h - runs the show: one clock, the plan, the score, the frames
 *
 * The light show's ground station, inside the chip.  Every tick (10 ms of
 * show time) it:
 *     - every 100 ms tells each drone "be here, moving like this" - the plan
 *       at time t, position AND velocity (the pilot's feed-forward);
 *     - steps the whole fleet one tick;
 *     - measures how far each drone is from where the plan says, and how
 *       close any two drones are;
 *     - every 100 ms sends the station its frames.
 * No drone talks to another, and none knows the plan.
 *
 *   REFUSED   the plan failed its check: nothing will fly (why: plan.why)
 *   WARMUP    on the pads, motors off, GPS settling (1.3 s of show time)
 *   READY     waiting for START (button A, key a, or `start`)
 *   FLYING    the show; LANDING once the drones were told to land
 *   DONE      everyone down, the score sent; START flies it again
 *
 * Frames (lesson 08's $BODY*CS), 10 per second of show time:
 *   $SWM,t_ms,n,stage,stages,name,kind,show_ms,wind_cms,wind_deg,rtk,ff,state
 *                                                       where the show is, its settings
 *   $DRN,i,mode,x,y,z,px,py,pz,sx,sy,sz,bat,ax,ay       cm: truth, plan now, slot;
 *                                                       ax ay: the air at the drone, cm/s
 *   $SEP,t_ms,now_cm,i,j,min_cm,mi,mj,sep_cm,safe_cm    closest pair now / ever
 *   $SCO,rms_cm,max_cm,closest_cm,landed,n,pass         once, at the end
 * tools/station/views/l19_swarm.py draws them.
 *
 * Pure C, no hardware.  Output goes through the emit function it is given:
 * the firmware sends it to the UART, host/test_show.c checks it.
 */
#ifndef DIRECTOR_H
#define DIRECTOR_H

#include <stdint.h>
#include "plan.h"

enum { DR_REFUSED, DR_WARMUP, DR_READY, DR_FLYING, DR_LANDING, DR_DONE, DR_COUNT };

/* must = 1: never drop this line (the score); 0: may be dropped if the link is busy */
typedef void (*emit_fn)(const char *line, int must);

typedef struct {
    float    sq, worst;              /* while following the plan: sum of err^2, max */
    float    fsq, fworst;            /* during holds only                            */
    uint32_t n, fn;
} drone_score_t;

typedef struct {
    plan_t   plan;
    emit_fn  emit;
    uint8_t  state, paused, aborted, landing_sent, feedforward, rtk, start_pending, quiet;
    uint32_t seed;
    float    wind, wind_deg;
    uint32_t ticks;                  /* since arming: show time = ticks x 10 ms     */
    uint32_t warm;                   /* ticks of WARMUP                             */
    /* ---- the score -------------------------------------------------------- */
    drone_score_t sc[FLEET_MAX];
    float    closest, closest_t;  uint8_t ci, cj;
    float    last_bad[PLAN_STAGES];  /* s: last moment a hold was out of tolerance  */
    uint8_t  hold_seen[PLAN_STAGES];
    float    aborted_at;
    uint8_t  landed, passed;
    float    rms, max_err;
} director_t;

int         director_init(director_t *d, const show_t *s, emit_fn emit);   /* -1: refused */
int         director_tick(director_t *d);      /* 1 if the fleet moved, 0 if idle */
void        director_start(director_t *d);     /* READY: go; DONE: again; paused: carry on */
void        director_pause(director_t *d);
void        director_go(director_t *d);
void        director_abort(director_t *d);
void        director_reset(director_t *d);     /* back on the pads, same settings */
void        director_set_wind(director_t *d, float speed, float toward_deg);   /* any time */
int         director_set_seed(director_t *d, uint32_t seed);   /* not in flight: -1 */
int         director_set_gps(director_t *d, int rtk);
int         director_set_ff(director_t *d, int on);
void        director_print_plan(director_t *d);
void        director_report(director_t *d);    /* the score, as text lines        */
void        director_heartbeat(director_t *d, uint32_t beat);  /* idle: every 100 ms */
const char *director_state_name(uint8_t state);
uint32_t    director_show_ms(const director_t *d);

#endif
