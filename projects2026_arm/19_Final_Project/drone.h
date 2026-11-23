/*
 * drone.h - ONE drone of the show, on its own chip
 *
 * Every drone of the swarm is a board of its own: this chip flies one
 * airframe (craft.c) with one flight computer (pilot.c), and nothing else.
 * Each chip plans the whole show at power-up - the same show.c, the same
 * plan.c, so the same plan on every chip, bit for bit - and flies only its
 * own track of it.  No chip knows another chip's state except through the
 * SCENE the ground station sends it.
 *
 * The ground station is host/conductor.py on the PC.  It does not fly
 * anyone.  It keeps the swarm on one clock and tells every drone what the
 * world looks like; each drone decides for itself what to do about it:
 *
 *   the clock      $TCK,t_ms        fly on until show time t_ms (a 100 ms window),
 *                                   then report.  The conductor sends the next
 *                                   window only when EVERY drone has reported, so
 *                                   five chips simulated at five different speeds
 *                                   (five Wokwi windows) still fly in lock-step
 *   the wind       $WND,cms,deg     the steady wind; each drone meets its own gusts
 *   the neighbours $NBR,t_ms,n,x0,y0,z0,x1,...   where every drone was at t_ms, cm.
 *                                   A drone that finds a neighbour closer than
 *                                   `react` steps aside from it, on its own
 *   the show       $ARM  $ABT  $END
 *
 * What each drone decides by itself, from its own copy of the plan: where it
 * should be at every moment, when to take off, when to land.  What it
 * decides from the scene: how hard it leans into the wind, and whether a
 * neighbour is too close.  What it decides from its own board: button B
 * takes THIS drone out of the show (it lands where it is), and the knob is a
 * breeze that only this drone feels.
 *
 * All frames are lesson 08's $BODY*CS.  Conductor -> drone:
 *   $WHO                                   who are you?           -> $HLO
 *   $SET,id,seed,rtk,ff,wind_cms,wind_deg  on the ground: you are drone `id`
 *                                          of the plan; go to your pad and warm
 *                                          your GPS up            -> $HLO, $ME
 *   $WND,cms,deg      $ARM -> $HLO        $ABT      $NBR,...
 *   $TCK,t_ms                              fly to t_ms             -> $ME
 *   $END                                   the show is over        -> $FIN, $STL..., $HLO
 *   $PRN                                   print your plan, as text
 * Drone -> conductor:
 *   $HLO,id,state,n,sig,show_ms,stages     sig: a checksum of the plan, in hex.
 *                                          The conductor refuses to fly drones
 *                                          whose plans differ
 *   $ME,id,t_ms,mode,x,y,z,px,py,pz,sx,sy,sz,bat,ax,ay,stage,grounded,push
 *                                          cm: truth, plan now, slot; ax ay the
 *                                          air at the drone, cm/s; push: how far
 *                                          it has stepped aside, cm
 *   $STG,k,kind,name,t0_ms,T_ms            the plan's stages, after $PRN
 *   $FIN,id,sq,n,worst,fsq,fn,fworst,charge,hardest,landed,reacts
 *                                          its own score, floats as their 32 bits
 *                                          in hex, so the PC adds them up exactly
 *   $STL,id,k,last_bad                     per hold: the last moment this drone was
 *                                          out of its slot (hex float, -1: never)
 *
 * Pure C, no hardware: Main.c feeds it the UART's lines, host/drone_pc.c
 * feeds it stdin's, so one PC process is one chip.
 */
#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include "plan.h"
#include "craft.h"
#include "pilot.h"

enum { DS_REFUSED, DS_UNSET, DS_WARMUP, DS_READY, DS_FLYING, DS_LANDING, DS_DONE, DS_COUNT };

/* must = 1: never drop this line; 0: may be dropped if the link is busy */
typedef void (*emit_fn)(const char *line, int must);

typedef struct {
    plan_t   plan;                   /* the whole show: every drone's track      */
    emit_fn  emit;
    int8_t   id;                     /* which track is mine; -1 until $SET       */
    uint8_t  state, feedforward, rtk, aborted, landing_sent, alone;
                                     /* alone: B was pressed, it left the show   */
    uint8_t  lone_pending;           /* B pressed: leave at the next tick        */
    uint32_t seed;
    float    wind, wind_deg;         /* the scene's steady wind                  */
    float    breeze;                 /* this board's knob: m/s, only this drone  */
    /* ---- the airframe and its pilot: lesson 14's drone, one of it -------- */
    craft_t  body;
    pilot_t  brain;
    cmd_t    asked;
    sky_t    sky;
    uint32_t air_ticks;              /* every 10 ms airframe tick since its pad  */
    uint32_t ticks;                  /* since arming: show time = ticks x 10 ms  */
    v3_t     want;                   /* where the plan has it now                */
    uint8_t  stage;
    /* ---- the scene, and what it does about it ----------------------------- */
    v3_t     seen[SHOW_MAX_DRONES];  /* where the others were, last $NBR         */
    uint8_t  seen_n;
    v3_t     push;                   /* stepping aside: added to the plan's goal */
    float    react;                  /* m: a neighbour closer than this = step aside */
    uint16_t reacts;                 /* windows in which it stepped aside        */
    /* ---- its own score ----------------------------------------------------- */
    float    sq, worst, fsq, fworst;
    uint32_t n, fn;
    float    last_bad[PLAN_STAGES];
    uint8_t  hold_seen[PLAN_STAGES];
} drone_t;

int         drone_init(drone_t *d, const show_t *s, emit_fn emit);   /* -1: refused */
void        drone_line(drone_t *d, const char *line);   /* one line from the conductor */
void        drone_leave(drone_t *d);                    /* button B                    */
void        drone_breeze(drone_t *d, float ms);         /* the knob                    */
void        drone_print_plan(drone_t *d);
uint32_t    drone_plan_sig(const plan_t *p);
const char *drone_state_name(uint8_t state);

#endif
