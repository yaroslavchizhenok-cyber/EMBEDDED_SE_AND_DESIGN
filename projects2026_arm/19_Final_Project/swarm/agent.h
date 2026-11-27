/*
 * agent.h - ONE drone of the swarm: everything this chip is
 *
 * One board = one drone.  The chip flies its own airframe (../craft.c) with
 * its own flight computer (../pilot.c) and talks to the rest of the swarm
 * only by radio (radio.h).  There is no plan of the swarm on any chip, no
 * list of members, no leader, and nothing in the loop that flies for it.
 * Everything below is decided HERE, from what this drone has heard:
 *
 *   who is near        its neighbour table (nbr.h): beacons heard, silence dropped
 *   how to move        local rules (steer.h): separation, look-ahead avoidance,
 *                      goal, and - when the mission is GOTO - cohesion and alignment
 *   what the mission is  the newest $MSN it has heard.  It relays it at once and
 *                      every 2 s, so a mission posted near the ground radio
 *                      floods through the swarm hop by hop (multi-hop)
 *   how far from home  its hop count to the ground radio: 1 if it hears the
 *                      ground radio, else 1 + its best neighbour's.  Every
 *                      hop count carries the number of the ground beacon it
 *                      was built on, and only a route whose number keeps
 *                      rising is believed (DSDV: Perkins and Bhagwat, 1994) -
 *                      so a group cut off from home cannot keep a route alive
 *                      by quoting each other's old hop counts
 *   which job          its ROLE - every drone has the same hardware and radio;
 *                      the straps only say which role it starts in:
 *                        MAPPER  covers the survey area cell by cell (cover.h)
 *                        RELAY   places itself between the ground radio and
 *                                the work, so the swarm stays connected
 *                      If the swarm's relay dies, one mapper - picked "at
 *                      random", by an order every drone works out the same
 *                      way - takes its place (agent.c, think())
 *                      (The straps' type 1 was a SENSOR, for sensing points,
 *                      until 2026-11-25; there are none now, and a board
 *                      strapped 1 is a mapper: every drone flies the survey.)
 *   when to stop       its own battery (low: fly home, land,
 *                      recharge on the pad, come back), a lost link (nobody
 *                      heard for 15 s, or no route to the ground radio for
 *                      60 s: fly back toward home - and turn round to work
 *                      again as soon as it hears the swarm), or its own
 *                      belief that the mission is complete (its map is full)
 *
 * What this chip sends on its UART - every $ frame but $PHY and $NBT is
 * RADIO, carried by the station's air to the drones in range:
 *   $BCN $MSN $MAP        radio (radio.h)
 *   $PHY,id,x,y,z,vx,vy,vz,mode,type   the TRUTH, cm: the simulated world, so
 *                         the air knows who is in range of whom.  Never delivered.
 *   $NBT,id,n,id1,q1,...  what this drone believes (its table): for the screen
 *
 * Pure C, no hardware: Main.c drives it from the UART and the board;
 * host/ drives several of it side by side.
 */
#ifndef AGENT_H
#define AGENT_H

#include <stdint.h>
#include "../craft.h"
#include "../pilot.h"
#include "radio.h"
#include "nbr.h"
#include "steer.h"
#include "cover.h"

#define AGENT_TICK_MS  10u        /* agent_tick() every 10 ms                  */
#define BCN_EVERY_MS   200u
#define HOVER_Z        4.0f       /* m: the lowest lane                        */
#define LANE_GAP       1.5f       /* m: drone k cruises at 4, 5.5 or 7 m, by k % 3:
                                     most crossings are then 1.5 m apart in height,
                                     as real swarms and light shows deconflict    */
#define CLEAR_R        2.5f       /* m: no landing or take-off with an airborne
                                     neighbour closer than this, sideways         */
#define PAD_GAP        3.0f       /* m between pads: drone k's is at x = 3(k-1) */
#define NUDGE_MS       2.0f       /* m/s, the joystick at full deflection      */
#define BAT_HOME       30.0f      /* %: below this, fly home                   */
#define BAT_GO         60.0f      /* %: a drone on its pad rejoins above this  */
#define LOST_MS        15000u     /* nobody heard for this long: fly home      */
#define RELAY_GONE_MS  10000u     /* no news of a live relay this long: it is
                                     dead, and a mapper takes its place          */
#define NOROUTE_MS     60000u     /* no route to the ground radio this long: home */
#define CUT_MS         3000u      /* GOTO: no fresh ground beacon number this long:
                                     out of reach - fall back, and learn the edge */
#define PROBE_MIN_MS   2000u      /* in touch: try 1 m further out every probe_ms -
                                     2 s at the quickest (following relays out),  */
#define PROBE_MAX_MS   32000u     /* 32 s at the slowest (a goal truly out of reach) */
#define OBS_KEEP_MS    2000u      /* an obstacle not seen for this long is forgotten */

enum { AG_NOID, AG_GROUND, AG_FLYING, AG_LANDING, AG_HOMING, AG_DEAD, AG_COUNT };
enum { TY_MAPPER = 0, TY_RELAY = 2, TY_COUNT = 3 };    /* the straps' values; 1 is a mapper too */
enum { MS_HOLD, MS_GOTO, MS_SURVEY, MS_HOME, MS_COUNT };

typedef void (*emit_fn)(const char *line, int must);

typedef struct {
    uint16_t mid;                 /* newer replaces older (wrapping compare)   */
    uint8_t  kind;
    float    gx, gy;              /* the ground radio, m                       */
    float    x0, y0, x1, y1;      /* GOTO: the point is (x0, y0); SURVEY: the area */
} mission_t;

typedef struct {
    uint8_t  id, type, state;
    emit_fn  emit;
    /* ---- the airframe and its pilot ------------------------------------ */
    craft_t  body;
    pilot_t  brain;
    cmd_t    asked;
    sky_t    sky;
    uint32_t ticks, now_ms;       /* this chip's own clock                     */
    v3_t     home, spot, hold;    /* its pad; the pilot's setpoint; where to hover */
    v3_t     goal, vcmd;
    float    cruise;              /* m: my lane                                */
    uint8_t  has_goal, avoiding;
    float    stick_x, stick_y, key_x, key_y, breeze;
    uint32_t key_ms;
    uint8_t  keys_down;
    /* ---- the radio ------------------------------------------------------ */
    nbrs_t   nb;
    uint16_t seq;
    uint32_t next_bcn, next_phy, next_nbt, next_exp, next_steer, next_think,
             next_map, next_msn, next_hop;
    uint32_t dice;
    uint32_t tx, rx, rx_bad, rx_old, rx_far, rx_clash, relayed;
    uint32_t heard_ms;            /* anything valid heard, last                */
    uint32_t gnd_ms;              /* the ground radio heard, last (0: never)   */
    uint8_t  hop;
    uint32_t route_ms;            /* when it last had a route to the ground    */
    uint16_t gs;                  /* the newest ground beacon number known     */
    uint32_t gs_ms;               /* when it last rose                         */
    float    relay_f;             /* RELAY: how far out along ground -> work   */
    float    edge;                /* GOTO: m from the ground radio it can work at (0: unknown) */
    float    cut_d;               /* how far out it was when it lost touch        */
    uint32_t cut_ms, edge_ms;     /* since when stale (0: fresh); last edge change */
    uint32_t probe_ms;            /* how long in touch before the next 1 m probe  */
    uint8_t  parked;              /* landed BY HAND: stays down until told        */
    uint8_t  standin;             /* a mapper that took a dead relay's place      */
    uint8_t  scout;               /* a mapper gone to see if the relay lives      */
    uint8_t  relay_known;         /* the swarm has had a relay since I started    */
    uint32_t relay_ms;            /* my freshest news of a live relay (my clock)  */
    obst_t   obs[OBS_MAX];        /* obstacles its rangefinder has seen           */
    uint8_t  nobs;
    uint8_t  falling_back;        /* GOTO: out of reach, flying back into it      */
    uint32_t relay_bad_ms;        /* RELAY: since when its link upstream is bad */
    uint8_t  gnd_seen;
    uint16_t gnd_seq, gnd_qf;     /* the ground radio's last beacon number, and
                                     that link's quality (0..65535, as nbr.h)   */
    uint8_t  via;                 /* the neighbour my route goes through (0: the
                                     ground radio itself, or none)             */
    uint8_t  route_trips;         /* no-route returns in this mission          */
    /* ---- the mission and the jobs ---------------------------------------- */
    mission_t m;
    uint8_t  has_m, done_m;       /* done_m: this drone thinks it is complete  */
    uint16_t finished_mid;
    uint8_t  finished_any;
    uint32_t m_ms;                /* when this mission arrived                 */
    cover_t  cv;
    uint16_t covered;
    const char *why;              /* why it is going home                      */
    /* ---- for the record ---------------------------------------------------- */
    float    closest;             /* m, the nearest neighbour seen while flying */
    uint32_t avoids;
} agent_t;

void        agent_init(agent_t *a, uint8_t id, uint8_t type, emit_fn emit);
void        agent_tick(agent_t *a);                     /* every AGENT_TICK_MS */
int         agent_line(agent_t *a, const char *line);   /* 1: it was a $ frame */
int         agent_command(agent_t *a, char *line);      /* 0: not a command    */
void        agent_takeoff(agent_t *a);
void        agent_land(agent_t *a);
void        agent_kill(agent_t *a);                     /* a crash: motors off, radio dead */
void        agent_stick(agent_t *a, float vx, float vy);
void        agent_breeze(agent_t *a, float ms);
void        agent_print_nbrs(agent_t *a);
const char *agent_state_name(uint8_t s);
const char *agent_type_name(uint8_t t);
int         msn_encode(const mission_t *m, char *buf, int size);   /* the frame's body */
int         msn_decode(const char *line, mission_t *m);           /* 0: ok            */

#endif
