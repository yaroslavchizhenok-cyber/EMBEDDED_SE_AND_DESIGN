/*
 * nbr.h - who I can hear: one drone's neighbour table
 *
 * Everything a drone knows about the rest of the swarm is in here, and all
 * of it came from beacons it heard itself.  There is no list of members to
 * download and nobody to ask: a drone that is heard is added, a drone that
 * goes silent for NBR_TIMEOUT_MS is dropped.
 *
 * The table has NBR_MAX rows and never grows.  That is the scalability
 * property in one line: a drone's memory and work are the same whether the
 * swarm is 3 drones or 3000, because it only ever keeps its nearest few.
 * When the table is full it keeps the NEAREST: a new voice that is closer
 * than the farthest row replaces it, one that is farther is not kept (it is
 * heard, counted in `full`, and let go).  Near drones are the ones that
 * matter - they are the ones a drone can collide with.
 *
 * Link quality, per row, 0..100 %: every beacon carries a sequence number,
 * so a gap of k numbers is k beacons this drone missed.  Each beacon heard
 * moves the quality 1/8 of the way toward 100, each one missed 1/8 of the
 * way toward 0 - a moving average over the last ~8 beacons.  It is kept in
 * 16 bits (qf, 0..65535) and shown in % (q): in 0..100 steps, the rounding
 * of each 1/8 would bias it upward by 10 points.
 *
 * Times are this drone's OWN milliseconds.  Drones' clocks are not
 * synchronised and need not be: "silent for 2 s" is measured by the one
 * who is listening.
 *
 * Pure C, no hardware: host/ compiles it too.
 */
#ifndef NBR_H
#define NBR_H

#include <stdint.h>
#include "../contract.h"
#include "radio.h"

#define NBR_MAX         8u
#define NBR_TIMEOUT_MS  3000u     /* 15 beacons missed in a row: gone.  (2 s made a
                                     neighbour at the fading edge of range flap in
                                     and out of the table every few seconds.)     */

typedef struct {
    uint8_t  id, mode, bat, nn, type, hop;
    uint8_t  q;                   /* link quality, %                            */
    uint16_t qf;                  /* the same, 0..65535: the average itself     */
    uint16_t seq;                 /* the last sequence number heard             */
    uint16_t gs;                  /* the ground beacon its route is built on    */
    uint32_t heard_ms;            /* when (my clock) it was last heard          */
    uint16_t rx, missed;          /* beacons heard, missed (seq gaps): wrap, fine */
    v3_t     pos, vel;            /* what it SAID about itself, m and m/s       */
    int16_t  gx, gy;              /* where it said it is heading, dm            */
} nbr_t;

typedef struct {
    nbr_t    e[NBR_MAX];
    uint8_t  n;                   /* rows in use: e[0] .. e[n-1]                */
    uint32_t joins, losses, evictions, stale, full;
} nbrs_t;

enum { NBR_SEEN = 0, NBR_NEW = 1, NBR_OLD = -1, NBR_SELF = -2, NBR_FAR = -3 };

void         nbr_init(nbrs_t *t);

/* A beacon heard at `now`.  NBR_NEW: a new row; NBR_SEEN: a row updated;
 * NBR_OLD: a repeat or an out-of-order beacon, ignored; NBR_SELF: it claims
 * to be me (two boards with the same ID straps); NBR_FAR: the table is full
 * of nearer drones.  `me` is where I think I am, to judge "nearer".
 * *evicted is set to the id a full table threw out, or 0. */
int          nbr_heard(nbrs_t *t, uint8_t self, v3_t me, const bcn_t *b, uint32_t now, uint8_t *evicted);

/* Drops every row silent for NBR_TIMEOUT_MS.  Their ids go into gone[] (at
 * most `max`); returns how many were dropped. */
int          nbr_expire(nbrs_t *t, uint32_t now, uint8_t *gone, int max);

const nbr_t *nbr_find(const nbrs_t *t, uint8_t id);

#endif
