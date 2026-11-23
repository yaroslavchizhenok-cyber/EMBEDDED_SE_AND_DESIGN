/*
 * tasks.h - who does which job: an auction with no auctioneer
 *
 * A mission may carry up to TASK_MAX sensing points ("hover here 5 s and
 * take a reading").  Nobody hands them out.  Every drone keeps its OWN copy
 * of who it thinks has each one, gossips that copy once a second, and
 * merges every copy it hears by one rule all drones share:
 *
 *   the LOWER BID wins (ties: the lower id).
 *
 * A bid is a cost in decimetres: the distance to the point, plus a penalty
 * for a low battery (TASK_BAT_DM per % below 100) - a tired drone bids
 * high, so a fresher one takes the job from it.  To take a task a drone
 * must beat the holder by TASK_MARGIN_DM, so two equal drones do not swap
 * it back and forth.  Because every drone applies the same rule to the same
 * claims, the copies agree once the gossip has spread - with no one in
 * charge (a "consensus-based auction": Choi, Brunet and How, 2009, CBBA,
 * cut down to one task per drone).
 *
 * WHEN THE WINNER DIES.  A winner bumps its claim's stamp every second.
 * Every copy also carries how old the stamp is.  A claim whose stamp has not
 * moved for TASK_STALE_MS is void in everyone's copy, and the task goes back
 * on the market.  An old claim is never allowed to unseat a live one: one
 * already older than TASK_STALE_MS when it arrives is ignored.  Drones'
 * clocks are not synchronised, so only AGES travel, never times.
 *
 * Giving a task up on purpose (low battery, going home): the winner sends
 * its claim with bid TASK_INF and a new stamp, and anyone outbids that.
 *
 * Pure C, no hardware: host/ compiles it too.
 */
#ifndef TASKS_H
#define TASKS_H

#include <stdint.h>
#include "../contract.h"

#define TASK_MAX        4u
#define TASK_INF        65535u
#define TASK_STALE_MS   4000u
#define TASK_MARGIN_DM  20u        /* 2 m better, or keep out                 */
#define TASK_BAT_DM     4u         /* per % of battery used                   */
#define TASK_HOLD_MS    5000u      /* hover this long at the point: done      */
#define TASK_AT_M       1.0f       /* ... within this                          */

typedef struct {
    float    x, y;                 /* m                                        */
    uint8_t  winner, done;         /* winner 0: nobody                         */
    uint16_t bid, stamp;
    uint32_t seen_ms;              /* (my clock) when the stamp last moved     */
} task_t;

typedef struct {
    task_t   t[TASK_MAX];
    uint8_t  n;
    int8_t   mine;                 /* the task I hold, or -1                   */
    uint16_t my_stamp;
    uint32_t claims, losses, voided;
} tasks_t;

void     tasks_reset(tasks_t *k, int n, const float *xy);       /* xy: x0,y0,x1,y1... m */
uint16_t tasks_cost(const task_t *t, v3_t me, float bat);
/* What I heard about task i: winner w, bid b, stamp s, age (0.1 s), done d. */
void     tasks_merge(tasks_t *k, uint8_t self, int i, uint8_t w, uint16_t b, uint16_t s,
                     uint16_t age_ds, uint8_t d, uint32_t now);
void     tasks_expire(tasks_t *k, uint8_t self, uint32_t now);
int      tasks_bid(tasks_t *k, uint8_t self, v3_t me, float bat, uint32_t now);   /* -> mine */
void     tasks_refresh(tasks_t *k, uint8_t self, v3_t me, float bat, uint32_t now);
void     tasks_release(tasks_t *k, uint8_t self, uint32_t now);
void     tasks_finish(tasks_t *k, int i);
int      tasks_all_done(const tasks_t *k);
uint16_t tasks_age_ds(const task_t *t, uint32_t now);

#endif
