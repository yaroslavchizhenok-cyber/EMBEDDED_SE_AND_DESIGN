/*
 * radio.h - what goes over the air
 *
 * Every frame is lesson 08's $BODY*CS line: printable, and a corrupted one
 * is thrown away by its checksum.  Nobody is addressed - everything is a
 * broadcast that whoever is in range hears.
 *
 *   $BCN,id,seq,x,y,z,vx,vy,vz,bat,mode,nn,type,hop,gx,gy,gs  5 x a second
 *        the BEACON: who I am, where I believe I am (cm) and how fast (cm/s),
 *        battery %, mode (AG_*), neighbours I hear, my type (TY_*), my hops
 *        to the ground radio (HOP_NONE: no route), where I am heading (dm),
 *        and the newest ground-radio beacon number my route is built on
 *   $MSN,mid,kind,gx,gy,x0,y0,x1,y1,np,p1x,p1y,...             relayed
 *        the MISSION, posted by the operator's ground radio and FLOODED
 *        drone to drone: a newer mid replaces an older one (agent.c)
 *   $TSK,from,mid,n, w,b,s,a,d, ...                           gossip, 1 Hz
 *        what I know of the mission's sensing tasks: per task its winner,
 *        winning bid (dm), the winner's stamp, how old that stamp is (0.1 s)
 *        and whether it is done (tasks.h)
 *   $MAP,from,mid,hi,lo                                        gossip, 1 Hz
 *        which cells of the survey area I know are covered (cover.h)
 *   $GND,seq,x,y                                               1 Hz
 *        the ground radio's own beacon: where it is (dm)
 *
 * Pure C, no hardware: host/ compiles it too.
 */
#ifndef RADIO_H
#define RADIO_H

#include <stddef.h>
#include <stdint.h>

#define RADIO_MAX_ID 63u           /* the straps give 1..7; the PC tests use more */
#define RADIO_LINE   120u          /* any frame, $ to \n, always fits             */
#define HOP_NONE     255u

typedef struct {
    uint8_t  id, mode, bat, nn, type, hop;
    uint16_t seq, gs;
    int32_t  x, y, z;              /* cm   */
    int32_t  vx, vy, vz;           /* cm/s */
    int32_t  gx, gy;               /* dm: where it is heading */
} bcn_t;

enum { BCN_OK = 0, BCN_E_FRAME = -1, BCN_E_NAME = -2, BCN_E_FIELDS = -3, BCN_E_RANGE = -4 };

int bcn_encode(const bcn_t *b, char *buf, size_t size);   /* "\n" included; its length */
int bcn_decode(const char *line, bcn_t *b);               /* BCN_OK, or why not        */

/* Any frame of integers: checks the checksum and the name, then reads up to
 * `max` comma-separated integers into out[].  Returns how many, or < 0. */
int frame_ints(const char *line, const char *name, int32_t *out, int max);

#endif
