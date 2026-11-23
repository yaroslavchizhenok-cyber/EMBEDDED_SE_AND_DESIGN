/*
 * show.h - what a show is: its rules and its formations
 *
 * A show is data, not code.  show.c holds the one this firmware flies -
 * edit it, rebuild, fly again.  plan.c turns it into a timed track for
 * every drone and refuses it, before take-off, if two drones would ever be
 * planned closer than `sep`.
 *
 * Frame: x = east, y = north, z = up, metres.  The pads are in a row along x
 * at y = 0, centred on 0.  The audience stands south of them, looking north.
 *
 * Pure C, no hardware.
 */
#ifndef SHOW_H
#define SHOW_H

#include <stdint.h>
#include "contract.h"

#define SHOW_MAX_FORMS 10
/* Drones in one show.  Every chip holds the whole plan (so it can be checked
 * on every chip), and that plan has a row per drone: 5 on this chip's RAM. */
#ifndef SHOW_MAX_DRONES
#define SHOW_MAX_DRONES 5
#endif

enum { SH_LINE, SH_V, SH_CIRCLE, SH_CROSS, SH_GRID, SH_HEART, SH_POINTS, SH_COUNT };
enum { FLAT, WALL };         /* lying flat (seen from above) / standing up, facing south */

typedef struct {
    uint8_t     shape;       /* SH_*                                              */
    uint8_t     plane;       /* FLAT or WALL                                      */
    float       hold;        /* s, how long the formation is held                 */
    float       x, y, z;     /* its centre, m                                     */
    float       size;        /* m across                                          */
    float       turn;        /* degrees, about the vertical                       */
    const char *name;        /* shown on the OLED and in the station (no spaces)  */
    const v3_t *points;      /* SH_POINTS only: one x, y, z per drone             */
} form_t;

typedef struct {
    const char *name;
    uint8_t drones;          /* how many fly, 1 .. SHOW_MAX_DRONES                      */
    float   pads;            /* m between launch pads                             */
    float   takeoff;         /* m - everyone first climbs straight up to this     */
    float   vmax, vzmax;     /* m/s - fastest any drone is asked to fly / climb   */
    float   amax;            /* m/s^2 - hardest it is asked to speed up or brake  */
    float   sep;             /* m - PLANNED: no two drones closer than this, ever */
    float   safe;            /* m - FLOWN: closer than this and the show fails    */
    float   settle;          /* m - "in formation" = every drone this close       */
    float   wind, wind_deg;  /* m/s, blowing toward degrees (0 east, 90 north)    */
    uint8_t rtk;             /* 1: RTK GPS (2 cm, show drones); 0: plain (0.5 m)  */
    uint8_t n_forms;
    form_t  forms[SHOW_MAX_FORMS];
} show_t;

extern const show_t SHOW;    /* the show this firmware flies: show.c */
extern const v3_t   show_w[5];/* an SH_POINTS example, show.c          */

#endif
