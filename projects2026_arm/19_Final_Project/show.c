/*
 * show.c - THE SHOW.  Edit this file to make your own, then build.bat.
 *
 * Shapes: SH_LINE SH_V SH_CIRCLE SH_CROSS SH_GRID SH_HEART, or SH_POINTS
 * with your own list of points (one per drone, see `my_w` below).
 * Planes: FLAT lies flat (seen from above, the map); WALL stands up facing
 * the audience in the south (the OLED and the station's 3D view).
 *
 * Every slot must be at least 2 m up and at least `sep` from every other
 * slot.  A show that breaks that, or whose moves cannot be made safe, is
 * REFUSED at power-up - the OLED and the serial monitor say why.  Make the
 * formation bigger and build again.
 */
#include "show.h"

/* An example of SH_POINTS: a "W" standing up, 5 drones.  Not in the show -
 * to fly it, replace one line below with
 *     { SH_POINTS, WALL, 6.0f, 0, 0, 0, 0, 0, "W", show_w },              */
const v3_t show_w[5] = {
    { -8.0f, 10.0f, 14.0f }, { -4.0f, 10.0f, 8.0f }, { 0.0f, 10.0f, 12.0f },
    {  4.0f, 10.0f,  8.0f }, {  8.0f, 10.0f, 14.0f },
};

const show_t SHOW = {
    .name    = "parade",
    .drones  = 5,
    .pads    = 4.0f,
    .takeoff = 5.0f,
    .vmax    = 3.0f,  .vzmax = 1.5f,  .amax = 1.0f,
    .sep     = 3.0f,  .safe  = 1.0f,  .settle = 0.5f,
    .wind    = 3.0f,  .wind_deg = 30.0f,
    .rtk     = 1,
    .n_forms = 7,
    .forms = {
        /* shape      plane  hold    x     y      z     size   turn    name   */
        { SH_LINE,    WALL,  5.0f,  0.0f,  8.0f,  8.0f, 16.0f,   0.0f, "line",   0 },
        { SH_V,       WALL,  6.0f,  0.0f, 10.0f, 11.0f, 12.0f,   0.0f, "v",      0 },
        { SH_CIRCLE,  WALL,  8.0f,  0.0f, 10.0f, 12.0f, 10.0f,   0.0f, "circle", 0 },
        { SH_HEART,   WALL,  8.0f,  0.0f, 10.0f, 13.0f, 14.0f,   0.0f, "heart",  0 },
        { SH_CROSS,   FLAT,  6.0f,  0.0f, 10.0f, 10.0f, 10.0f,   0.0f, "cross",  0 },
        { SH_V,       FLAT,  6.0f,  0.0f, 10.0f, 10.0f, 12.0f, 180.0f, "arrow",  0 },
        { SH_CIRCLE,  WALL,  6.0f,  0.0f, 10.0f, 14.0f, 14.0f,  45.0f, "wide",   0 },
    },
};
