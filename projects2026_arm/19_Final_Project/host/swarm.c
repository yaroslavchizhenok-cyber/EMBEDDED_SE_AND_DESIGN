/*
 * swarm.c - the firmware's fleet as a PC library (swarm.dll), for Python
 *
 * The SAME fleet.c, pilot.c and craft.c the chip runs, built by the PC's gcc
 * with FLEET_MAX raised to 64 - so host/show.py can fly 25 drones in a few
 * seconds and score them, which the chip (5 drones, slow motion in Wokwi)
 * cannot.  This file only gives the fleet a C ABI ctypes can call.
 *
 * ctypes believes whatever it is told about a struct, and a wrong layout
 * reads wrong numbers rather than crashing.  So the library reports its own
 * layout (swarm_drone_size, swarm_drone_layout) and fleet.py refuses to run
 * if it disagrees field by field.
 */
#include <stddef.h>
#include "fleet.h"

#ifdef _WIN32
#  define API __declspec(dllexport)
#else
#  define API __attribute__((visibility("default")))
#endif

#define SWARM_VERSION 2              /* fleet.py's VERSION must match */

API int swarm_version(void)    { return SWARM_VERSION; }
API int swarm_drone_size(void) { return (int)sizeof(fleet_drone_t); }

API int swarm_drone_layout(uint32_t *off, int max)
{
    static const uint32_t at[] = {
        offsetof(fleet_drone_t, t_ms),  offsetof(fleet_drone_t, mode),
        offsetof(fleet_drone_t, why),   offsetof(fleet_drone_t, grounded),
        offsetof(fleet_drone_t, fixes),
        offsetof(fleet_drone_t, x),  offsetof(fleet_drone_t, y),  offsetof(fleet_drone_t, z),
        offsetof(fleet_drone_t, vx), offsetof(fleet_drone_t, vy), offsetof(fleet_drone_t, vz),
        offsetof(fleet_drone_t, ex), offsetof(fleet_drone_t, ey), offsetof(fleet_drone_t, ez),
        offsetof(fleet_drone_t, gx), offsetof(fleet_drone_t, gy), offsetof(fleet_drone_t, gz),
        offsetof(fleet_drone_t, roll), offsetof(fleet_drone_t, pitch), offsetof(fleet_drone_t, yaw),
        offsetof(fleet_drone_t, lift), offsetof(fleet_drone_t, charge),
        offsetof(fleet_drone_t, air_x), offsetof(fleet_drone_t, air_y),
        offsetof(fleet_drone_t, hardest_landing),
    };
    int n = (int)(sizeof at / sizeof at[0]);
    for (int k = 0; k < n && k < max; k++) { off[k] = at[k]; }
    return n;
}

API int      swarm_init(int n, uint32_t seed)          { return fleet_init(n, seed); }
API int      swarm_count(void)                         { return fleet_count(); }
API void     swarm_place(int i, float x, float y)      { fleet_place(i, x, y); }
API void     swarm_wind(float speed, float toward)     { fleet_wind(speed, toward); }
API void     swarm_gps(float wander, float jitter)     { fleet_gps(wander, jitter); }
API int      swarm_arm(int i)                          { return fleet_arm(i); }
API int      swarm_land(int i)                         { return fleet_land(i); }
API void     swarm_step(void)                          { fleet_step(); }
API uint32_t swarm_time_ms(void)                       { return fleet_time_ms(); }
API int      swarm_state(int i, fleet_drone_t *out)    { return fleet_view(i, out); }

API void swarm_set_target(int i, float x, float y, float z, float vx, float vy, float vz)
{
    fleet_target(i, (v3_t){ x, y, z }, (v3_t){ vx, vy, vz });
}

API void swarm_run(int ticks)
{
    while (ticks-- > 0) { fleet_step(); }
}

const char *pilot_mode_name(uint8_t mode);
API const char *swarm_mode_name(int mode) { return pilot_mode_name((uint8_t)mode); }
