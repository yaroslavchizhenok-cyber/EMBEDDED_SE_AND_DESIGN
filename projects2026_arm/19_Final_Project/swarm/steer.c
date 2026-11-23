/*
 * steer.c - local rules.  See steer.h.
 */
#include "steer.h"
#include "../trig.h"

#define K_SEP    1.5f          /* separation gain                               */
#define K_AVOID  1.0f          /* avoidance: fraction of vmax at a dead-centre pass */
#define K_GOAL   0.6f          /* 1/s: goal gap -> speed                        */
#define MAX_AGE  0.5f          /* s: a beacon older than this is not moved on further */

static float len2(float x, float y) { return t_sqrt(x * x + y * y); }

void steer(const steer_in_t *in, const nbrs_t *nb, uint32_t now_ms, steer_out_t *out)
{
    float vmax = in->vmax > 0.0f && in->vmax < STEER_VMAX ? in->vmax : STEER_VMAX;
    float sx = 0, sy = 0;                         /* safety: separation + avoidance */
    float cx = 0, cy = 0, ax = 0, ay = 0;         /* cohesion, alignment sums       */
    int   flock = 0;
    float brake = 1.0f;                           /* goal speed factor          */
    out->closest = 99.0f;
    out->avoiding = 0;

    for (unsigned i = 0; i < nb->n; i++) {
        const nbr_t *o = &nb->e[i];
        float age = (float)(now_ms - o->heard_ms) * 0.001f;
        if (age > MAX_AGE) { age = MAX_AGE; }
        float px = o->pos.x + o->vel.x * age, py = o->pos.y + o->vel.y * age;
        float pz = o->pos.z + o->vel.z * age;
        float rx = px - in->me.x, ry = py - in->me.y, rz = pz - in->me.z;
        float d = t_sqrt(rx * rx + ry * ry + rz * rz);
        if (d < out->closest) { out->closest = d; }
        if (pz < 0.5f && in->me.z > 1.5f) { continue; }   /* parked below me: no threat */
        float dh = len2(rx, ry);

        /* separation: push straight away, ((SEP/d)^2 - 1): 0 at SEP, 3 at SEP/2 */
        if (d < STEER_SEP) {
            float r = STEER_SEP / (d > 0.2f ? d : 0.2f);
            float k = K_SEP * (r * r - 1.0f);
            if (dh > 0.01f) { sx -= k * rx / dh;  sy -= k * ry / dh; }
            else            { sx += k * (in->me.x >= px ? 1.0f : -1.0f); }   /* stacked: pick a side */
        }

        /* avoidance: the closest approach in the next STEER_HORIZ seconds */
        float wx = o->vel.x - in->vel.x, wy = o->vel.y - in->vel.y;
        float closing = rx * wx + ry * wy;                 /* < 0: getting closer */
        float w2 = wx * wx + wy * wy;
        if (closing < 0.0f && w2 > 0.01f) {
            float t = -closing / w2;
            if (t < STEER_HORIZ) {
                float qx = rx + wx * t, qy = ry + wy * t;  /* where it will be, from me */
                float dmin = len2(qx, qy);
                if (dmin < STEER_AVOID && t > 0.0f) {
                    float k = K_AVOID * vmax * (1.0f - dmin / STEER_AVOID) * (1.0f - 0.5f * t / STEER_HORIZ);
                    float b = 0.3f + 0.7f * dmin / STEER_AVOID;   /* 0.3 at a dead-centre pass */
                    if (b < brake) { brake = b; }
                    float ux, uy;
                    if (dmin > 0.3f) { ux = -qx / dmin;  uy = -qy / dmin; }  /* open the gap */
                    else {                                   /* head-on: keep RIGHT */
                        ux = ry / (dh > 0.01f ? dh : 1.0f);  uy = -rx / (dh > 0.01f ? dh : 1.0f);
                    }
                    sx += k * ux;  sy += k * uy;
                    out->avoiding = 1;
                }
            }
        }

        if (in->k_coh > 0.0f || in->k_ali > 0.0f) {
            if (o->mode == 2u) {                           /* flying neighbours only */
                cx += px;  cy += py;  ax += o->vel.x;  ay += o->vel.y;  flock++;
            }
        }
    }

    /* the safety part first, capped at vmax */
    float s = len2(sx, sy);
    if (s > vmax) { sx *= vmax / s;  sy *= vmax / s;  s = vmax; }

    /* then the goal (and the flock), in whatever speed is left */
    float gx = 0, gy = 0;
    if (in->has_goal) {
        float ex = in->goal.x - in->me.x, ey = in->goal.y - in->me.y, e = len2(ex, ey);
        float pull = e > in->slack ? K_GOAL * (e - in->slack) : 0.0f;    /* m/s */
        if (e > 0.01f) { gx = pull * ex / e;  gy = pull * ey / e; }
    }
    if (flock) {
        float n = (float)flock;
        gx += in->k_coh * (cx / n - in->me.x) + in->k_ali * (ax / n - in->vel.x);
        gy += in->k_coh * (cy / n - in->me.y) + in->k_ali * (ay / n - in->vel.y);
    }
    gx *= brake;  gy *= brake;
    float left = vmax - s, g = len2(gx, gy);
    if (g > left) { gx *= left / g;  gy *= left / g; }

    out->v = (v3_t){ sx + gx, sy + gy, 0.0f };
}
