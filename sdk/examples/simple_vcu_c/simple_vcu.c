/* simple_vcu.c - a minimal VCU as a native plugin in C (the `vcu` slot).
 *
 * Linear accelerator map to the four motors, drive torque cut while braking,
 * no regen, no traction control. Shows the whole C ABI in one file.
 *
 *   components:
 *     vcu: {type: native, path: simple_vcu, params: {torque_max: 21}}
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vsim_plugin.h"

typedef struct {
    double torque_max;  /* [N*m] per motor at full pedal */
    double brake_cut;   /* brake pedal above which drive torque is cut */
} SimpleVcu;

/* Ports named after bus signals connect themselves; no mapping needed. */
static const VsimPort ports[] = {
    {"drv.cmd.throttle", "-", VSIM_IN},
    {"drv.cmd.brake", "-", VSIM_IN},
    {"pt.motor.fl.torque_cmd", "N*m", VSIM_OUT},
    {"pt.motor.fr.torque_cmd", "N*m", VSIM_OUT},
    {"pt.motor.rl.torque_cmd", "N*m", VSIM_OUT},
    {"pt.motor.rr.torque_cmd", "N*m", VSIM_OUT},
};

static int describe(const VsimPort **p, int *n) {
    *p = ports;
    *n = (int)(sizeof ports / sizeof ports[0]);
    return 0;
}

/* A number from a flat JSON object, or the fallback. */
static double param(const char *json, const char *key, double fallback) {
    char pattern[64];
    const char *at;
    snprintf(pattern, sizeof pattern, "\"%s\":", key);
    at = strstr(json, pattern);
    return at ? strtod(at + strlen(pattern), NULL) : fallback;
}

static void *create(const char *params_json, double period, const VsimHost *host) {
    SimpleVcu *self = (SimpleVcu *)calloc(1, sizeof *self);
    (void)period;
    if (self == NULL) {
        return NULL;
    }
    self->torque_max = param(params_json, "torque_max", 21.0);
    self->brake_cut = param(params_json, "brake_cut", 0.05);
    if (!(self->torque_max > 0.0)) {
        host->log(host->context, VSIM_LOG_ERROR, "torque_max must be positive");
        free(self);
        return NULL;
    }
    return self;
}

static int step(void *p, double t, const double *in, double *out) {
    const SimpleVcu *self = (const SimpleVcu *)p;
    double throttle = in[0], torque;
    const double brake = in[1];
    int i;
    (void)t;
    if (throttle < 0.0) throttle = 0.0;
    if (throttle > 1.0) throttle = 1.0;
    torque = brake > self->brake_cut ? 0.0 : throttle * self->torque_max;
    for (i = 0; i < 4; ++i) {
        out[i] = torque;
    }
    return 0;
}

static void destroy(void *p) { free(p); }

static const VsimPluginApi api = {VSIM_PLUGIN_ABI_VERSION, "simple_vcu", "1.0", 0.01, describe, create, step, destroy};

VSIM_PLUGIN_EXPORT const VsimPluginApi *vsim_plugin_entry(void) { return &api; }
