/* vsim_plugin.h - the C ABI of vsim native plugins (design doc §11.6).
 *
 * A plugin is a shared library (.dll / .so / .dylib) that exports one
 * function, vsim_plugin_entry, returning a table of functions. vsim calls
 * create() once, step() every `period` seconds of simulated time, and
 * destroy() at the end. Values are doubles in the units the plugin declares;
 * vsim converts from and to the signals they are connected to
 * (components.<name>.ports in the vehicle project).
 *
 *   #include "vsim_plugin.h"
 *   static const VsimPort ports[] = {
 *       {"throttle", "-", VSIM_IN},
 *       {"torque", "N*m", VSIM_OUT},
 *   };
 *   static int describe(const VsimPort **p, int *n) { *p = ports; *n = 2; return 0; }
 *   static void *create(const char *params_json, double period, const VsimHost *host) { ... }
 *   static int step(void *self, double t, const double *in, double *out) { out[0] = 230.0 * in[0]; return 0; }
 *   static void destroy(void *self) { ... }
 *   static const VsimPluginApi api = {VSIM_PLUGIN_ABI_VERSION, "my_vcu", "1.0", 0.01,
 *                                     describe, create, step, destroy};
 *   VSIM_PLUGIN_EXPORT const VsimPluginApi *vsim_plugin_entry(void) { return &api; }
 *
 * Inputs arrive in `in` in the order the input ports are listed, outputs go
 * to `out` in the order of the output ports. A plugin must not use the wall
 * clock or its own unseeded random numbers if runs are to stay repeatable;
 * `t` is the simulated time and params may carry a seed.
 */
#ifndef VSIM_PLUGIN_H
#define VSIM_PLUGIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define VSIM_PLUGIN_ABI_VERSION 1

#if defined(_WIN32)
#define VSIM_PLUGIN_EXPORT __declspec(dllexport)
#else
#define VSIM_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

enum { VSIM_IN = 0, VSIM_OUT = 1 };
enum { VSIM_LOG_INFO = 0, VSIM_LOG_WARNING = 1, VSIM_LOG_ERROR = 2 };

typedef struct VsimPort {
    const char *name;  /* unique within the plugin */
    const char *unit;  /* e.g. "rad/s", "rpm", "N*m", "-"; "" for none */
    int direction;     /* VSIM_IN or VSIM_OUT */
} VsimPort;

/* Services of the host, valid from create() until destroy(). */
typedef struct VsimHost {
    void *context;
    void (*log)(void *context, int level, const char *message);
} VsimHost;

typedef struct VsimPluginApi {
    int abi_version;        /* VSIM_PLUGIN_ABI_VERSION */
    const char *name;       /* plugin name and version, recorded with each run */
    const char *version;
    double default_period;  /* [s] used when the project gives none; 0: must be given */
    /* Ports, valid for the library's lifetime. Returns 0 on success. */
    int (*describe)(const VsimPort **ports, int *n_ports);
    /* A new instance; params_json is the component's `params` as a JSON
     * object ("{}" if none). Returns NULL on failure (log why first). */
    void *(*create)(const char *params_json, double period, const VsimHost *host);
    /* One period at simulated time t. Returns 0 on success; anything else stops the run. */
    int (*step)(void *self, double t, const double *in, double *out);
    void (*destroy)(void *self);
} VsimPluginApi;

typedef const VsimPluginApi *(*VsimPluginEntry)(void);

VSIM_PLUGIN_EXPORT const VsimPluginApi *vsim_plugin_entry(void);

#ifdef __cplusplus
}
#endif

#endif /* VSIM_PLUGIN_H */
