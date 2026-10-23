/* tv_fmu.c - the torque vectoring law of tv_cpp as an FMI 2.0 Co-Simulation
 * FMU (what a Simulink, Modelica or AMESim FMU export provides to vsim).
 * CMake packages it as plugins/tv_fmu.fmu with modelDescription.xml.
 */
#include <stdlib.h>
#include <string.h>

#define VSIM_FMI2_EXPORT_FUNCTIONS
#include "vsim_fmi2.h"

enum {
    /* inputs */
    STEER, VX, YAW_RATE, TQ_FL_IN, TQ_FR_IN, TQ_RL_IN, TQ_RR_IN,
    /* outputs */
    TQ_FL, TQ_FR, TQ_RL, TQ_RR, SHIFT,
    /* parameters */
    GAIN, WHEELBASE, STEER_RATIO, UNDERSTEER, A_MAX, MAX_SHIFT, V_ON,
    N_VARIABLES
};

typedef struct {
    double v[N_VARIABLES];
    fmi2CallbackFunctions callbacks;
    char name[64];
} Instance;

static double clamp(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

static void compute(Instance *s) {
    double *v = s->v;
    const double steer = v[STEER], vx = v[VX], yaw_rate = v[YAW_RATE];
    const double delta = steer / v[STEER_RATIO];
    const double speed = vx > 1.0 ? vx : 1.0;
    double r_ref = vx * delta / (v[WHEELBASE] + v[UNDERSTEER] * vx * vx);
    const double r_lim = v[A_MAX] / speed;
    double fade, shift;
    r_ref = clamp(r_ref, -r_lim, r_lim);
    fade = clamp((vx - v[V_ON]) / 2.0, 0.0, 1.0);
    shift = clamp(v[GAIN] * (r_ref - yaw_rate), -v[MAX_SHIFT], v[MAX_SHIFT]) * fade;
    v[TQ_FL] = v[TQ_FL_IN] - shift;
    v[TQ_FR] = v[TQ_FR_IN] + shift;
    v[TQ_RL] = v[TQ_RL_IN] - shift;
    v[TQ_RR] = v[TQ_RR_IN] + shift;
    v[SHIFT] = shift;
}

const char *fmi2GetVersion(void) { return "2.0"; }
const char *fmi2GetTypesPlatform(void) { return "default"; }

fmi2Component fmi2Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID,
                              fmi2String fmuResourceLocation, const fmi2CallbackFunctions *functions,
                              fmi2Boolean visible, fmi2Boolean loggingOn) {
    Instance *s;
    (void)fmuResourceLocation;
    (void)visible;
    (void)loggingOn;
    if (fmuType != fmi2CoSimulation || fmuGUID == NULL || strcmp(fmuGUID, "{8c4e810f-3df3-4a00-8276-176fa3c9f000}") != 0) {
        return NULL;
    }
    s = (Instance *)calloc(1, sizeof *s);
    if (s == NULL) {
        return NULL;
    }
    s->callbacks = *functions;
    strncpy(s->name, instanceName ? instanceName : "tv_fmu", sizeof s->name - 1);
    s->v[GAIN] = 40.0;
    s->v[WHEELBASE] = 1.53;
    s->v[STEER_RATIO] = 4.5;
    s->v[UNDERSTEER] = 0.0;
    s->v[A_MAX] = 15.0;
    s->v[MAX_SHIFT] = 8.0;
    s->v[V_ON] = 3.0;
    return s;
}

void fmi2FreeInstance(fmi2Component c) { free(c); }

fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean toleranceDefined, fmi2Real tolerance,
                               fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime) {
    (void)c; (void)toleranceDefined; (void)tolerance; (void)startTime; (void)stopTimeDefined; (void)stopTime;
    return fmi2OK;
}
fmi2Status fmi2EnterInitializationMode(fmi2Component c) { (void)c; return fmi2OK; }
fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
    compute((Instance *)c);
    return fmi2OK;
}
fmi2Status fmi2Terminate(fmi2Component c) { (void)c; return fmi2OK; }

fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]) {
    const Instance *s = (const Instance *)c;
    size_t i;
    for (i = 0; i < nvr; ++i) {
        if (vr[i] >= N_VARIABLES) return fmi2Error;
        value[i] = s->v[vr[i]];
    }
    return fmi2OK;
}
fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]) {
    Instance *s = (Instance *)c;
    size_t i;
    for (i = 0; i < nvr; ++i) {
        if (vr[i] >= N_VARIABLES || (vr[i] >= TQ_FL && vr[i] <= SHIFT)) return fmi2Error;
        s->v[vr[i]] = value[i];
    }
    return fmi2OK;
}
fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[]) {
    (void)c; (void)vr; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[]) {
    (void)c; (void)vr; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[]) {
    (void)c; (void)vr; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[]) {
    (void)c; (void)vr; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[]) {
    (void)c; (void)vr; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}

fmi2Status fmi2DoStep(fmi2Component c, fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize,
                      fmi2Boolean noSetFMUStatePriorCommunicationPoint) {
    (void)currentCommunicationPoint;
    (void)communicationStepSize;
    (void)noSetFMUStatePriorCommunicationPoint;
    compute((Instance *)c);
    return fmi2OK;
}
