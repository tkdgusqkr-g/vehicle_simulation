/* vsim_fmi2.h - the FMI 2.0 types and function signatures vsim uses
 * (Functional Mock-up Interface 2.0, Co-Simulation; fmi-standard.org).
 * Written for vsim from the standard's definitions; an FMU compiled
 * against the official fmi2Functions.h is binary compatible.
 *
 * Define VSIM_FMI2_EXPORT_FUNCTIONS before including it to get the exported
 * prototypes for implementing an FMU (as the example FMU does).
 */
#ifndef VSIM_FMI2_H
#define VSIM_FMI2_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void *fmi2Component;
typedef void *fmi2ComponentEnvironment;
typedef void *fmi2FMUstate;
typedef unsigned int fmi2ValueReference;
typedef double fmi2Real;
typedef int fmi2Integer;
typedef int fmi2Boolean;
typedef char fmi2Char;
typedef const fmi2Char *fmi2String;
typedef char fmi2Byte;

#define fmi2True 1
#define fmi2False 0

typedef enum { fmi2OK, fmi2Warning, fmi2Discard, fmi2Error, fmi2Fatal, fmi2Pending } fmi2Status;
typedef enum { fmi2ModelExchange, fmi2CoSimulation } fmi2Type;

typedef void (*fmi2CallbackLogger)(fmi2ComponentEnvironment, fmi2String instanceName, fmi2Status status,
                                   fmi2String category, fmi2String message, ...);
typedef void *(*fmi2CallbackAllocateMemory)(size_t n, size_t size);
typedef void (*fmi2CallbackFreeMemory)(void *p);
typedef void (*fmi2StepFinished)(fmi2ComponentEnvironment, fmi2Status);

typedef struct {
    fmi2CallbackLogger logger;
    fmi2CallbackAllocateMemory allocateMemory;
    fmi2CallbackFreeMemory freeMemory;
    fmi2StepFinished stepFinished;
    fmi2ComponentEnvironment componentEnvironment;
} fmi2CallbackFunctions;

typedef const char *fmi2GetVersionTYPE(void);
typedef fmi2Component fmi2InstantiateTYPE(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID,
                                          fmi2String fmuResourceLocation, const fmi2CallbackFunctions *functions,
                                          fmi2Boolean visible, fmi2Boolean loggingOn);
typedef void fmi2FreeInstanceTYPE(fmi2Component c);
typedef fmi2Status fmi2SetupExperimentTYPE(fmi2Component c, fmi2Boolean toleranceDefined, fmi2Real tolerance,
                                           fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime);
typedef fmi2Status fmi2EnterInitializationModeTYPE(fmi2Component c);
typedef fmi2Status fmi2ExitInitializationModeTYPE(fmi2Component c);
typedef fmi2Status fmi2TerminateTYPE(fmi2Component c);
typedef fmi2Status fmi2GetRealTYPE(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]);
typedef fmi2Status fmi2GetIntegerTYPE(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                      fmi2Integer value[]);
typedef fmi2Status fmi2GetBooleanTYPE(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                      fmi2Boolean value[]);
typedef fmi2Status fmi2SetRealTYPE(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                   const fmi2Real value[]);
typedef fmi2Status fmi2SetIntegerTYPE(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                      const fmi2Integer value[]);
typedef fmi2Status fmi2SetBooleanTYPE(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                      const fmi2Boolean value[]);
typedef fmi2Status fmi2SetStringTYPE(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                     const fmi2String value[]);
typedef fmi2Status fmi2DoStepTYPE(fmi2Component c, fmi2Real currentCommunicationPoint,
                                  fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorCommunicationPoint);

#ifdef VSIM_FMI2_EXPORT_FUNCTIONS
#if defined(_WIN32)
#define VSIM_FMI2_EXPORT __declspec(dllexport)
#else
#define VSIM_FMI2_EXPORT __attribute__((visibility("default")))
#endif
VSIM_FMI2_EXPORT fmi2GetVersionTYPE fmi2GetVersion;
VSIM_FMI2_EXPORT const char *fmi2GetTypesPlatform(void);
VSIM_FMI2_EXPORT fmi2InstantiateTYPE fmi2Instantiate;
VSIM_FMI2_EXPORT fmi2FreeInstanceTYPE fmi2FreeInstance;
VSIM_FMI2_EXPORT fmi2SetupExperimentTYPE fmi2SetupExperiment;
VSIM_FMI2_EXPORT fmi2EnterInitializationModeTYPE fmi2EnterInitializationMode;
VSIM_FMI2_EXPORT fmi2ExitInitializationModeTYPE fmi2ExitInitializationMode;
VSIM_FMI2_EXPORT fmi2TerminateTYPE fmi2Terminate;
VSIM_FMI2_EXPORT fmi2GetRealTYPE fmi2GetReal;
VSIM_FMI2_EXPORT fmi2GetIntegerTYPE fmi2GetInteger;
VSIM_FMI2_EXPORT fmi2GetBooleanTYPE fmi2GetBoolean;
VSIM_FMI2_EXPORT fmi2SetRealTYPE fmi2SetReal;
VSIM_FMI2_EXPORT fmi2SetIntegerTYPE fmi2SetInteger;
VSIM_FMI2_EXPORT fmi2SetBooleanTYPE fmi2SetBoolean;
VSIM_FMI2_EXPORT fmi2SetStringTYPE fmi2SetString;
VSIM_FMI2_EXPORT fmi2DoStepTYPE fmi2DoStep;
#endif

#ifdef __cplusplus
}
#endif

#endif /* VSIM_FMI2_H */
