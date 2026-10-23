/* Shaped like Embedded Coder output for a single-rate model with
 * nonreusable functions (tests of the vsim wrapper only). */
#ifndef FAKE_VCU_H
#define FAKE_VCU_H
#include "rtwtypes.h"

typedef struct {
  real_T APPS;              /* '<Root>/APPS' [%] */
  real32_T WheelSpeedFL;    /* '<Root>/WheelSpeedFL' [rpm] */
  boolean_T Enable;         /* '<Root>/Enable' */
} ExtU_fake_vcu_T;

typedef struct {
  real_T TorqueCmd;         /* '<Root>/TorqueCmd' [N*m] */
  real32_T WheelSpeedEcho;  /* '<Root>/WheelSpeedEcho' [rpm] */
  int32_T Count;            /* '<Root>/Count' */
} ExtY_fake_vcu_T;

extern ExtU_fake_vcu_T fake_vcu_U;
extern ExtY_fake_vcu_T fake_vcu_Y;

extern void fake_vcu_initialize(void);
extern void fake_vcu_step(void);
extern void fake_vcu_terminate(void);
#endif
