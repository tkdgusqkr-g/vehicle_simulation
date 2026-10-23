#include "fake_vcu.h"

ExtU_fake_vcu_T fake_vcu_U;
ExtY_fake_vcu_T fake_vcu_Y;
static int32_T count;

void fake_vcu_initialize(void) {
  count = 0;
  fake_vcu_Y.TorqueCmd = 0.0;
  fake_vcu_Y.WheelSpeedEcho = 0.0F;
  fake_vcu_Y.Count = 0;
}

void fake_vcu_step(void) {
  fake_vcu_Y.TorqueCmd = fake_vcu_U.Enable ? 0.21 * fake_vcu_U.APPS : 0.0;
  fake_vcu_Y.WheelSpeedEcho = fake_vcu_U.WheelSpeedFL;
  fake_vcu_Y.Count = ++count;
}

void fake_vcu_terminate(void) {}
