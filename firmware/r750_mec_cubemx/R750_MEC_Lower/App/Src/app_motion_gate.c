#include "app_motion_gate.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

void AppMotionGate_Init(AppMotionGate *gate)
{
  if (gate != NULL) {
    memset(gate, 0, sizeof(*gate));
  }
}

bool AppMotionGate_ComputeAvailable(
  bool runtime_ready,
  bool motion_inhibited,
  bool fault_latched,
  bool imu_motion_usable)
{
  return runtime_ready && !motion_inhibited && !fault_latched && imu_motion_usable;
}

bool AppMotionGate_Update(AppMotionGate *gate, bool available)
{
  if (gate == NULL) {
    return false;
  }
  if (gate->available && !available) {
    if (gate->generation == UINT32_MAX) {
      gate->available = false;
      return false;
    }
    gate->generation++;
  }
  gate->available = available;
  return true;
}

AppMotionPrepareResult AppMotionGate_PrepareCommand(
  const AppMotionGate *gate,
  const AppCommCommand *command,
  AppMotionGatedRequest *request)
{
  if (gate == NULL || command == NULL || request == NULL) {
    return APP_MOTION_PREPARE_INVALID;
  }

  AppMotorRequestType request_type;
  switch (command->type) {
    case APP_COMM_COMMAND_ARM:
      request_type = APP_MOTOR_REQUEST_ARM;
      break;
    case APP_COMM_COMMAND_PWM:
      request_type = APP_MOTOR_REQUEST_SET_PWM;
      break;
    case APP_COMM_COMMAND_STOP:
      request_type = APP_MOTOR_REQUEST_STOP;
      break;
    case APP_COMM_COMMAND_ESTOP:
    case APP_COMM_COMMAND_STATUS:
      return APP_MOTION_PREPARE_NOT_MOTION_COMMAND;
    case APP_COMM_COMMAND_NONE:
    default:
      return APP_MOTION_PREPARE_INVALID;
  }

  if (!gate->available) {
    return APP_MOTION_PREPARE_GATE_REJECTED;
  }

  request->request.type = request_type;
  request->request.sequence = command->sequence;
  request->request.pwm = command->pwm;
  request->gate_generation = gate->generation;
  return APP_MOTION_PREPARE_READY;
}

bool AppMotionGate_IsRequestCurrent(
  const AppMotionGate *gate,
  const AppMotionGatedRequest *request)
{
  return gate != NULL && request != NULL && gate->available &&
         request->gate_generation == gate->generation;
}
