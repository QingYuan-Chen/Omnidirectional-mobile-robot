#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "bsp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

BspStatus BspMotor_Init(void);
BspStatus BspMotor_SetPwm(BspMotorId motor, int16_t pwm);
void BspMotor_Brake(BspMotorId motor);
void BspMotor_BrakeAll(void);
void BspMotor_Coast(BspMotorId motor);
void BspMotor_CoastAll(void);
void BspMotor_EmergencyCoastAll(void);

#ifdef __cplusplus
}
#endif

#endif
