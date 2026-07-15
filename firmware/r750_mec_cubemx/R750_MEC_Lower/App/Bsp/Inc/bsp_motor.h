#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "bsp_types.h"

/* 电机 BSP 封装四路方向、PWM、制动和空转，安全层始终拥有最终覆盖权。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 启动 PWM 通道前保持全部驱动为空转。 */
BspStatus BspMotor_Init(void);
/* 设置有符号 PWM，绝对值在 BSP 满量程内限幅。 */
BspStatus BspMotor_SetPwm(BspMotorId motor, int16_t pwm);
/* 普通制动保持 PWM 基础设施运行，可在控制状态机中恢复。 */
void BspMotor_Brake(BspMotorId motor);
void BspMotor_BrakeAll(void);
/* 普通空转只改变驱动输出，不关闭定时器。 */
void BspMotor_Coast(BspMotorId motor);
void BspMotor_CoastAll(void);
/* 紧急空转会停止 PWM 基础设施，只允许系统复位恢复。 */
void BspMotor_EmergencyCoastAll(void);

#ifdef __cplusplus
}
#endif

#endif
