#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "bsp_types.h"

/*
 * 四路直流电机双输入 PWM 驱动封装。
 *
 * 每个电机使用两个定时器通道表达方向和有效占空比。本模块区分三种停止语义：Brake
 * 保持两输入高电平形成主动制动；Coast 保持两输入低电平但不停止 PWM 定时器，可由控制
 * 状态机恢复；EmergencyCoastAll 直接关闭通道、主输出和定时器并把引脚改成低电平 GPIO，
 * 普通 SetPwm 不能恢复。BSP 技术上可由完整 BspMotor_Init 重建 PWM，但运行期安全策略
 * 禁止自行重启，只允许系统复位后恢复。安全层始终可以覆盖普通控制输出。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 安全初始化全部电机 PWM。
 * 在 GPIO 复用和每个通道启动前先执行紧急空转，所有比较值先清零；任一 HAL 启动失败会
 * 再次紧急关闭全部电机并返回 BSP_ERROR。全部成功后进入普通空转，允许后续控制。
 */
BspStatus BspMotor_Init(void);
/*
 * 给指定电机设置有符号 PWM 命令。
 * 正负号决定方向，绝对值先受 ROBOT_CONFIG_PWM_LIMIT 和定时器 ARR 满量程双重限幅；零
 * 命令映射为主动制动。motor 无效返回 BSP_INVALID_ARG，其余情况返回 BSP_OK。
 */
BspStatus BspMotor_SetPwm(BspMotorId motor, int16_t pwm);
/* 普通主动制动：保持 PWM 基础设施运行，控制状态机可在下一周期恢复驱动。 */
void BspMotor_Brake(BspMotorId motor);
/* 对四路电机依次执行普通主动制动。 */
void BspMotor_BrakeAll(void);
/* 普通空转：两输入置低，但不关闭定时器或通道，允许后续 SetPwm 恢复。 */
void BspMotor_Coast(BspMotorId motor);
/* 对四路电机依次执行普通空转。 */
void BspMotor_CoastAll(void);
/*
 * 全局紧急空转。
 * 该寄存器级路径可在 HAL 和 GPIO 常规初始化前后调用，会关闭 TIM1/TIM9/TIM12 的比较
 * 输出与计数器，并把八个方向引脚强制为低电平输出。调用后普通 SetPwm 无法恢复；再次
 * 完整 BspMotor_Init 在技术上可重建 PWM，但应用层急停锁存禁止运行期这样做，系统策略
 * 仍为复位恢复。
 */
void BspMotor_EmergencyCoastAll(void);

#ifdef __cplusplus
}
#endif

#endif
