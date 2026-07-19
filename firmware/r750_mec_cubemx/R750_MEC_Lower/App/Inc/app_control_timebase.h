#ifndef APP_CONTROL_TIMEBASE_H
#define APP_CONTROL_TIMEBASE_H

#include "app_encoder_period_accumulator.h"
#include "bsp_types.h"

#include <stdint.h>

/*
 * 确定性控制节拍适配层。
 *
 * TIM7 以候选 1 kHz 频率产生更新中断。向量入口尽早读取 DWT 周期计数，HAL 回调随后只
 * 完成定长编码器快照、漏中断统计、环形缓冲发布和直接任务通知；测速、状态机和控制律
 * 全部留在唯一登记的控制任务中。启动时会核对主频、APB1 定时器时钟、分频参数和中断
 * 优先级，避免 CubeMX 与 robot_config.h 配置漂移后仍静默运行。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 控制任务一次唤醒取得的完整节拍。
 * irq_timestamp_cycles、irq_period_cycles、task_wake_cycles 的单位均为 CPU 周期；
 * notification_count 大于 1 表示多个通知在任务得到调度前发生了合并；编码器值与
 * tick_sequence 对应的中断时刻绑定，不是读取函数调用时的“最新值”。
 */
typedef struct {
  uint32_t tick_sequence;
  uint32_t irq_timestamp_cycles;
  uint32_t irq_period_cycles;
  uint32_t timer_irq_missed_period_count;
  uint32_t task_wake_cycles;
  uint32_t notification_count;
  AppEncoderPeriodSnapshot encoder_period_ma;
  uint16_t encoder_raw[BSP_MOTOR_COUNT];
} AppControlTick;

/*
 * 由控制任务调用，校验配置、启用 DWT、清除旧通知并启动 TIM7。
 * 成功后当前任务成为唯一等待者；重复启动返回 BSP_BUSY，配置或硬件启动异常返回
 * BSP_ERROR。函数成功后必须持续调用 Wait 消费节拍。
 */
BspStatus AppControlTimebase_Start(void);
/*
 * 停止 TIM7 更新中断并撤销任务所有权。未启动时也返回 BSP_OK；HAL 停止失败返回
 * BSP_ERROR。该接口用于故障收敛，不负责恢复已经紧急关闭的电机 PWM。
 */
BspStatus AppControlTimebase_Stop(void);
/*
 * 阻塞等待一个或多个 TIM7 通知，并复制与累计通知数对应的快照。
 * 仅 Start 时登记的控制任务可调用。成功返回 BSP_OK；参数、所有权或启动状态不合法
 * 返回 BSP_INVALID_ARG；槽位覆盖或通知与快照不一致返回 BSP_ERROR。
 */
BspStatus AppControlTimebase_Wait(AppControlTick *tick);
/* 返回当前 32 位 DWT 周期计数，允许自然回绕，用于周期执行时间和截止期计算。 */
uint32_t AppControlTimebase_GetCycleCount(void);

/*
 * G3_SPEED 诊断专用 MA TIM2 CC1 边沿事件。
 * Start 仅在 TIM7/DWT 已运行时启用，运行时设置 IRQ 优先级并只打开 CC1；Stop 关闭并清
 * pending。该路径不修改 CubeMX 的常驻 NVIC 配置，普通 G2 电机采集期间保持完全关闭。
 */
BspStatus AppControlTimebase_StartMaPeriodCapture(void);
BspStatus AppControlTimebase_StopMaPeriodCapture(void);
bool AppControlTimebase_GetMaPeriodStats(AppEncoderPeriodStats *stats);
void AppControlTimebase_OnMaEncoderIrqFromIsr(void);
/*
 * TIM7 向量入口钩子。必须在 HAL_TIM_IRQHandler 之前调用，以减小 HAL 分派开销对中断
 * 周期和任务唤醒延迟测量的污染；未启动时不产生状态变化。
 */
void AppControlTimebase_OnTimerIrqEntryFromIsr(void);
/*
 * TIM7 更新完成回调钩子。只允许在中断上下文调用；执行路径固定且不得加入控制计算、
 * 阻塞操作或日志格式化。函数完成快照发布后用 FromISR 通知控制任务并按需触发调度。
 */
void AppControlTimebase_OnTimerElapsedFromIsr(void);

#ifdef __cplusplus
}
#endif

#endif
