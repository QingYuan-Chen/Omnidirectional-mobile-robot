#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

/*
 * 系统级编译期配置集中入口。
 *
 * 本文件只保存已经进入当前实现的时钟、容量、机械参数和安全门槛。修改控制频率或硬件
 * 参数时，必须同步核对 CubeMX 定时器配置、静态断言、主机测试和板上测量结果；不能把
 * 候选值误当成已经验收的最终参数。通信缓冲区和队列容量是静态内存预算，运行时命令
 * 无权扩大。
 */

/* 发送深度不含环形队列内部为判空保留的额外槽位。 */
#define ROBOT_CONFIG_UART_BAUDRATE                 (230400U)
#define ROBOT_CONFIG_UART_LINE_BUFFER_SIZE         (64U)
#define ROBOT_CONFIG_UART_TX_QUEUE_DEPTH           (4U)
#define ROBOT_CONFIG_UART_TX_FRAME_MAX_LENGTH      (416U)
#define ROBOT_CONFIG_MOTOR_COMMAND_QUEUE_DEPTH     (8U)
#define ROBOT_CONFIG_CPU_CLOCK_HZ                  (168000000U)

/*
 * 编码器与底盘几何参数。
 * 1024 PPR 与减速比 30 当前来自参考资料，尚未关闭编码器四倍频含义造成的 30720 轮端
 * 计数歧义；77 mm 是车轮名义直径，有效滚动直径仍待测。0.196 m 轮距和 0.160 m 轴距
 * 是参考安装基线，M4 运动学验收前必须按实车几何复测。上述值当前只能支持软件框架和
 * 后续辨识准备，不能作为已验收标定参数。
 */
#define ROBOT_CONFIG_ENCODER_COUNTS_PER_MOTOR_REV (1024U)
#define ROBOT_CONFIG_MOTOR_REDUCTION_RATIO        (30U)
#define ROBOT_CONFIG_ENCODER_COUNTS_PER_WHEEL_REV \
  (ROBOT_CONFIG_ENCODER_COUNTS_PER_MOTOR_REV * ROBOT_CONFIG_MOTOR_REDUCTION_RATIO)

#define ROBOT_CONFIG_WHEEL_DIAMETER_M             (0.077f)
#define ROBOT_CONFIG_TRACK_WIDTH_M                (0.196f)
#define ROBOT_CONFIG_WHEEL_BASE_M                 (0.160f)

/*
 * 确定性控制时基参数。
 * MCU 与 HCLK 保持 168 MHz；TIM7 输入时钟为 84 MHz，经 84×1000 分频得到候选 1 kHz
 * 控制节拍。中断优先级必须允许调用 FreeRTOS FromISR 接口。控制心跳由固定数量的控制
 * 节拍分频产生，环形快照深度决定任务最多可承受的积压窗口。
 */
#define ROBOT_CONFIG_CONTROL_RATE_HZ              (1000U)
#define ROBOT_CONFIG_CONTROL_TIMER_CLOCK_HZ       (84000000U)
#define ROBOT_CONFIG_CONTROL_TIMER_PRESCALER      (83U)
#define ROBOT_CONFIG_CONTROL_TIMER_AUTORELOAD     (999U)
#define ROBOT_CONFIG_CONTROL_TIMER_IRQ_PRIORITY   (5U)
#define ROBOT_CONFIG_CONTROL_HEARTBEAT_PERIOD_MS  (10U)
#define ROBOT_CONFIG_CONTROL_HEARTBEAT_DIVIDER \
  ((ROBOT_CONFIG_CONTROL_RATE_HZ * ROBOT_CONFIG_CONTROL_HEARTBEAT_PERIOD_MS) / 1000U)
#define ROBOT_CONFIG_CONTROL_TICK_RING_SIZE       (16U)

/*
 * IMU 采样、标定与质量管理参数。
 * 5 ms 是候选处理基础周期，不替代芯片时间戳；IMU 任务实际等待超时为其两倍，即 10 ms。
 * ODR、量程换算必须与 QMI8658A CTRL2/CTRL3
 * 配置一致。20 Hz 输出低通、15 m/s² 与 10 rad/s 突变门限、连续 3 次升级和 8 个稳定
 * 样本恢复均为软件候选值；M1.3 已延期到 M2 单电机阶段，届时需结合静止噪声、运动日志
 * 和故障注入确定。输出低通不得用于 ESKF 输入。
 */
#define ROBOT_CONFIG_IMU_PERIOD_MS                (5U)
#define ROBOT_CONFIG_IMU_ODR_HZ                   (224.2f)
#define ROBOT_CONFIG_IMU_ACCEL_LSB_PER_G          (8192.0f)
#define ROBOT_CONFIG_IMU_GYRO_LSB_PER_DPS         (64.0f)
#define ROBOT_CONFIG_STANDARD_GRAVITY_MPS2        (9.80665f)
#define ROBOT_CONFIG_IMU_CALIBRATION_SAMPLES      (400U)
#define ROBOT_CONFIG_IMU_CALIBRATION_TIMEOUT_MS   (10000U)
#define ROBOT_CONFIG_IMU_STALE_TIMEOUT_MS         (20U)
#define ROBOT_CONFIG_IMU_OUTPUT_FILTER_CUTOFF_HZ  (20.0f)
#define ROBOT_CONFIG_IMU_ACCEL_SPIKE_LIMIT_MPS2   (15.0f)
#define ROBOT_CONFIG_IMU_GYRO_SPIKE_LIMIT_RAD_S   (10.0f)
#define ROBOT_CONFIG_IMU_PERSISTENT_FAULT_THRESHOLD (3U)
#define ROBOT_CONFIG_IMU_RECOVERY_STABLE_SAMPLES  (8U)

/*
 * 非控制任务周期与系统健康门槛。
 * 通信服务和遥测使用毫秒任务周期；安全任务按检查窗收集关键心跳，连续缺失达到
 * HEALTH_MISS_LIMIT 才升级硬故障。默认任务只在 RUNTIME_READY_TIMEOUT 内等待关键任务
 * 心跳完整，并等待 IMU 完成有效数据、时间戳、倾角和 ESKF 收敛门槛。
 */
#define ROBOT_CONFIG_RUNTIME_READY_TIMEOUT_MS      (3000U)
#define ROBOT_CONFIG_COMM_SERVICE_PERIOD_MS       (2U)
#define ROBOT_CONFIG_TELEMETRY_PERIOD_MS          (20U)
#define ROBOT_CONFIG_SAFETY_PERIOD_MS             (100U)
#define ROBOT_CONFIG_MONITOR_PERIOD_MS            (500U)
#define ROBOT_CONFIG_HEALTH_MISS_LIMIT             (3U)

/*
 * G1 阶段 MA 单电机开环试验约束。
 * BSP_PWM_LIMIT 是驱动层绝对上限；MA_OPEN_LOOP_PWM_LIMIT 进一步限制单电机试验风险；
 * RAMP_COUNTS_PER_MS 使斜坡速度不依赖偶发任务延迟；REVERSE_BRAKE_TICKS 是过零后的控制
 * 节拍制动时间。进入正式闭环后需重新评审这些试验参数，不能直接当作控制器限幅。
 */
#define ROBOT_CONFIG_PWM_LIMIT                    (4200)
#define ROBOT_CONFIG_MA_OPEN_LOOP_PWM_LIMIT       (840)
#define ROBOT_CONFIG_MA_PWM_RAMP_COUNTS_PER_MS    (1U)
#define ROBOT_CONFIG_MA_REVERSE_BRAKE_TICKS       (1U)

/*
 * 电池 ADC 换算参数。
 * 当前实现使用整数 NUM/DEN 避免浮点格式化路径，DIVIDER_RATIO 仅保留物理含义。VREF、
 * 分压比和电阻误差尚需万用表对照标定；在完成标定前，电压值只用于遥测观察，不能作为
 * 精确欠压保护阈值。
 */
#define ROBOT_CONFIG_BATTERY_DIVIDER_RATIO        (11.0f)
#define ROBOT_CONFIG_ADC_VREF_MV                  (3300U)
#define ROBOT_CONFIG_ADC_MAX_RAW                  (4095U)
#define ROBOT_CONFIG_BATTERY_DIVIDER_NUM          (11U)
#define ROBOT_CONFIG_BATTERY_DIVIDER_DEN          (1U)
#define ROBOT_CONFIG_ADC_POLL_TIMEOUT_MS           (1U)

#define ROBOT_CONFIG_CMD_TIMEOUT_MS               (500U)

/*
 * 预留给后续上层状态汇总的通用状态码。
 * 当前 G1/M1 逻辑以 AppRuntimeSnapshot 的细分健康字段为准，本枚举尚不承担安全仲裁。
 */
typedef enum {
  ROBOT_STATUS_OK = 0,
  ROBOT_STATUS_CMD_TIMEOUT = 1,
  ROBOT_STATUS_LOW_BATTERY = 2,
  ROBOT_STATUS_FAULT = 3
} RobotStatus;

#endif
