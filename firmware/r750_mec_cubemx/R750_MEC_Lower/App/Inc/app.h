#ifndef APP_H
#define APP_H

/*
 * 应用启动入口。
 *
 * 本模块承接 CubeMX 创建的默认任务，但不承担长期业务。它依次完成板级外设初始化、
 * IMU 静止标定、运行任务创建和系统就绪检查；任何一步失败都会进入不可恢复的安全
 * 停机循环。只有 IMU 数据质量与估计器状态同时满足启动门槛后，才会鸣笛确认并释放
 * 默认任务栈。因此，新增启动步骤时必须保持“电机先安全、依赖后使用、失败不带病运行”
 * 的顺序。
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CubeMX 默认任务入口。
 * 参数当前未使用；函数成功完成启动后调用 osThreadExit，不会返回。初始化失败时会保持
 * 电机紧急空转并用 LED2 周期闪烁指示，也不会返回。
 */
void App_DefaultTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
