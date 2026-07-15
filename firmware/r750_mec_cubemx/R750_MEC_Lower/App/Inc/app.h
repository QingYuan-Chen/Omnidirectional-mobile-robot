#ifndef APP_H
#define APP_H

/* 应用入口负责按安全顺序初始化板级资源、校准 IMU、创建任务并等待系统就绪。 */

#ifdef __cplusplus
extern "C" {
#endif

/* CubeMX 默认任务入口；启动完成后主动退出并归还任务栈。 */
void App_DefaultTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
