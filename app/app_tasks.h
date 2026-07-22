/**
 * @file  app_tasks.h
 * @brief 可选任务的注册表与生命周期契约。
 *
 * 每个任务=一个 APP_TASK_DESC (名字 + on_enter/on_tick/on_exit 三钩子)。
 * 新增任务只需在 app_tasks.c 的 TASK_REGISTRY[] 加一行, 菜单/调度/进出清理/
 * 故障处理全部自动接入。钩子契约:
 *   - on_enter: 进入 RUN 一次, 复位本任务全部私有状态; 不驱动执行器、不写模式。
 *   - on_tick : RUN 期每控制周期调用, 非阻塞, 返回 RUNNING/DONE/FAULT。
 *   - on_exit : 正常退出/中止时一次的任务专属收尾(可为 NULL); 通用 Brake 由框架做。
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 任务单周期执行结果。
 */
typedef enum {
    APP_TASK_RUNNING = 0,   /**< 继续运行。 */
    APP_TASK_DONE,          /**< 任务完成，框架刹停并退回菜单。 */
    APP_TASK_FAULT          /**< 可恢复故障，框架进入 FAULT 态。 */
} APP_TASK_STATUS;

/**
 * @brief 可选任务描述符。
 */
typedef struct {
    /** 菜单显示名。 */
    const char *name;
    /** 进入 RUN 时调用一次，复位私有状态。 */
    void (*on_enter)(void);
    /** RUN 期每控制周期调用，dt 为控制周期（s）。 */
    APP_TASK_STATUS (*on_tick)(float dt);
    /** 正常退出/中止时调用一次的专属收尾，可为 NULL。 */
    void (*on_exit)(void);
} APP_TASK_DESC;

/**
 * @brief 注册任务总数。
 */
uint8_t App_TaskCount(void);

/**
 * @brief 获取指定任务的菜单名，越界返回空串。
 */
const char *App_TaskName(uint8_t index);

/**
 * @brief 获取指定任务描述符，越界返回 NULL。
 */
const APP_TASK_DESC *App_TaskAt(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASKS_H */
