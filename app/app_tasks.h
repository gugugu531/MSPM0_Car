/**
 * @file  app_tasks.h
 * @brief 任务生命周期类型与业务任务描述符。
 *
 * 每个任务=一个 APP_TASK_DESC（名字 + on_enter/on_tick/on_exit 三钩子）。钩子契约:
 *   - on_enter: 进入 RUN 一次, 复位本任务全部私有状态; 不驱动执行器、不写模式。
 *   - on_tick : RUN 期每控制周期调用, 非阻塞, 返回 RUNNING/DONE/FAULT。
 *   - on_exit : 正常退出/中止时一次的任务专属收尾(可为 NULL); 通用 Brake 由框架做。
 * 任务通过菜单树(app_menu_def)被引用; 自检任务见 app_checks.h。
 */
#ifndef APP_TASKS_H
#define APP_TASKS_H

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
 * @brief 任务描述符。
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

/** 业务任务：5s 倒计时自检入口（用作框架自测）。 */
extern const APP_TASK_DESC APP_TASK_TIMER;

#ifdef __cplusplus
}
#endif

#endif /* APP_TASKS_H */
