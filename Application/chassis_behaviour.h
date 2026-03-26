/**
 * @file chassis_behaviour.h
 * @brief 底盘行为/位置环外环控制模块
 * ----------------------------------------------------------------------------
 * @version 1.0.0.0
 * @date 2026-01-27
 * @remark 位置环外环，输出世界系速度指令给 chassis_task
 */

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H

/*-----------------------------------头文件引用-----------------------------------*/

#include "struct_typedef.h"
#include "pid.h"

/*-----------------------------------宏定义-----------------------------------*/

// 位置环控制周期（与底盘任务保持一致）
#define BEHAVIOUR_CONTROL_TIME      0.1f
#define BEHAVIOUR_CONTROL_TIME_MS   100

// 位置环 PID 参数（X/Y 轴可分开调参，初始先用同一套）
#define CHASSIS_POSITION_PID_KP         0.9f
#define CHASSIS_POSITION_PID_KI         0.0f
#define CHASSIS_POSITION_PID_KD         0.05f
#define CHASSIS_POSITION_PID_MAX_OUT    0.60f    // 输出限幅：最大速度 m/s
#define CHASSIS_POSITION_PID_MAX_IOUT   0.1f    // 积分限幅

// 【新增】非线性开根号算法专用的比例系数 Kp
// 注意：开根号后的误差值变小了，所以这个 Kp 通常要比上面线性的 Kp 调得大一些
#define CHASSIS_POSITION_ROOT_KP        0.6f
// 非线性开根号算法专用的微分系数 Kd（与线性PID的KD独立）
#define CHASSIS_POSITION_ROOT_KD        0.2f

// 位置误差死区（单位：米），小于此值认为到达
#define CHASSIS_POSITION_DEADBAND       0.005f

// 到位判定阈值（单位：米）
#define CHASSIS_POSITION_ARRIVE_THRESHOLD  0.005f

// 目标位置约束（单位：米），对称限幅范围为 [-LIMIT, LIMIT]
#define CHASSIS_POSITION_X_LIMIT_M         2.3f
#define CHASSIS_POSITION_Y_LIMIT_M         2.3f

/*-----------------------------------数据结构定义-----------------------------------*/

// 定义底盘的控制模式
typedef enum {
    CHASSIS_MODE_MANUAL = 0,    // 遥控器手动模式
    CHASSIS_MODE_AUTO_NAV,      // 自动导航模式（归 trajectory_planner 管）
    CHASSIS_MODE_VISUAL_ALIGN,  // 视觉微调模式（归视觉算法管，今天的主角）
} chassis_behaviour_mode_e;

// 位置环控制模式
typedef enum
{
    BEHAVIOUR_MODE_DISABLE = 0,     // 位置环关闭，不输出
    BEHAVIOUR_MODE_POSITION,        // 位置环使能，跟踪目标位置
} behaviour_mode_e;

// 底盘里程计/位置环结构体
typedef struct
{
    behaviour_mode_e mode;                      // 当前控制模式

    const volatile fp32 *position_x_ptr;        // 指向CAN接收的X方向位移数据（单位：米）
    const volatile fp32 *position_y_ptr;        // 指向CAN接收的Y方向位移数据（单位：米）

    pid_type_def position_x_pid;                // X轴位置PID（独立实例）
    pid_type_def position_y_pid;                // Y轴位置PID（独立实例）

    fp32 position_x;                            // X方向当前位置（米）
    fp32 position_y;                            // Y方向当前位置（米）

    fp32 position_x_set;                        // X方向目标位置（米）
    fp32 position_y_set;                        // Y方向目标位置（米）

    fp32 vx_out;                                // X方向速度输出（m/s，世界系）
    fp32 vy_out;                                // Y方向速度输出（m/s，世界系）

    fp32 yaw_set;                               // 航向角目标（rad）

    uint8_t arrive_flag;                        // 到位标志：1=已到达目标位置

} chassis_odometry_t;

/*-----------------------------------外部函数声明-----------------------------------*/

/**
 * @brief  全局位置环模块初始化（在 main 初始化阶段调用）
 * @retval none
 */
extern void global_behaviour_init(void);

/**
 * @brief  位置环周期任务（在主循环中周期调用，与底盘任务同频）
 * @retval none
 */
extern void behaviour_task_cycle(void);

/**
 * @brief  设置目标位置（世界坐标系，单位：米）
 * @param  x_m  X方向目标位置
 * @param  y_m  Y方向目标位置
 * @retval none
 */
extern void behaviour_set_position(fp32 x_m, fp32 y_m);

/**
 * @brief  设置目标位置和航向角（世界坐标系）
 * @param  x_m      X方向目标位置（米）
 * @param  y_m      Y方向目标位置（米）
 * @param  yaw_rad  航向角目标（rad）
 * @retval none
 */
extern void behaviour_set_position_yaw(fp32 x_m, fp32 y_m, fp32 yaw_rad);

/**
 * @brief  使能位置环控制
 * @param  latch_current_target  1=使能时将目标锁存为当前位置；0=保留已有目标
 * @retval none
 */
extern void behaviour_enable(uint8_t latch_current_target);

/**
 * @brief  关闭位置环控制（停止输出速度指令，并释放底盘航向保持/世界系模式）
 * @retval none
 */
extern void behaviour_disable(void);

/**
 * @brief  查询是否到达目标位置
 * @retval 1=到达，0=未到达
 */
extern uint8_t behaviour_is_arrived(void);

/**
 * @brief  获取位置环结构体指针（调试用）
 * @retval 结构体指针
 */
extern const chassis_odometry_t *get_behaviour_data(void);

#endif /* CHASSIS_BEHAVIOUR_H */