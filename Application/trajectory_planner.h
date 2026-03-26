/**
 * @file trajectory_planner.h
 * @brief 底盘位置轨迹规划封装模块（对外统一接口）。
 *
 * 设计目的：
 * 1) 将“位置移动”相关接口统一放在本模块；
 * 2) 上层任务不再直接调用 behaviour_*，而是调用 trajectory_planner_*；
 * 3) 内部通过直线梯形速度规划 + 预瞄前馈，减小长距离移动超调。
 */

#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "struct_typedef.h"

/*-----------------------------------宏定义-----------------------------------*/

// 轨迹规划更新周期（与 behaviour 外环保持一致）
#define TRAJECTORY_PLANNER_CONTROL_TIME      0.1f
#define TRAJECTORY_PLANNER_CONTROL_TIME_MS   100

typedef enum
{
    TRAJ_PHASE_IDLE = 0,  // 空闲（未运行）
    TRAJ_PHASE_ACCEL,     // 加速段
    TRAJ_PHASE_CRUISE,    // 匀速段
    TRAJ_PHASE_DECEL,     // 减速段
    TRAJ_PHASE_SETTLE,    // 末端收敛段（交给 behaviour 到位）
    TRAJ_PHASE_DONE,      // 完成
} trajectory_phase_e;

typedef struct
{
    uint8_t running;          // 1=规划器正在运行
    uint8_t finished;         // 1=本次轨迹已完成
    trajectory_phase_e phase; // 当前阶段

    fp32 start_x;             // 轨迹起点 X（m）
    fp32 start_y;             // 轨迹起点 Y（m）
    fp32 target_x;            // 轨迹终点 X（m）
    fp32 target_y;            // 轨迹终点 Y（m）
    fp32 target_yaw;          // 终点航向角（rad）

    fp32 ref_x;               // 当前下发的参考点 X（m）
    fp32 ref_y;               // 当前下发的参考点 Y（m）
    fp32 ref_s;               // 沿轨迹弧长参考（m）
    fp32 ref_v;               // 沿轨迹速度参考（m/s）

    fp32 total_dist;          // 总路径长度（m）
    fp32 elapsed_s;           // 本次运行已用时间（s）
} trajectory_planner_state_t;

/**
 * @brief 初始化轨迹规划器（使用默认速度/加速度/前馈参数）。
 */
//extern void trajectory_planner_init(void);

/**
 * @brief 全局初始化接口（工程风格封装）
 * @note 建议在 main 初始化阶段调用。
 */
extern void global_trajectory_planner_init(void);

/**
 * @brief 周期任务接口（工程风格封装）
 * @note 该函数内部按 100ms 节拍调用 trajectory_planner_update()。
 */
extern void trajectory_planner_task_cycle(void);

/**
 * @brief 设置直线轨迹的速度与加速度约束。
 * @param max_vel_mps 规划最大线速度（m/s），必须 > 0
 * @param max_acc_mps2 规划最大线加速度（m/s^2），必须 > 0
 * @note 参数越大越快，但过大可能增加超调与轮滑风险。
 */
extern void trajectory_planner_set_limits(fp32 max_vel_mps, fp32 max_acc_mps2);

/**
 * @brief 设置前馈预瞄时间。
 * @param preview_time_s 预瞄时间（s），内部使用 ref_v * preview_time_s 作为前瞻距离
 * @note 越大响应越积极，但过大可能导致末端提前量过高。
 */
extern void trajectory_planner_set_feedforward(fp32 preview_time_s);

/**
 * @brief 设置到位保持次数（抗抖）。
 * @param hold_cycles 连续 N 次更新都到位才判定完成
 */
extern void trajectory_planner_set_arrive_hold(uint8_t hold_cycles);

/**
 * @brief 启动一次“直线”规划移动。
 * @param target_x_m 目标点 X（世界系，m）
 * @param target_y_m 目标点 Y（世界系，m）
 * @param target_yaw_rad 目标航向角（世界系，rad）
 * @param latch_start_from_current 1=从当前里程计位置锁存起点，0=沿用上次起点
 * @note 推荐常用 1，保证每次都从实时位置重新规划。
 */
extern void trajectory_planner_start_line(fp32 target_x_m,
                                          fp32 target_y_m,
                                          fp32 target_yaw_rad,
                                          uint8_t latch_start_from_current);

/**
 * @brief 停止规划器并关闭 behaviour 输出。
 */
extern void trajectory_planner_stop(void);

/**
 * @brief 周期更新函数。
 * @note 需在主循环持续调用，建议调用位置与 behaviour_task_cycle 同一层级。
 */
extern void trajectory_planner_update(void);

/**
 * @brief 查询规划器是否正在运行。
 */
extern uint8_t trajectory_planner_is_running(void);

/**
 * @brief 查询本次规划是否完成。
 */
extern uint8_t trajectory_planner_is_finished(void);

/**
 * @brief 获取内部状态（用于上位机显示/串口调试）。
 */
extern const trajectory_planner_state_t *trajectory_planner_get_state(void);

#endif /* TRAJECTORY_PLANNER_H */
