/**
 * @file mission_fsm.h
 * @brief 主任务状态机 - 协调底盘移动与云台动作
 * @date 2026-01-29
 */

#ifndef MISSION_FSM_H
#define MISSION_FSM_H

#include "struct_typedef.h"
#include <stdint.h>
#include <stdbool.h>

/*-------------------------- 宏定义 --------------------------*/

#define FACING_FORWARD_ANGLE 0.0f   // 面向前方的航向角（单位：rad），根据实际调整
#define FACING_LEFT_ANGLE 1.5708f  // 面向左侧的航向角（单位：rad），根据实际调整
#define FACING_BACKWARD_ANGLE 3.1415f // 面向后方的航向角（单位：rad），根据实际调整
#define FACING_RIGHT_ANGLE -1.5708f  // 面向右侧的航向角（单位：rad），根据实际调整

/*-------------------------- 主状态机状态枚举 --------------------------*/

typedef enum {
    MISSION_STATE_IDLE = 0,         // 空闲，等待启动
    MISSION_STATE_GOTO_POINT,       // 底盘移动到目标点
    MISSION_STATE_GIMBAL_ACTION,    // 云台执行动作（丝杆+伸缩+夹爪）
    MISSION_STATE_WAIT_COMPLETE,    // 等待动作完成
    MISSION_STATE_NEXT_STEP,        // 切换到下一步骤
    MISSION_STATE_DONE,             // 任务完成
    MISSION_STATE_ERROR,            // 错误状态
} mission_state_e;

// 错误原因（调试用）
typedef enum {
    MISSION_ERR_NONE = 0,
    MISSION_ERR_INVALID_PARAM,
    MISSION_ERR_GOTO_TIMEOUT,
    MISSION_ERR_GOTO_ODOM_STALE,
    MISSION_ERR_GOTO_YAW_STALE,
    MISSION_ERR_GIMBAL_TIMEOUT,
    MISSION_ERR_INVALID_STEP,
} mission_error_e;

/*-------------------------- 云台动作子状态枚举 --------------------------*/

typedef enum {
    GIMBAL_SUB_IDLE = 0,            // 空闲
    GIMBAL_SUB_BALLSCREW_MOVE,      // 丝杆移动中
    GIMBAL_SUB_TELESCOPIC_MOVE,     // 伸缩臂移动中
    GIMBAL_SUB_GRIPPER_ACTION,      // 夹爪动作中
    GIMBAL_SUB_WAIT_DELAY,          // 延时等待
    GIMBAL_SUB_COMPLETE,            // 动作完成
} gimbal_sub_state_e;

/*-------------------------- 任务步骤定义 --------------------------*/

// 单步任务类型
typedef enum {
    STEP_TYPE_GOTO,                 // 移动到位置
    STEP_TYPE_VISUAL_ALIGN, // [新增] 视觉对准
    STEP_TYPE_BALLSCREW,            // 丝杆动作
    STEP_TYPE_TELESCOPIC,           // 伸缩臂动作
    STEP_TYPE_GRIPPER,              // 夹爪动作
    STEP_TYPE_DELAY,                // 延时
    STEP_TYPE_COMPOUND,             // 复合动作（云台多轴联动）
} step_type_e;

// 单步任务参数
typedef struct {
    step_type_e type;               // 步骤类型
    
    // 底盘移动参数
    fp32 target_x;                  // 目标X(m)
    fp32 target_y;                  // 目标Y(m)
    fp32 target_yaw;                // 目标航向(rad)

    uint8_t visual_target_id;
    
    // 云台动作参数
    uint8_t ballscrew_target;       // 丝杆目标位置枚举
    uint8_t telescopic_target;      // 伸缩臂目标位置枚举
    uint8_t gripper_action;         // 夹爪动作：0=张开, 1=闭合
    
    // 延时参数
    uint32_t delay_ms;              // 延时时间(ms)
    
} mission_step_t;

/*-------------------------- 任务控制结构体 --------------------------*/

typedef struct {
    mission_state_e     state;              // 主状态
    gimbal_sub_state_e  gimbal_sub_state;   // 云台子状态
    
    const mission_step_t *step_list;        // 步骤列表指针
    uint8_t             step_count;         // 步骤总数
    uint8_t             current_step;       // 当前步骤索引
    
    uint32_t            timer_ms;           // 通用计时器
    uint32_t            timeout_ms;         // 超时阈值
    
    bool                is_running;         // 任务运行标志
    bool                error_flag;         // 错误标志
    mission_error_e     error_code;         // 错误码（调试）
    
} mission_fsm_t;

/*-------------------------- 外部接口 --------------------------*/

/**
 * @brief 任务状态机初始化
 */
void Mission_FSM_Init(void);

/**
 * @brief 任务状态机周期运行（在10ms中断中调用）
 */
void Mission_FSM_Run(void);

/**
 * @brief 启动任务序列
 * @param steps 步骤数组
 * @param count 步骤数量
 */
void Mission_Start(const mission_step_t *steps, uint8_t count);

/**
 * @brief 停止当前任务
 */
void Mission_Stop(void);

/**
 * @brief 查询任务是否完成
 * @return true=完成或空闲, false=进行中
 */
bool Mission_Is_Done(void);

/**
 * @brief 获取当前状态（调试用）
 */
mission_state_e Mission_Get_State(void);

/**
 * @brief 获取任务控制结构体指针（调试用）
 */
const mission_fsm_t* Mission_Get_FSM(void);

#endif /* MISSION_FSM_H */
