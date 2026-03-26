/**
 * @file mission_fsm.c
 * @brief 主任务状态机实现 - 协调底盘移动与云台动作
 * @date 2026-01-29
 */

#include "mission_fsm.h"
#include "main.h"
#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "zdt_motor.h"
#include "bsp_servo_pwm.h"  // 夹爪舵机
#include <string.h>
#include <math.h>

/*-----------------------------------宏定义-----------------------------------*/

// 超时配置
#define GOTO_TIMEOUT_MS         15000   // 底盘移动超时 15s
#define GIMBAL_TIMEOUT_MS       5000    // 云台动作超时 5s
#define DEFAULT_DELAY_MS        500     // 默认动作间隔

// 防跳步配置：避免上一段残留到位标志导致新 GOTO 立即完成
#define GOTO_MIN_EFFECTIVE_MS   150     // 新 GOTO 至少运行 150ms 后才允许判定到位
#define GOTO_ARRIVE_CONFIRM_CNT 3       // 连续 3 次(10ms周期)检测到位才确认完成
#define GOTO_ODOM_STALE_MS      8000    // 里程计长时间无变化，判定为异常
#define GOTO_PROGRESS_EPS_M     0.0001f // 认为“有进展”的最小位移
#define GOTO_STALE_NEAR_ARRIVE_M 0.030f // 触发停滞时，若已足够接近则直接判完成
#define GOTO_DIRECT_ARRIVE_M    0.020f  // 直接位置到位阈值（2cm，避免低速死区导致近目标卡停）
#define GOTO_YAW_ARRIVE_RAD     0.01745f // 航向到位阈值（约1度）
#define GOTO_TURN_ONLY_DIST_M   0.030f  // 判定“原地转向步”的位置误差阈值
#define GOTO_SAME_TARGET_EPS_M  0.010f  // 与上一步同点判定阈值
#define GOTO_TURN_POS_RELAX_M   0.120f  // 原地转向步允许的位置误差阈值
#define GOTO_TURN_MIN_MS        300     // 原地转向步最短执行时间，避免只发一次控制
#define GOTO_YAW_PROGRESS_EPS   0.0087f // 航向有进展阈值（约0.5度）
#define GOTO_YAW_STALE_MS       3000    // 航向长时间无进展判定异常

// 接近目标后的微调阶段：允许额外时间，避免“快到位时超调微调”被误判超时
#define GOTO_SETTLE_ZONE_M      0.080f  // 进入微调区的位置误差阈值
#define GOTO_SETTLE_YAW_RAD     0.200f  // 进入微调区的航向误差阈值（约11.5度）
#define GOTO_SETTLE_GRACE_MS    3000    // 微调区额外超时宽限

// 夹爪舵机角度（根据实际调整，若已在别处定义则注释掉）
#ifndef GRIPPER_OPEN_ANGLE
#define GRIPPER_OPEN_ANGLE      0
#endif
#ifndef GRIPPER_CLOSE_ANGLE
#define GRIPPER_CLOSE_ANGLE     90
#endif

/*-----------------------------------变量声明-----------------------------------*/

static mission_fsm_t mission_fsm;
static uint8_t goto_arrive_confirm_cnt = 0;
static fp32 goto_last_x = 0.0f;
static fp32 goto_last_y = 0.0f;
static fp32 goto_last_yaw = 0.0f;
static uint32_t goto_odom_stale_ms = 0;
static uint32_t goto_yaw_stale_ms = 0;
static uint8_t goto_turn_only_step = 0;
static uint32_t goto_last_tick_ms = 0;

/*-----------------------------------内部函数声明-----------------------------------*/

static void FSM_StateIdle(void);
static void FSM_StateGotoPoint(void);
static void FSM_StateGimbalAction(void);
static void FSM_StateWaitComplete(void);
static void FSM_StateNextStep(void);
static void FSM_StateDone(void);
static void FSM_StateError(void);

static void Mission_Reset_GotoRuntime(void);
static uint32_t Mission_GotoElapsedMs(void);
static void Mission_Enter_GotoStep(const mission_step_t *step);
static void Execute_CurrentStep(void);
static bool Check_StepComplete(void);

/*-----------------------------------函数实现-----------------------------------*/

void Mission_FSM_Init(void)
{
    memset(&mission_fsm, 0, sizeof(mission_fsm));
    mission_fsm.state = MISSION_STATE_IDLE;
    mission_fsm.gimbal_sub_state = GIMBAL_SUB_IDLE;
    mission_fsm.is_running = false;
    mission_fsm.error_code = MISSION_ERR_NONE;
}

void Mission_FSM_Run(void)
{
    // 主状态机调度
    switch (mission_fsm.state)
    {
        case MISSION_STATE_IDLE:
            FSM_StateIdle();
            break;
        case MISSION_STATE_GOTO_POINT:
            FSM_StateGotoPoint();
            break;
        case MISSION_STATE_GIMBAL_ACTION:
            FSM_StateGimbalAction();
            break;
        case MISSION_STATE_WAIT_COMPLETE:
            FSM_StateWaitComplete();
            break;
        case MISSION_STATE_NEXT_STEP:
            FSM_StateNextStep();
            break;
        case MISSION_STATE_DONE:
            FSM_StateDone();
            break;
        case MISSION_STATE_ERROR:
            FSM_StateError();
            break;
        default:
            mission_fsm.state = MISSION_STATE_ERROR;
            break;
    }
}

void Mission_Start(const mission_step_t *steps, uint8_t count)
{
    if (steps == NULL || count == 0)
    {
        mission_fsm.error_flag = true;
        mission_fsm.error_code = MISSION_ERR_INVALID_PARAM;
        mission_fsm.state = MISSION_STATE_ERROR;
        return;
    }
    
    mission_fsm.step_list = steps;
    mission_fsm.step_count = count;
    mission_fsm.current_step = 0;
    mission_fsm.is_running = true;
    mission_fsm.error_flag = false;
    mission_fsm.error_code = MISSION_ERR_NONE;
    mission_fsm.timer_ms = 0;
    Mission_Reset_GotoRuntime();
    
    // 根据第一步类型决定初始状态
    Execute_CurrentStep();
}

void Mission_Stop(void)
{
    // 停止所有执行器
    behaviour_disable();
    ZDT_Motor_Stop_All();
    
    mission_fsm.state = MISSION_STATE_IDLE;
    mission_fsm.gimbal_sub_state = GIMBAL_SUB_IDLE;
    mission_fsm.is_running = false;
    mission_fsm.error_flag = false;
    mission_fsm.error_code = MISSION_ERR_NONE;
    Mission_Reset_GotoRuntime();
}

bool Mission_Is_Done(void)
{
    return (mission_fsm.state == MISSION_STATE_DONE || 
            mission_fsm.state == MISSION_STATE_IDLE);
}

mission_state_e Mission_Get_State(void)
{
    return mission_fsm.state;
}

const mission_fsm_t* Mission_Get_FSM(void)
{
    return &mission_fsm;
}

static void Mission_Reset_GotoRuntime(void)
{
    goto_arrive_confirm_cnt = 0;
    goto_odom_stale_ms = 0;
    goto_yaw_stale_ms = 0;
    goto_turn_only_step = 0;
    goto_last_tick_ms = 0;
}

static uint32_t Mission_GotoElapsedMs(void)
{
    uint32_t now_tick = HAL_GetTick();
    uint32_t dt_ms = 10U;

    if (goto_last_tick_ms != 0U)
    {
        dt_ms = now_tick - goto_last_tick_ms;
        if (dt_ms == 0U)
        {
            dt_ms = 1U;
        }
        else if (dt_ms > 100U)
        {
            dt_ms = 100U;
        }
    }

    goto_last_tick_ms = now_tick;
    return dt_ms;
}

static void Mission_Enter_GotoStep(const mission_step_t *step)
{
    const chassis_odometry_t *odom = get_behaviour_data();
    const chassis_move_t *chassis = get_chassis_move_data();
    const mission_step_t *prev_step = NULL;

    behaviour_enable(0);
    behaviour_set_position_yaw(step->target_x, step->target_y, step->target_yaw);
    mission_fsm.state = MISSION_STATE_GOTO_POINT;
    mission_fsm.timeout_ms = GOTO_TIMEOUT_MS;
    mission_fsm.timer_ms = 0;
    Mission_Reset_GotoRuntime();
    goto_last_tick_ms = HAL_GetTick();

    if (odom != NULL)
    {
        goto_last_x = odom->position_x;
        goto_last_y = odom->position_y;

        {
            fp32 dx0 = step->target_x - odom->position_x;
            fp32 dy0 = step->target_y - odom->position_y;
            fp32 d0 = sqrtf(dx0 * dx0 + dy0 * dy0);
            if (d0 <= GOTO_TURN_ONLY_DIST_M)
            {
                goto_turn_only_step = 1;
            }
        }
    }
    else
    {
        goto_last_x = 0.0f;
        goto_last_y = 0.0f;
    }

    if (mission_fsm.current_step > 0)
    {
        prev_step = &mission_fsm.step_list[mission_fsm.current_step - 1];
        if ((prev_step->type == STEP_TYPE_GOTO || prev_step->type == STEP_TYPE_VISUAL_ALIGN) &&
            fabsf(step->target_x - prev_step->target_x) <= GOTO_SAME_TARGET_EPS_M &&
            fabsf(step->target_y - prev_step->target_y) <= GOTO_SAME_TARGET_EPS_M)
        {
            goto_turn_only_step = 1;
        }
    }

    if (chassis != NULL)
    {
        goto_last_yaw = chassis->chassis_yaw;
    }
    else
    {
        goto_last_yaw = 0.0f;
    }
}

/*-----------------------------------内部函数实现-----------------------------------*/

/**
 * @brief 执行当前步骤
 */
static void Execute_CurrentStep(void)
{
    if (mission_fsm.current_step >= mission_fsm.step_count)
    {
        mission_fsm.state = MISSION_STATE_DONE;
        return;
    }
    
    const mission_step_t *step = &mission_fsm.step_list[mission_fsm.current_step];
    
    switch (step->type)
    {
        case STEP_TYPE_GOTO:
            Mission_Enter_GotoStep(step);
            break;

        case STEP_TYPE_VISUAL_ALIGN:
            // 视觉对准当前仍复用 GOTO 管线
            Mission_Enter_GotoStep(step);
            break;
            
        case STEP_TYPE_BALLSCREW:
            Ballscrew_MoveToTarget((ballscrew_target_pos_e)step->ballscrew_target);
            mission_fsm.state = MISSION_STATE_GIMBAL_ACTION;
            mission_fsm.gimbal_sub_state = GIMBAL_SUB_BALLSCREW_MOVE;
            mission_fsm.timeout_ms = GIMBAL_TIMEOUT_MS;
            mission_fsm.timer_ms = 0;
            break;
            
        case STEP_TYPE_TELESCOPIC:
            Telescopic_MoveToTarget((telescopic_target_pos_e)step->telescopic_target);
            mission_fsm.state = MISSION_STATE_GIMBAL_ACTION;
            mission_fsm.gimbal_sub_state = GIMBAL_SUB_TELESCOPIC_MOVE;
            mission_fsm.timeout_ms = GIMBAL_TIMEOUT_MS;
            mission_fsm.timer_ms = 0;
            break;
            
        case STEP_TYPE_GRIPPER:
            // 夹爪舵机控制（根据你的实际接口修改）
            if (step->gripper_action == 0)
            {
                // bsp_servo_set_angle(GRIPPER_OPEN_ANGLE);
            }
            else
            {
                // bsp_servo_set_angle(GRIPPER_CLOSE_ANGLE);
            }
            mission_fsm.state = MISSION_STATE_GIMBAL_ACTION;
            mission_fsm.gimbal_sub_state = GIMBAL_SUB_GRIPPER_ACTION;
            mission_fsm.timeout_ms = 500;  // 夹爪动作时间
            mission_fsm.timer_ms = 0;
            break;
            
        case STEP_TYPE_DELAY:
            mission_fsm.state = MISSION_STATE_WAIT_COMPLETE;
            mission_fsm.timeout_ms = step->delay_ms;
            mission_fsm.timer_ms = 0;
            break;
            
        case STEP_TYPE_COMPOUND:
            // 复合动作：丝杆+伸缩臂同时运动
            Ballscrew_MoveToTarget((ballscrew_target_pos_e)step->ballscrew_target);
            Telescopic_MoveToTarget((telescopic_target_pos_e)step->telescopic_target);
            mission_fsm.state = MISSION_STATE_GIMBAL_ACTION;
            mission_fsm.gimbal_sub_state = GIMBAL_SUB_BALLSCREW_MOVE;  // 等最慢的完成
            mission_fsm.timeout_ms = GIMBAL_TIMEOUT_MS;
            mission_fsm.timer_ms = 0;
            break;
            
        default:
            mission_fsm.error_flag = true;
            mission_fsm.error_code = MISSION_ERR_INVALID_STEP;
            mission_fsm.state = MISSION_STATE_ERROR;
            break;
    }
}

/**
 * @brief 检查当前步骤是否完成
 */
static bool Check_StepComplete(void)
{
    const mission_step_t *step = &mission_fsm.step_list[mission_fsm.current_step];
    
    switch (step->type)
    {
        case STEP_TYPE_GOTO:
            return behaviour_is_arrived();
            
        case STEP_TYPE_BALLSCREW:
            return (Ballscrew_GetMotor()->state == MOTOR_STATE_IDLE);
            
        case STEP_TYPE_TELESCOPIC:
            return (Telescopic_GetMotor()->state == MOTOR_STATE_IDLE);
            
        case STEP_TYPE_GRIPPER:
        case STEP_TYPE_DELAY:
            return (mission_fsm.timer_ms >= mission_fsm.timeout_ms);
            
        case STEP_TYPE_COMPOUND:
            return (Ballscrew_GetMotor()->state == MOTOR_STATE_IDLE &&
                    Telescopic_GetMotor()->state == MOTOR_STATE_IDLE);
            
        default:
            return true;
    }
}

/*-----------------------------------主状态机各状态处理-----------------------------------*/

static void FSM_StateIdle(void)
{
    // 空闲状态，等待 Mission_Start() 触发
}

static void FSM_StateGotoPoint(void)
{
    bool pos_arrived = false;
    bool yaw_arrived = false;
    bool arrived = false;
    bool in_settle_zone = false;
    const chassis_odometry_t *odom = get_behaviour_data();
    const chassis_move_t *chassis = get_chassis_move_data();
    const mission_step_t *step = &mission_fsm.step_list[mission_fsm.current_step];
    fp32 dist = 0.0f;
    fp32 yaw_err_abs = 0.0f;
    uint32_t timeout_limit = mission_fsm.timeout_ms;
    uint32_t dt_ms = Mission_GotoElapsedMs();

    // 使用实际经过时间推进状态机计时，减少节拍抖动造成的判定误差
    mission_fsm.timer_ms += dt_ms;

    // 切入新目标后先给 behaviour 一个更新窗口，避免旧到位标志触发跳步
    if (mission_fsm.timer_ms < GOTO_MIN_EFFECTIVE_MS)
    {
        goto_arrive_confirm_cnt = 0;
        return;
    }

    if (goto_turn_only_step && mission_fsm.timer_ms < GOTO_TURN_MIN_MS)
    {
        goto_arrive_confirm_cnt = 0;
        return;
    }

    // 位置到位判定：使用 behaviour 到位标志 + 直接位置误差，减少单一标志失真
    pos_arrived = behaviour_is_arrived();
    if (odom != NULL)
    {
        fp32 dx = step->target_x - odom->position_x;
        fp32 dy = step->target_y - odom->position_y;
        dist = sqrtf(dx * dx + dy * dy);
        if (dist <= GOTO_DIRECT_ARRIVE_M)
        {
            pos_arrived = true;
        }

        // 原地转向步允许更宽的位置误差，避免位置判据阻塞纯转向
        if (goto_turn_only_step && dist <= GOTO_TURN_POS_RELAX_M)
        {
            pos_arrived = true;
        }

        // 里程计停滞检测：仅在“位置未到位”时启用，避免同点转向误判停滞
        {
            fp32 dmx = odom->position_x - goto_last_x;
            fp32 dmy = odom->position_y - goto_last_y;
            fp32 moved = sqrtf(dmx * dmx + dmy * dmy);

            if (moved >= GOTO_PROGRESS_EPS_M)
            {
                goto_odom_stale_ms = 0;
                goto_last_x = odom->position_x;
                goto_last_y = odom->position_y;
            }
            else if (!pos_arrived && !goto_turn_only_step && (dist > GOTO_SETTLE_ZONE_M))
            {
                goto_odom_stale_ms += dt_ms;
                if (goto_odom_stale_ms >= GOTO_ODOM_STALE_MS)
                {
                    // 靠近目标时不因“停滞”直接判错，避免低速微调被误伤
                    if (dist <= GOTO_STALE_NEAR_ARRIVE_M)
                    {
                        behaviour_disable();
                        goto_arrive_confirm_cnt = 0;
                        goto_odom_stale_ms = 0;
                        goto_yaw_stale_ms = 0;
                        goto_turn_only_step = 0;
                        mission_fsm.state = MISSION_STATE_NEXT_STEP;
                        return;
                    }

                    behaviour_disable();
                    goto_arrive_confirm_cnt = 0;
                    mission_fsm.error_flag = true;
                    mission_fsm.error_code = MISSION_ERR_GOTO_ODOM_STALE;
                    mission_fsm.state = MISSION_STATE_ERROR;
                    return;
                }
            }
            else if (goto_turn_only_step)
            {
                goto_odom_stale_ms = 0;
            }
        }
    }

    // 航向到位判定：使用最短角误差，避免跨 ±PI 时误差突变
    if (chassis != NULL)
    {
        fp32 yaw_err = atan2f(sinf(step->target_yaw - chassis->chassis_yaw),
                              cosf(step->target_yaw - chassis->chassis_yaw));
        yaw_err_abs = fabsf(yaw_err);
        yaw_arrived = (yaw_err_abs <= GOTO_YAW_ARRIVE_RAD);

        // 航向停滞检测：仅在原地转向且航向未到位时启用
        if (goto_turn_only_step && !yaw_arrived)
        {
            fp32 dyaw = atan2f(sinf(chassis->chassis_yaw - goto_last_yaw),
                               cosf(chassis->chassis_yaw - goto_last_yaw));
            if (fabsf(dyaw) >= GOTO_YAW_PROGRESS_EPS)
            {
                goto_yaw_stale_ms = 0;
                goto_last_yaw = chassis->chassis_yaw;
            }
            else
            {
                goto_yaw_stale_ms += dt_ms;
                if (goto_yaw_stale_ms >= GOTO_YAW_STALE_MS)
                {
                    behaviour_disable();
                    goto_arrive_confirm_cnt = 0;
                    mission_fsm.error_flag = true;
                    mission_fsm.error_code = MISSION_ERR_GOTO_YAW_STALE;
                    mission_fsm.state = MISSION_STATE_ERROR;
                    return;
                }
            }
        }
        else
        {
            goto_yaw_stale_ms = 0;
            goto_last_yaw = chassis->chassis_yaw;
        }
    }

    // 微调区判定：位置接近且航向误差不大，给予额外超时宽限
    if (dist <= GOTO_SETTLE_ZONE_M)
    {
        if (chassis == NULL || yaw_err_abs <= GOTO_SETTLE_YAW_RAD)
        {
            in_settle_zone = true;
        }
    }
    if (in_settle_zone)
    {
        timeout_limit += GOTO_SETTLE_GRACE_MS;
    }

    if (mission_fsm.timer_ms >= timeout_limit)
    {
        // 超时
        behaviour_disable();
        goto_arrive_confirm_cnt = 0;
        mission_fsm.error_flag = true;
        mission_fsm.error_code = MISSION_ERR_GOTO_TIMEOUT;
        mission_fsm.state = MISSION_STATE_ERROR;
        return;
    }

    if (chassis == NULL)
    {
        // 无法读到底盘航向时，不阻塞任务流程
        yaw_arrived = true;
    }

    arrived = pos_arrived && yaw_arrived;

    if (arrived)
    {
        if (goto_arrive_confirm_cnt < GOTO_ARRIVE_CONFIRM_CNT)
        {
            goto_arrive_confirm_cnt++;
        }

        if (goto_arrive_confirm_cnt >= GOTO_ARRIVE_CONFIRM_CNT)
        {
            behaviour_disable();
            goto_arrive_confirm_cnt = 0;
            goto_odom_stale_ms = 0;
            goto_yaw_stale_ms = 0;
            goto_turn_only_step = 0;
            mission_fsm.state = MISSION_STATE_NEXT_STEP;
        }
    }
    else
    {
        goto_arrive_confirm_cnt = 0;
    }
}

static void FSM_StateVisualServo(void)
{
    mission_fsm.timer_ms += 10;
    
    // 获取视觉误差
    //float err_x = Vision_Get_Error_X();
    
    // 如果误差足够小，认为对准完成
    //if (fabs(err_x) < 0.05f) {
        //Vision_Stop(); // 关闭视觉
    //    mission_fsm.state = MISSION_STATE_NEXT_STEP; // 进入下一步
    //}
    // 超时处理
    //else if (mission_fsm.timer_ms >= mission_fsm.timeout_ms) {
    //    mission_fsm.state = MISSION_STATE_ERROR;
    //}
}

static void FSM_StateGimbalAction(void)
{
    mission_fsm.timer_ms += 10;
    
    if (Check_StepComplete())
    {
        mission_fsm.gimbal_sub_state = GIMBAL_SUB_COMPLETE;
        mission_fsm.state = MISSION_STATE_NEXT_STEP;
    }
    else if (mission_fsm.timer_ms >= mission_fsm.timeout_ms)
    {
        ZDT_Motor_Stop_All();
        mission_fsm.error_flag = true;
        mission_fsm.error_code = MISSION_ERR_GIMBAL_TIMEOUT;
        mission_fsm.state = MISSION_STATE_ERROR;
    }
}

static void FSM_StateWaitComplete(void)
{
    mission_fsm.timer_ms += 10;
    
    if (mission_fsm.timer_ms >= mission_fsm.timeout_ms)
    {
        mission_fsm.state = MISSION_STATE_NEXT_STEP;
    }
}

static void FSM_StateNextStep(void)
{
    mission_fsm.current_step++;
    
    if (mission_fsm.current_step >= mission_fsm.step_count)
    {
        mission_fsm.state = MISSION_STATE_DONE;
    }
    else
    {
        Execute_CurrentStep();
    }
}

static void FSM_StateDone(void)
{
    mission_fsm.is_running = false;
    // 可以在此添加完成回调或通知
}

static void FSM_StateError(void)
{
    mission_fsm.is_running = false;
    behaviour_disable();
    ZDT_Motor_Stop_All();
    // 可以添加错误处理、报警等
}
