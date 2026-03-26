/**
 * @file chassis_behaviour.c
 * @brief 底盘行为/位置环外环控制模块
 * ----------------------------------------------------------------------------
 * @version 1.0.0.0
 * @date 2026-01-27
 * @remark 位置环外环，输出世界系速度指令给 chassis_task
 */

/*-----------------------------------头文件引用-----------------------------------*/

#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "CAN_receive.h"
#include <math.h>

/*-----------------------------------宏定义-----------------------------------*/

#define BEHAVIOUR_ODOM_OFFLINE_MISS_THRESHOLD 5U


/*-----------------------------------变量声明-----------------------------------*/

// 位置环控制数据
static chassis_odometry_t chassis_odometry;

/*------------------------ 位置环指令（由外部任务/主循环写入） -------------------------*/

static volatile fp32 g_behaviour_target_x = 0.0f;
static volatile fp32 g_behaviour_target_y = 0.0f;
static volatile fp32 g_behaviour_target_yaw = 0.0f;
static volatile uint8_t g_behaviour_enable = 0;
static volatile uint8_t g_behaviour_odom_fresh = 0;
static volatile uint8_t g_behaviour_odom_miss_cnt = 0;

static fp32 behaviour_clamp_abs(fp32 value, fp32 abs_limit)
{
    if (value > abs_limit)
    {
        return abs_limit;
    }
    if (value < -abs_limit)
    {
        return -abs_limit;
    }
    return value;
}

/*-----------------------------------内部函数声明-----------------------------------*/

/**
 * @brief  位置环模块初始化
 * @param  odom_init  里程计结构体指针
 * @retval none
 */
static void behaviour_init(chassis_odometry_t *odom_init);

/**
 * @brief  位置环反馈数据更新
 * @param  odom_update  里程计结构体指针
 * @retval none
 */
static void behaviour_feedback_update(chassis_odometry_t *odom_update);

/**
 * @brief  位置环控制量计算
 * @param  odom_control  里程计结构体指针
 * @retval none
 */
static void behaviour_control_calc(chassis_odometry_t *odom_control);

/**
 * @brief  位置环输出到底盘任务
 * @param  odom_output  里程计结构体指针
 * @retval none
 */
static void behaviour_output(chassis_odometry_t *odom_output);

/*-----------------------------------函数实现-----------------------------------*/

// ------------------------- 对外接口实现 -------------------------

void behaviour_set_position(fp32 x_m, fp32 y_m)
{
    g_behaviour_target_x = behaviour_clamp_abs(x_m, CHASSIS_POSITION_X_LIMIT_M);
    g_behaviour_target_y = behaviour_clamp_abs(y_m, CHASSIS_POSITION_Y_LIMIT_M);
}

void behaviour_set_position_yaw(fp32 x_m, fp32 y_m, fp32 yaw_rad)
{
    g_behaviour_target_x = behaviour_clamp_abs(x_m, CHASSIS_POSITION_X_LIMIT_M);
    g_behaviour_target_y = behaviour_clamp_abs(y_m, CHASSIS_POSITION_Y_LIMIT_M);
    g_behaviour_target_yaw = yaw_rad;
}

void behaviour_enable(uint8_t latch_current_target)
{
    g_behaviour_enable = 1;
    chassis_odometry.mode = BEHAVIOUR_MODE_POSITION;
    // 可选：使能时锁存当前位置作为初始目标，避免突跳
    if (latch_current_target)
    {
        g_behaviour_target_x = chassis_odometry.position_x;
        g_behaviour_target_y = chassis_odometry.position_y;
    }
}

void behaviour_disable(void)
{
    g_behaviour_enable = 0;
    chassis_odometry.mode = BEHAVIOUR_MODE_DISABLE;
    // 关闭时清除PID积分，避免下次使能时残留
    PID_clear(&chassis_odometry.position_x_pid);
    PID_clear(&chassis_odometry.position_y_pid);

    // 位置环曾通过 chassis_set_world_target_yaw() 启用“世界系速度 + 航向保持”。
    // 仅关闭 behaviour 自身并不会自动关闭底盘的航向角闭环，因此这里一并释放底盘控制状态。
    chassis_cmd_disable_yaw_hold();
    chassis_cmd_disable_world_frame();
    chassis_cmd_set_wz(0.0f);
    chassis_cmd_set_speed_body(0.0f, 0.0f);
}

uint8_t behaviour_is_arrived(void)
{
    return chassis_odometry.arrive_flag;
}

const chassis_odometry_t *get_behaviour_data(void)
{
    return &chassis_odometry;
}

// ------------------------- 初始化与周期任务 -------------------------

void global_behaviour_init(void)
{
    behaviour_init(&chassis_odometry);
    g_behaviour_odom_fresh = 1;
    g_behaviour_odom_miss_cnt = 0;
}


// ------------------------- 内部函数实现 -------------------------

static void behaviour_init(chassis_odometry_t *odom_init)
{
    if (odom_init == NULL)
    {
        return;
    }

    // 获取CAN里程计数据指针
    extern volatile float can_distence_x_m;
    extern volatile float can_distence_y_m;
    odom_init->position_x_ptr = &can_distence_x_m;
    odom_init->position_y_ptr = &can_distence_y_m;

    // 位置环PID参数（X/Y轴各自独立实例，但初始参数相同）
    const static fp32 position_pid_x[3] = {
        CHASSIS_POSITION_PID_KP,
        CHASSIS_POSITION_PID_KI,
        CHASSIS_POSITION_PID_KD
    };
    const static fp32 position_pid_y[3] = {
        CHASSIS_POSITION_PID_KP,
        CHASSIS_POSITION_PID_KI,
        CHASSIS_POSITION_PID_KD
    };

    // 初始化X轴位置PID
    PID_init(&odom_init->position_x_pid, PID_POSITION, position_pid_x,
             CHASSIS_POSITION_PID_MAX_OUT, CHASSIS_POSITION_PID_MAX_IOUT);

    // 初始化Y轴位置PID
    PID_init(&odom_init->position_y_pid, PID_POSITION, position_pid_y,
             CHASSIS_POSITION_PID_MAX_OUT, CHASSIS_POSITION_PID_MAX_IOUT);

    // 初始状态
    odom_init->mode = BEHAVIOUR_MODE_DISABLE;
    odom_init->arrive_flag = 0;

    // 读取初始位置
    odom_init->position_x = *(odom_init->position_x_ptr);
    odom_init->position_y = *(odom_init->position_y_ptr);

    // 目标位置初始化为当前位置
    odom_init->position_x_set = odom_init->position_x;
    odom_init->position_y_set = odom_init->position_y;
    odom_init->yaw_set = 0.0f;

    // 输出清零
    odom_init->vx_out = 0.0f;
    odom_init->vy_out = 0.0f;
}

// 修改position_x_ptr和position_y_ptr指针，使其指向CAN接收的里程计数据变量和来自maxicam的数据变量
static void behaviour_feedback_pointer(chassis_odometry_t *odom_update)
{
    if (odom_update == NULL)
    {
        return;
    }
    
    // 从CAN接收的里程计数据更新当前位置
    odom_update->position_x = *(odom_update->position_x_ptr);
    odom_update->position_y = *(odom_update->position_y_ptr);
}

static void behaviour_feedback_update(chassis_odometry_t *odom_update)
{
    if (odom_update == NULL)
    {
        return;
    }

    if (can_odom_new_data_flag == 0)
    {
        // 本周期未收到新里程计，累计丢包计数
        if (g_behaviour_odom_miss_cnt < BEHAVIOUR_ODOM_OFFLINE_MISS_THRESHOLD)
        {
            g_behaviour_odom_miss_cnt++;
        }

        // 连续丢包达到阈值后，才触发离线保护
        g_behaviour_odom_fresh = (g_behaviour_odom_miss_cnt < BEHAVIOUR_ODOM_OFFLINE_MISS_THRESHOLD) ? 1 : 0;
        return;
    }

    // 位置环读取到新数据后立即清零标志，等待下次中断再次置位
    can_odom_new_data_flag = 0;
    g_behaviour_odom_miss_cnt = 0;
    g_behaviour_odom_fresh = 1;

    // 从CAN接收的里程计数据更新当前位置
    odom_update->position_x = *(odom_update->position_x_ptr);
    odom_update->position_y = *(odom_update->position_y_ptr);
}

static void behaviour_control_calc(chassis_odometry_t *odom_control)
{
    static fp32 last_err_x = 0.0f;
    static fp32 last_err_y = 0.0f;

    if (odom_control == NULL)
    {
        return;
    }

    // 模式判断：未使能则不计算
    if (odom_control->mode == BEHAVIOUR_MODE_DISABLE || g_behaviour_enable == 0)
    {
        odom_control->vx_out = 0.0f;
        odom_control->vy_out = 0.0f;
        odom_control->arrive_flag = 0;
        last_err_x = 0.0f;
        last_err_y = 0.0f;
        return;
    }

    // 离线保护：位置环周期内无新里程计，立即下发0速度
    if (g_behaviour_odom_fresh == 0)
    {
        odom_control->vx_out = 0.0f;
        odom_control->vy_out = 0.0f;
        odom_control->arrive_flag = 0;
        last_err_x = 0.0f;
        last_err_y = 0.0f;
        return;
    }

    // 更新目标位置（从外部指令读取）
    odom_control->position_x_set = g_behaviour_target_x;
    odom_control->position_y_set = g_behaviour_target_y;
    odom_control->yaw_set = g_behaviour_target_yaw;

    // 计算位置误差
    fp32 err_x = odom_control->position_x_set - odom_control->position_x;
    fp32 err_y = odom_control->position_y_set - odom_control->position_y;

    // 到位判定
    fp32 dist = sqrtf(err_x * err_x + err_y * err_y);
    if (dist < CHASSIS_POSITION_ARRIVE_THRESHOLD)
    {
        odom_control->arrive_flag = 1;
    }
    else
    {
        odom_control->arrive_flag = 0;
    }

    // 死区处理：误差很小时不输出，避免抖动
    if (fabsf(err_x) < CHASSIS_POSITION_DEADBAND)
    {
        err_x = 0.0f;
    }
    if (fabsf(err_y) < CHASSIS_POSITION_DEADBAND)
    {
        err_y = 0.0f;
    }
    /*
    // 【新增】非线性开根号 P 控制算法
    // 1. 提取误差的方向符号 (如果是正数就乘 1.0，负数就乘 -1.0)
    fp32 dir_x = (err_x > 0.0f) ? -1.0f : ((err_x < 0.0f) ? 1.0f : 0.0f);
    fp32 dir_y = (err_y > 0.0f) ? -1.0f : ((err_y < 0.0f) ? 1.0f : 0.0f);

    // 2. 根据公式 V = Kp * sqrt(|err|) * dir 计算期望速度
    // 注意：sqrtf 是 math.h 中针对 float 的开方函数，fabsf 是取绝对值
    fp32 target_vx = CHASSIS_POSITION_ROOT_KP * sqrtf(fabsf(err_x)) * dir_x;
    fp32 target_vy = CHASSIS_POSITION_ROOT_KP * sqrtf(fabsf(err_y)) * dir_y;

    // 3. 叠加D项（沿用原PID的符号约定：D = -Kd * (err(k)-err(k-1))）
    fp32 d_term_x = -CHASSIS_POSITION_ROOT_KD * (err_x - last_err_x);
    fp32 d_term_y = -CHASSIS_POSITION_ROOT_KD * (err_y - last_err_y);

    // 4. 限幅输出，保护最大速度不超标
    odom_control->vx_out = behaviour_clamp_abs(target_vx + d_term_x, CHASSIS_POSITION_PID_MAX_OUT);
    odom_control->vy_out = behaviour_clamp_abs(target_vy + d_term_y, CHASSIS_POSITION_PID_MAX_OUT);

    // 更新误差历史，供下一周期计算D项
    last_err_x = err_x;
    last_err_y = err_y;
    */
    // 【保留】PID计算：set=0, ref=err（误差PID方式）
    // 输出为速度指令 (m/s)
    odom_control->vx_out = PID_calc(&odom_control->position_x_pid, 0.0f, -err_x);
    odom_control->vy_out = PID_calc(&odom_control->position_y_pid, 0.0f, -err_y);
}

static void behaviour_output(chassis_odometry_t *odom_output)
{
    if (odom_output == NULL)
    {
        return;
    }

    // 未使能时不输出
    if (odom_output->mode == BEHAVIOUR_MODE_DISABLE || g_behaviour_enable == 0)
    {
        return;
    }

    // 调用底盘任务的世界系速度+航向角接口
    chassis_set_world_target_yaw(odom_output->vx_out, odom_output->vy_out, odom_output->yaw_set);
}