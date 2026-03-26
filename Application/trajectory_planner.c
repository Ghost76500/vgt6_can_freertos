/**
 * @file trajectory_planner.c
 * @brief 直线梯形速度轨迹规划器（通过 behaviour 接口下发参考点）。
 *
 * 说明：
 * 1) 本模块不替换 behaviour 的位置环，仅负责生成“随时间变化的参考位置”；
 * 2) 通过 preview_time 做简单前馈预瞄（提前给参考点）；
 * 3) 末段进入 SETTLE 后，交给 behaviour 自身到位判定收敛。
 */

#include "trajectory_planner.h"

#include "chassis_behaviour.h"
#include "main.h"
#include <math.h>

// 默认参数：可在初始化后由 set_* 接口覆盖
#define TRAJ_DEFAULT_MAX_VEL_MPS      0.45f   // 默认最大速度（m/s）
#define TRAJ_DEFAULT_MAX_ACC_MPS2     0.35f   // 默认最大加速度（m/s^2）
#define TRAJ_DEFAULT_PREVIEW_TIME_S   0.06f   // 默认预瞄时间（s），配合底盘内环一阶滤波取保守值
#define TRAJ_DEFAULT_ARRIVE_HOLD      2u      // 默认连续到位计数门限
#define TRAJ_EPS_DIST_M               0.0005f // 小于该距离视为“几乎零位移”

typedef struct
{
    fp32 max_vel;                 // 规划最大速度
    fp32 max_acc;                 // 规划最大加速度
    fp32 preview_time;            // 前馈预瞄时间
    uint8_t arrive_hold_cycles;   // 完成判定连续计数门限

    fp32 ux;                      // 轨迹方向单位向量 x
    fp32 uy;                      // 轨迹方向单位向量 y

    fp32 t_acc;                   // 加速段时间
    fp32 t_cruise;                // 匀速段时间
    fp32 t_total;                 // 总时间
    fp32 v_peak;                  // 峰值速度（可能小于 max_vel）

    fp32 s_acc;                   // 加速段走过的位移

    uint8_t settle_counter;       // SETTLE 阶段连续到位计数

    trajectory_planner_state_t state;
} trajectory_planner_ctx_t;

static trajectory_planner_ctx_t g_traj;
static uint32_t g_traj_last_tick_ms = 0u;

static fp32 traj_clampf(fp32 v, fp32 vmin, fp32 vmax)
{
    if (v < vmin)
    {
        return vmin;
    }
    if (v > vmax)
    {
        return vmax;
    }
    return v;
}

static void trajectory_compute_profile(trajectory_planner_ctx_t *ctx)
{
    // 对“总位移 d”做梯形/三角速度剖面分解。
    fp32 d = ctx->state.total_dist;
    fp32 vmax = ctx->max_vel;
    fp32 amax = ctx->max_acc;

    fp32 d_switch = (vmax * vmax) / amax;
    if (d <= d_switch)
    {
        // 三角剖面：距离不足以到达 vmax
        ctx->v_peak = sqrtf(d * amax);
        ctx->t_acc = ctx->v_peak / amax;
        ctx->t_cruise = 0.0f;
        ctx->t_total = 2.0f * ctx->t_acc;
    }
    else
    {
        // 梯形剖面：加速-匀速-减速
        ctx->v_peak = vmax;
        ctx->t_acc = vmax / amax;
        ctx->t_cruise = (d - d_switch) / vmax;
        ctx->t_total = 2.0f * ctx->t_acc + ctx->t_cruise;
    }

    ctx->s_acc = 0.5f * amax * ctx->t_acc * ctx->t_acc;
}

void trajectory_planner_init(void)
{
    // 默认参数初始化
    g_traj.max_vel = TRAJ_DEFAULT_MAX_VEL_MPS;
    g_traj.max_acc = TRAJ_DEFAULT_MAX_ACC_MPS2;
    g_traj.preview_time = TRAJ_DEFAULT_PREVIEW_TIME_S;
    g_traj.arrive_hold_cycles = TRAJ_DEFAULT_ARRIVE_HOLD;
    g_traj.settle_counter = 0;

    g_traj.state.running = 0;
    g_traj.state.finished = 1;
    g_traj.state.phase = TRAJ_PHASE_IDLE;
    g_traj.state.start_x = 0.0f;
    g_traj.state.start_y = 0.0f;
    g_traj.state.target_x = 0.0f;
    g_traj.state.target_y = 0.0f;
    g_traj.state.target_yaw = 0.0f;
    g_traj.state.ref_x = 0.0f;
    g_traj.state.ref_y = 0.0f;
    g_traj.state.ref_s = 0.0f;
    g_traj.state.ref_v = 0.0f;
    g_traj.state.total_dist = 0.0f;
    g_traj.state.elapsed_s = 0.0f;

    g_traj_last_tick_ms = HAL_GetTick();
}

void global_trajectory_planner_init(void)
{
    trajectory_planner_init();
}

void trajectory_planner_task_cycle(void)
{
    // 工程采用统一周期调度文件，这里按 100ms 节拍推进轨迹。
    uint32_t now_tick = HAL_GetTick();
    if ((now_tick - g_traj_last_tick_ms) < TRAJECTORY_PLANNER_CONTROL_TIME_MS)
    {
        return;
    }
    g_traj_last_tick_ms = now_tick;

    trajectory_planner_update();
}

void trajectory_planner_set_limits(fp32 max_vel_mps, fp32 max_acc_mps2)
{
    // 非法值直接忽略，避免误配导致异常。
    if (max_vel_mps > 0.0f)
    {
        g_traj.max_vel = max_vel_mps;
    }
    if (max_acc_mps2 > 0.0f)
    {
        g_traj.max_acc = max_acc_mps2;
    }
}

void trajectory_planner_set_feedforward(fp32 preview_time_s)
{
    // 预瞄时间做限幅，避免给出过大超前目标。
    g_traj.preview_time = traj_clampf(preview_time_s, 0.0f, 0.30f);
}

void trajectory_planner_set_arrive_hold(uint8_t hold_cycles)
{
    if (hold_cycles == 0u)
    {
        g_traj.arrive_hold_cycles = 1u;
    }
    else
    {
        g_traj.arrive_hold_cycles = hold_cycles;
    }
}

void trajectory_planner_start_line(fp32 target_x_m,
                                   fp32 target_y_m,
                                   fp32 target_yaw_rad,
                                   uint8_t latch_start_from_current)
{
    // 使用 behaviour 的当前里程计作为起点来源。
    const chassis_odometry_t *odom = get_behaviour_data();
    if (odom == 0)
    {
        return;
    }

    if (latch_start_from_current)
    {
        // 常用模式：每次启动都从实时位置重新规划。
        g_traj.state.start_x = odom->position_x;
        g_traj.state.start_y = odom->position_y;
    }
    else if (g_traj.state.running == 0u)
    {
        // 若未要求锁存且当前未运行，也使用实时位置作为起点，避免沿用陈旧起点。
        g_traj.state.start_x = odom->position_x;
        g_traj.state.start_y = odom->position_y;
    }

    g_traj.state.target_x = target_x_m;
    g_traj.state.target_y = target_y_m;
    g_traj.state.target_yaw = target_yaw_rad;

    fp32 dx = g_traj.state.target_x - g_traj.state.start_x;
    fp32 dy = g_traj.state.target_y - g_traj.state.start_y;
    fp32 dist = sqrtf(dx * dx + dy * dy);
    g_traj.state.total_dist = dist;

    g_traj.state.ref_s = 0.0f;
    g_traj.state.ref_v = 0.0f;
    g_traj.state.elapsed_s = 0.0f;
    g_traj.settle_counter = 0u;

    if (dist < TRAJ_EPS_DIST_M)
    {
        // 目标几乎与当前重合，直接进入收敛阶段。
        g_traj.ux = 0.0f;
        g_traj.uy = 0.0f;
        g_traj.state.ref_x = g_traj.state.target_x;
        g_traj.state.ref_y = g_traj.state.target_y;
        g_traj.state.phase = TRAJ_PHASE_SETTLE;
    }
    else
    {
        // 正常路径：先计算方向单位向量和时间剖面。
        g_traj.ux = dx / dist;
        g_traj.uy = dy / dist;
        trajectory_compute_profile(&g_traj);
        g_traj.state.ref_x = g_traj.state.start_x;
        g_traj.state.ref_y = g_traj.state.start_y;
        g_traj.state.phase = TRAJ_PHASE_ACCEL;
    }

    g_traj.state.running = 1u;
    g_traj.state.finished = 0u;
    g_traj_last_tick_ms = HAL_GetTick();

    // 启动 behaviour 并给首个参考点。
    behaviour_enable(0u);
    behaviour_set_position_yaw(g_traj.state.ref_x, g_traj.state.ref_y, g_traj.state.target_yaw);
}

void trajectory_planner_stop(void)
{
    // 停止后主动关闭 behaviour，避免保留旧目标。
    g_traj.state.running = 0u;
    g_traj.state.finished = 0u;
    g_traj.state.phase = TRAJ_PHASE_IDLE;
    g_traj.state.ref_v = 0.0f;
    g_traj.state.elapsed_s = 0.0f;
    g_traj.settle_counter = 0u;
    behaviour_disable();
}

void trajectory_planner_update(void)
{
    // 必须周期调用，否则轨迹不会推进。
    if (g_traj.state.running == 0u)
    {
        return;
    }

    if (g_traj.state.phase == TRAJ_PHASE_SETTLE)
    {
        // 收敛阶段直接给最终点，让 behaviour 做最后误差收敛。
        behaviour_set_position_yaw(g_traj.state.target_x, g_traj.state.target_y, g_traj.state.target_yaw);

        if (behaviour_is_arrived())
        {
            g_traj.settle_counter++;
            if (g_traj.settle_counter >= g_traj.arrive_hold_cycles)
            {
                // 连续到位满足门限，判定完成。
                g_traj.state.running = 0u;
                g_traj.state.finished = 1u;
                g_traj.state.phase = TRAJ_PHASE_DONE;
            }
        }
        else
        {
            g_traj.settle_counter = 0u;
        }

        return;
    }

    // 按规划器节拍推进，保持与外环一致。
    g_traj.state.elapsed_s += TRAJECTORY_PLANNER_CONTROL_TIME;
    fp32 t = g_traj.state.elapsed_s;
    fp32 amax = g_traj.max_acc;
    fp32 s = 0.0f;
    fp32 v = 0.0f;

    if (t <= g_traj.t_acc)
    {
        // 加速段：s=1/2*a*t^2, v=a*t
        g_traj.state.phase = TRAJ_PHASE_ACCEL;
        s = 0.5f * amax * t * t;
        v = amax * t;
    }
    else if (t <= (g_traj.t_acc + g_traj.t_cruise))
    {
        // 匀速段：s=s_acc + v_peak*tc
        g_traj.state.phase = TRAJ_PHASE_CRUISE;
        fp32 tc = t - g_traj.t_acc;
        s = g_traj.s_acc + g_traj.v_peak * tc;
        v = g_traj.v_peak;
    }
    else if (t <= g_traj.t_total)
    {
        // 减速段：s=s0 + v_peak*td - 1/2*a*td^2
        g_traj.state.phase = TRAJ_PHASE_DECEL;
        fp32 td = t - (g_traj.t_acc + g_traj.t_cruise);
        s = g_traj.s_acc + g_traj.v_peak * g_traj.t_cruise +
            g_traj.v_peak * td - 0.5f * amax * td * td;
        v = g_traj.v_peak - amax * td;
    }
    else
    {
        // 轨迹时间走完，进入收敛阶段。
        g_traj.state.phase = TRAJ_PHASE_SETTLE;
        s = g_traj.state.total_dist;
        v = 0.0f;
    }

    if (s > g_traj.state.total_dist)
    {
        s = g_traj.state.total_dist;
    }
    if (v < 0.0f)
    {
        v = 0.0f;
    }

    g_traj.state.ref_s = s;
    g_traj.state.ref_v = v;

    // 前馈预瞄：将参考点沿路径方向前移 v*preview_time。
    fp32 preview_s = s + v * g_traj.preview_time;
    if (preview_s > g_traj.state.total_dist)
    {
        preview_s = g_traj.state.total_dist;
    }

    g_traj.state.ref_x = g_traj.state.start_x + g_traj.ux * preview_s;
    g_traj.state.ref_y = g_traj.state.start_y + g_traj.uy * preview_s;

    if (g_traj.state.phase == TRAJ_PHASE_SETTLE)
    {
        g_traj.state.ref_x = g_traj.state.target_x;
        g_traj.state.ref_y = g_traj.state.target_y;
    }

    behaviour_set_position_yaw(g_traj.state.ref_x, g_traj.state.ref_y, g_traj.state.target_yaw);
}

uint8_t trajectory_planner_is_running(void)
{
    return g_traj.state.running;
}

uint8_t trajectory_planner_is_finished(void)
{
    return g_traj.state.finished;
}

const trajectory_planner_state_t *trajectory_planner_get_state(void)
{
    return &g_traj.state;
}
