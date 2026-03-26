#include "test_task.h"
#include "pid.h"
#include "bsp_encoder.h"
#include "dc_motor.h"
#include <stddef.h>
#include "math.h"
#include "struct_typedef.h"
#include "chassis_task.h"

// ==================== 圆周运动参数 ====================
#define TEST_TASK_CIRCLE_RADIUS_M (1.4f)
#define TEST_TASK_CIRCLE_SPEED_MPS (0.6f)
#define TEST_TASK_CIRCLE_DT_S (0.04f)
#define TEST_TASK_SPIN_WZ_RAD_S (2.3f)  // 自转角速度 rad/s（0.10764/0.04 ≈ 2.69）

static uint8_t test_task_circle_enable = 0;

void test_task_circle_start(void)
{
	test_task_circle_enable = 1;
}

void test_task_circle_stop(void)
{
	test_task_circle_enable = 0;
	chassis_cmd_set_speed_world(0.0f, 0.0f);
}

void test_task_circle_40ms(void)
{
	if (test_task_circle_enable == 0)
	{
		return;
	}

	const fp32 omega = TEST_TASK_CIRCLE_SPEED_MPS / TEST_TASK_CIRCLE_RADIUS_M;
	static fp32 theta = 0.0f;

	// 角度推进
	theta += omega * TEST_TASK_CIRCLE_DT_S;
	if (theta > (2.0f * PI))
	{
		theta -= (2.0f * PI);
	}

	// 世界系切向速度，使轨迹为半径 0.5m 的圆
	const fp32 vx_world = -TEST_TASK_CIRCLE_SPEED_MPS * sinf(theta);
	const fp32 vy_world =  TEST_TASK_CIRCLE_SPEED_MPS * cosf(theta);

	// 设置速度指令：世界系速度 + 角速度直接控制（不用位置环）
	chassis_set_world_target(vx_world, vy_world, TEST_TASK_SPIN_WZ_RAD_S);
}

// ==================== 车头朝向圆心画圆 ====================
static uint8_t test_task_circle_face_center_enable = 0;

void test_task_circle_face_center_start(void)
{
	test_task_circle_face_center_enable = 1;
}

void test_task_circle_face_center_stop(void)
{
	test_task_circle_face_center_enable = 0;
	chassis_cmd_set_speed_world(0.0f, 0.0f);
}

void test_task_circle_face_center_40ms(void)
{
	if (test_task_circle_face_center_enable == 0)
	{
		return;
	}

	const fp32 omega = TEST_TASK_CIRCLE_SPEED_MPS / TEST_TASK_CIRCLE_RADIUS_M;
	static fp32 theta = 0.0f;

	// 角度推进
	theta += omega * TEST_TASK_CIRCLE_DT_S;
	if (theta > PI)
	{
		theta -= 2.0f * PI;
	}
	else if (theta < -PI)
	{
		theta += 2.0f * PI;
	}

	// 世界系切向速度
	const fp32 vx_world = -TEST_TASK_CIRCLE_SPEED_MPS * sinf(theta);
	const fp32 vy_world =  TEST_TASK_CIRCLE_SPEED_MPS * cosf(theta);

	// 车头朝向圆心：yaw = theta + π（小车在圆周上，圆心方向是径向向内）
	fp32 yaw_face_center = theta + PI;
	if (yaw_face_center > PI)
	{
		yaw_face_center -= 2.0f * PI;
	}

	// 设置速度指令：世界系速度 + 航向锁定（车头始终朝向圆心）
	chassis_set_world_target_yaw(vx_world, vy_world, yaw_face_center);
}

