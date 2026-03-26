#ifndef TEST_TASK_H
#define TEST_TASK_H

#include "stdint.h"
#include "struct_typedef.h"


/**
 * @brief 40ms周期调用：世界系速度环画圆
 * @note 半径=1m，使用世界系速度指令更新
 */
void test_task_circle_40ms(void);

/**
 * @brief 一键启动圆周运动
 */
void test_task_circle_start(void);

/**
 * @brief 停止圆周运动
 */
void test_task_circle_stop(void);

/**
 * @brief 40ms周期调用：车头朝向圆心画圆
 * @note 使用世界系速度+航向锁定，车头始终指向圆心
 */
void test_task_circle_face_center_40ms(void);

/**
 * @brief 启动车头朝向圆心画圆
 */
void test_task_circle_face_center_start(void);

/**
 * @brief 停止车头朝向圆心画圆
 */
void test_task_circle_face_center_stop(void);




#endif /* TEST_TASK_H */
