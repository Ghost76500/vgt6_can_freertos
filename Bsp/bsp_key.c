/**
 * @file    bsp_key.c
 * @brief   按键驱动模块实现
 */
#include "bsp_key.h"
#include "bsp_music.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdio.h> // 用于printf
#include "bsp_led.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"
#include "bsp_servo_pwm.h"
#include "gm65.h"
#include "usart.h"
#include "Emm_V5.h"
// #include "ZDT_X42_V2.h"
#include "test_task.h"
#include "dc_motor.h"
#include "chassis_task.h"
#include "position_task.h"

/* ================== 私有类型定义 ================== */

typedef struct
{
    GPIO_TypeDef* port;     // GPIO端口
    uint16_t      pin;      // GPIO引脚
    uint16_t      cnt;      // 消抖计数器
    uint8_t       lock;     // 自锁标志
    uint8_t       flag;     // 按下标志
    GPIO_PinState valid_level; // 有效电平（按下是高还是低）
} Key_t;

/* ================== 硬件配置数组 ================== */
static Key_t keys[KEY_COUNT] = {
    /* KEY_ID_0 */
    {GPIOE, GPIO_PIN_2,  0, 0, 0, GPIO_PIN_RESET}, // 假设PA0按下是低电平(RESET)
    
    /* KEY_ID_1 */
    {GPIOE, GPIO_PIN_3, 0, 0, 0, GPIO_PIN_RESET}, // 假设PC13按下是低电平

    /* KEY_ID_2 */
    {GPIOE, GPIO_PIN_4, 0, 0, 0, GPIO_PIN_RESET}, // 假设PC13按下是低电平

    {GPIOC, GPIO_PIN_13, 0, 0, 0, GPIO_PIN_RESET},

    {GPIOC, GPIO_PIN_14, 0, 0, 0, GPIO_PIN_RESET},
};

/* ================== 函数实现 ================== */

/**
 * @brief  按键扫描 (ISR上下文)
 * @note   每1ms调用一次，只负责置位标志，不执行具体耗时业务
 */
void Key_Scan_1ms(void)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        // 读取引脚并判断是否为有效电平
        if (HAL_GPIO_ReadPin(keys[i].port, keys[i].pin) == keys[i].valid_level)
        {
            if (keys[i].cnt < 25)
            {
                keys[i].cnt++;
            }

            if (keys[i].cnt >= 20) // 消抖确认
            {
                if (keys[i].lock == 0)
                {
                    keys[i].flag = 1; // 置位标志
                    keys[i].lock = 1; // 上锁
                }
            }
        }
        else
        {
            keys[i].cnt = 0;
            keys[i].lock = 0; // 解锁
        }
    }
}

/**
 * @brief  具体的按键功能执行 (Task上下文)
 * @note   在这里写 printf, LED翻转, 电机控制等逻辑
 */
static void Key_Execute_Action(Key_ID_t key_id)
{
    switch (key_id)
    {
        case KEY_ID_0:
            bsp_led_toggle(CORE_TWO);
            bsp_gimbal_angle_set(GIMBAL_MIN_ANGLE); // 设置云台到最小角度
            // uart_send(&huart5);
            //ZDT_X42_V2_Read_Sys_Params(0x01, S_PID);
            //Emm_V5_En_Control(0x01, true, 0x00); // 使能电机
            //Emm_V5_Origin_Set_O(1, true); // 设置当前位置为零点
            // 停止电机
            Emm_V5_Stop_Now(1, false);
            //test_task_run(80.0f);
            //Play_Song(WhiteAlbum, WhiteAlbum_Len, 10);
            //pwm_cmd_chassis(20, 20, 20, 20);
            position_set_position_yaw(1.0f, 0.0f, 0.0f);

            break;

        case KEY_ID_1:
            bsp_led_toggle(CORE_TWO);
            bsp_gimbal_angle_set(GIMBAL_MID_ANGLE); // 设置云台到中间角度
            //ZDT_X42_V2_En_Control(0x01, true, 0x00);
            //Emm_V5_Vel_Control(1, 1, 1000, 255, false);
            //test_task_run(40.0f);
            //Play_Song(BadApple, BadApple_Len, 10);
            //pwm_cmd_chassis(-20, -20, -20, 20);
            //Emm_V5_Stop_Now(1, false);
            position_enable(1);

            break;

        case KEY_ID_2:
            bsp_led_toggle(CORE_TWO);
            bsp_gimbal_angle_set(GIMBAL_MAX_ANGLE); // 设置云台到最大角度
            //ZDT_X42_V2_Velocity_Control(0x01, 0x01, 0x03E8, 0x4E20, 0x00);
            //Emm_V5_Pos_Control(1, 0, 500, 100, 5000, false, false); // 位置控制
            //test_task_run(0.0f);
            //Play_Song(FuwaFuwaTime, FuwaFuwaTime_Len, 10);
            //pwm_cmd_chassis(0, 0, 0, 0);
            position_disable();

            break;

        case KEY_ID_3:
            bsp_led_toggle(CORE_TWO);
            //ZDT_X42_V2_Stop_Now(0x01, 0x00);
            //Emm_V5_Pos_Control(1, 1, 2000, 200, 50000, false, false);
            //Emm_V5_Origin_Trigger_Return(1, 2, false); // 触发回零，多圈无限位碰撞回零
            //Emm_V5_Pos_Control(1, 1, 3000, 255, 150000, false, false); // 位置控制
            //Play_Song(Haruhikage_Full, Haruhikage_Full_Len, 10);
            //chassis_set_world_target_yaw(0.0f, 0.3f,0.0f);
            position_set_position_yaw(0.0f, 3.0f, 0.0f);
            //test_task_circle_start();
            //Play_Song(SenrenBanka, SenrenBanka_Len, 50);
            
            break;


        case KEY_ID_4:
            bsp_led_toggle(CORE_TWO);
            // ZDT_X42_V2_Bypass_Position_LV_Control(0X01, 0X01, 0X4e20, float position, uint8_t raf, uint8_t snF);
            //ZDT_X42_V2_Modify_Ctrl_Mode(0x01, true, 1);
            //Emm_V5_Stop_Now(1, false);
            //Play_Song(Mixue_Theme, Mixue_Theme_Len, 10);
            //Emm_V5_Stop_Now(1, false);
            position_set_position_yaw(0.0f, 0.0f, 0.0f);
            //test_task_circle_face_center_start();
            //Play_Song(MyBeloved, MyBeloved_Len, 50);    

            
            break; 

        default:
            break;
    }
}

/**
 * @brief  按键循环处理
 * @note   放在 main while(1) 中，轮询标志位
 */
void Key_Handle_Loop(void)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        if (keys[i].flag == 1)
        {
            keys[i].flag = 0; // 清除标志，防止重复执行
            
            // 执行具体的动作
            Key_Execute_Action((Key_ID_t)i);
        }
    }
}