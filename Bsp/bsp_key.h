/**
 * @file    bsp_key.h
 * @brief   按键驱动模块头文件
 */
#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "main.h" // 引入HAL库和GPIO定义

/* ================== 用户配置区域 ================== */

// 定义按键的ID，方便在代码里用名字而不是数字
typedef enum
{
    KEY_ID_0 = 0,   // 对应数组第0个元素
    KEY_ID_1,       // 对应数组第1个元素
    KEY_ID_2,    // 如果有更多，继续往下加
    KEY_ID_3,
    KEY_ID_4,
    
    KEY_COUNT       // 自动计算按键总数，不要删
} Key_ID_t;

/* ================== 函数声明 ================== */

/**
 * @brief  按键扫描核心函数
 * @note   必须在 1ms 定时器中断中调用
 */
extern void Key_Scan_1ms(void);

/**
 * @brief  按键逻辑处理函数
 * @note   在 main 函数的 while(1) 中调用
 */
extern void Key_Handle_Loop(void);

#endif /* __BSP_KEY_H */