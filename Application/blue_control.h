#ifndef BLUE_CONTROL_H
#define BLUE_CONTROL_H

#include "struct_typedef.h"
#include "usart.h"
#include <stdint.h>
#include <stdbool.h>

/*-------------------------- 蓝牙命令定义 --------------------------*/

// 命令类型
typedef enum {
    BLUE_CMD_STOP = 0x00,           // 停止
    BLUE_CMD_SPEED_BODY,            // 车体系速度控制
    BLUE_CMD_SPEED_WORLD,           // 世界系速度控制
    BLUE_CMD_SPEED_WORLD_YAW,       // 世界系速度 + 航向角
    BLUE_CMD_YAW_ABS,               // 设置绝对航向角
    BLUE_CMD_YAW_REL,               // 设置相对航向角
    BLUE_CMD_MISSION_START,         // 启动任务
    BLUE_CMD_MISSION_STOP,          // 停止任务
} blue_cmd_type_e;

// 蓝牙数据包格式（协议：帧头 + 命令类型 + 数据 + 校验）
// 帧头: 0xAA 0x55
// 数据长度根据命令类型而定
#define BLUE_FRAME_HEADER_0     0xAA
#define BLUE_FRAME_HEADER_1     0x55
#define BLUE_RX_BUF_SIZE        64

/*-------------------------- 蓝牙控制结构体 --------------------------*/

typedef struct {
    // 接收缓冲
    uint8_t rx_buf[BLUE_RX_BUF_SIZE];
    uint16_t rx_len;
    
    // 解析后的控制量
    fp32 vx;            // X方向速度 m/s
    fp32 vy;            // Y方向速度 m/s
    fp32 wz;            // 旋转角速度 rad/s
    fp32 yaw;           // 航向角 rad
    
    // 控制模式
    bool world_frame;   // true=世界系, false=车体系
    bool yaw_hold;      // true=航向角保持, false=角速度控制
    
    // 状态
    bool connected;     // 连接状态
    uint32_t last_rx_tick;  // 上次接收时间戳
    
} blue_control_t;

/*-------------------------- 外部接口 --------------------------*/

/**
 * @brief 蓝牙控制模块初始化（启动DMA空闲接收）
 * @param huart 蓝牙所用的UART句柄（如 &huart1）
 */
void Blue_Control_Init(UART_HandleTypeDef *huart);

/**
 * @brief 蓝牙DMA空闲接收回调（在 HAL_UARTEx_RxEventCallback 中调用）
 * @param huart UART句柄
 * @param size 接收到的数据长度
 */
void Blue_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);

/**
 * @brief 蓝牙控制周期处理（在主循环或定时任务中调用）
 * @note 检查超时、更新控制量到底盘
 */
void Blue_Control_Process(void);

/**
 * @brief 直接设置速度（供简单遥控器使用，无需协议解析）
 * @param vx X方向速度 m/s（世界系）
 * @param vy Y方向速度 m/s（世界系）
 * @param wz 旋转角速度 rad/s
 */
void Blue_Set_Speed(fp32 vx, fp32 vy, fp32 wz);

/**
 * @brief 停止蓝牙控制
 */
void Blue_Control_Stop(void);

/**
 * @brief 查询蓝牙是否连接（最近有收到数据）
 */
bool Blue_Is_Connected(void);

/**
 * @brief 获取蓝牙控制结构体指针（调试用）
 */
const blue_control_t* Blue_Get_Control(void);

#endif /* BLUE_CONTROL_H */