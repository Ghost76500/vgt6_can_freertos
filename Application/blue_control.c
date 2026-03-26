/**
 * @file blue_control.c
 * @brief 蓝牙遥控模块 - DMA空闲接收 + 世界系速度控制
 * @date 2026-01-31
 */

#include "blue_control.h"
#include "chassis_behaviour.h"
#include "chassis_task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*-----------------------------------宏定义-----------------------------------*/

// 连接超时时间（ms）
#define BLUE_TIMEOUT_MS         500

// 速度缩放系数（根据遥控器实际值范围调整）
#define BLUE_SPEED_SCALE        0.01f   // 接收值 * 该系数 = m/s
#define BLUE_YAW_SCALE          0.01f   // 接收值 * 该系数 = rad

// 归一化摇杆[-1,1] -> 速度/角速度映射
#define BLUE_JOY_MAX_SPEED_MPS      0.50f
#define BLUE_JOY_MAX_WZ_RAD_S       1.20f

// 位置环模式下：摇杆[-1,1]映射到“原点附近目标点”范围（m）
#define BLUE_JOY_POS_RANGE_X_M      0.60f
#define BLUE_JOY_POS_RANGE_Y_M      0.60f

/*-----------------------------------变量声明-----------------------------------*/

static blue_control_t blue_ctrl;
static UART_HandleTypeDef *blue_huart = NULL;
static uint8_t blue_dma_buf[BLUE_RX_BUF_SIZE];  // DMA接收缓冲

// VOFA控件状态（归一化摇杆 + 开关）
static fp32 blue_joy_x_norm = 0.0f;      // 水平轴，右正
static fp32 blue_joy_y_norm = 0.0f;      // 垂直轴，上正
static fp32 blue_joy_w_norm = 0.0f;      // 旋转轴，左转正（可选）
static bool blue_pos_loop_enable = false;
static bool blue_yaw_ctrl_enable = false;
static fp32 blue_pos_origin_x = 0.0f;    // 位置环开启时锁存原点
static fp32 blue_pos_origin_y = 0.0f;

/*-----------------------------------内部函数声明-----------------------------------*/

static void Blue_Parse_Protocol(uint8_t *data, uint16_t len);
static void Blue_Parse_Simple(uint8_t *data, uint16_t len);
static uint8_t Blue_Checksum(uint8_t *data, uint16_t len);
static fp32 Blue_Clamp(fp32 v, fp32 min_v, fp32 max_v);
static void Blue_Update_From_Normalized_Joy(void);
static void Blue_Set_Position_Loop(bool enable);
static void Blue_Set_Yaw_Ctrl(bool enable);
static char* Blue_Trim(char *s);
static void Blue_Parse_Text_Line(char *line);

/*-----------------------------------函数实现-----------------------------------*/

void Blue_Control_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return;
    
    blue_huart = huart;
    memset(&blue_ctrl, 0, sizeof(blue_ctrl));
    blue_ctrl.world_frame = true;   // 默认使用世界系
    blue_ctrl.yaw_hold = false;
    blue_ctrl.connected = false;
    
    // 启动DMA空闲接收
    HAL_UARTEx_ReceiveToIdle_DMA(blue_huart, blue_dma_buf, BLUE_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(blue_huart->hdmarx, DMA_IT_HT);  // 关闭半传输中断
}

void Blue_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != blue_huart || size == 0) return;
    
    // 复制数据到控制结构体
    blue_ctrl.rx_len = (size > BLUE_RX_BUF_SIZE) ? BLUE_RX_BUF_SIZE : size;
    memcpy(blue_ctrl.rx_buf, blue_dma_buf, blue_ctrl.rx_len);
    
    // 更新时间戳和连接状态
    blue_ctrl.last_rx_tick = HAL_GetTick();
    blue_ctrl.connected = true;
    
    // 解析数据
    // 检查是否为协议格式（帧头 0xAA 0x55）
    if (blue_ctrl.rx_len >= 4 && 
        blue_ctrl.rx_buf[0] == BLUE_FRAME_HEADER_0 && 
        blue_ctrl.rx_buf[1] == BLUE_FRAME_HEADER_1)
    {
        Blue_Parse_Protocol(blue_ctrl.rx_buf, blue_ctrl.rx_len);
    }
    else
    {
        // 简单ASCII命令解析（用于调试/简易遥控器）
        Blue_Parse_Simple(blue_ctrl.rx_buf, blue_ctrl.rx_len);
    }
    
    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(blue_huart, blue_dma_buf, BLUE_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(blue_huart->hdmarx, DMA_IT_HT);
}

void Blue_Control_Process(void)
{
    // 检查超时
    if (blue_ctrl.connected)
    {
        if (HAL_GetTick() - blue_ctrl.last_rx_tick > BLUE_TIMEOUT_MS)
        {
            blue_ctrl.connected = false;
            // 超时停止
            blue_ctrl.vx = 0;
            blue_ctrl.vy = 0;
            blue_ctrl.wz = 0;

            // 失联时退出位置环/角度控制，防止继续执行旧目标
            if (blue_pos_loop_enable)
            {
                Blue_Set_Position_Loop(false);
            }
            blue_yaw_ctrl_enable = false;
        }
    }
    
    // 输出控制量到底盘
    if (blue_ctrl.connected)
    {
        // 位置环模式：摇杆控制目标点（相对开启位置环时的原点）
        if (blue_pos_loop_enable)
        {
            fp32 target_x = blue_pos_origin_x + (-blue_joy_y_norm) * BLUE_JOY_POS_RANGE_X_M;
            fp32 target_y = blue_pos_origin_y + ( blue_joy_x_norm) * BLUE_JOY_POS_RANGE_Y_M;

            if (blue_yaw_ctrl_enable)
            {
                behaviour_set_position_yaw(target_x, target_y, blue_ctrl.yaw);
            }
            else
            {
                behaviour_set_position(target_x, target_y);
            }
            return;
        }

        if (blue_ctrl.world_frame)
        {
            if (blue_ctrl.yaw_hold)
            {
                // 世界系速度 + 航向角保持
                chassis_set_world_target_yaw(blue_ctrl.vx, blue_ctrl.vy, blue_ctrl.yaw);
            }
            else
            {
                // 世界系速度 + 角速度控制
                chassis_set_world_target(blue_ctrl.vx, blue_ctrl.vy, blue_ctrl.wz);
            }
        }
        else
        {
            // 车体系速度控制
            chassis_cmd_set_speed_body(blue_ctrl.vx, blue_ctrl.vy);
            chassis_cmd_set_wz(blue_ctrl.wz);
        }
    }
}

void Blue_Set_Speed(fp32 vx, fp32 vy, fp32 wz)
{
    blue_ctrl.vx = vx;
    blue_ctrl.vy = vy;
    blue_ctrl.wz = wz;
    blue_ctrl.world_frame = true;
    blue_ctrl.yaw_hold = false;
    blue_ctrl.connected = true;
    blue_ctrl.last_rx_tick = HAL_GetTick();
}

void Blue_Control_Stop(void)
{
    blue_ctrl.vx = 0;
    blue_ctrl.vy = 0;
    blue_ctrl.wz = 0;
    blue_ctrl.connected = false;
    
    // 停止底盘
    chassis_cmd_set_speed_body(0, 0);
    chassis_cmd_set_wz(0);
}

bool Blue_Is_Connected(void)
{
    return blue_ctrl.connected;
}

const blue_control_t* Blue_Get_Control(void)
{
    return &blue_ctrl;
}

/*-----------------------------------内部函数实现-----------------------------------*/

/**
 * @brief 解析协议格式数据
 * 帧格式: [0xAA] [0x55] [CMD] [LEN] [DATA...] [CHECKSUM]
 */
static void Blue_Parse_Protocol(uint8_t *data, uint16_t len)
{
    if (len < 5) return;  // 最小帧长度
    
    uint8_t cmd = data[2];
    uint8_t data_len = data[3];
    
    // 校验长度
    if (len < (uint16_t)(4 + data_len + 1)) return;
    
    // 校验和验证
    uint8_t checksum = Blue_Checksum(data, 4 + data_len);
    if (checksum != data[4 + data_len]) return;
    
    // 解析命令
    switch (cmd)
    {
        case BLUE_CMD_STOP:
            blue_ctrl.vx = 0;
            blue_ctrl.vy = 0;
            blue_ctrl.wz = 0;
            break;
            
        case BLUE_CMD_SPEED_BODY:
            // 数据: [vx_h][vx_l][vy_h][vy_l][wz_h][wz_l] (int16, 单位0.01m/s)
            if (data_len >= 6)
            {
                int16_t vx_raw = (int16_t)((data[4] << 8) | data[5]);
                int16_t vy_raw = (int16_t)((data[6] << 8) | data[7]);
                int16_t wz_raw = (int16_t)((data[8] << 8) | data[9]);
                
                blue_ctrl.vx = vx_raw * BLUE_SPEED_SCALE;
                blue_ctrl.vy = vy_raw * BLUE_SPEED_SCALE;
                blue_ctrl.wz = wz_raw * BLUE_YAW_SCALE;
                blue_ctrl.world_frame = false;
                blue_ctrl.yaw_hold = false;
            }
            break;
            
        case BLUE_CMD_SPEED_WORLD:
            // 数据: [vx_h][vx_l][vy_h][vy_l][wz_h][wz_l]
            if (data_len >= 6)
            {
                int16_t vx_raw = (int16_t)((data[4] << 8) | data[5]);
                int16_t vy_raw = (int16_t)((data[6] << 8) | data[7]);
                int16_t wz_raw = (int16_t)((data[8] << 8) | data[9]);
                
                blue_ctrl.vx = vx_raw * BLUE_SPEED_SCALE;
                blue_ctrl.vy = vy_raw * BLUE_SPEED_SCALE;
                blue_ctrl.wz = wz_raw * BLUE_YAW_SCALE;
                blue_ctrl.world_frame = true;
                blue_ctrl.yaw_hold = false;
            }
            break;
            
        case BLUE_CMD_SPEED_WORLD_YAW:
            // 数据: [vx_h][vx_l][vy_h][vy_l][yaw_h][yaw_l]
            if (data_len >= 6)
            {
                int16_t vx_raw = (int16_t)((data[4] << 8) | data[5]);
                int16_t vy_raw = (int16_t)((data[6] << 8) | data[7]);
                int16_t yaw_raw = (int16_t)((data[8] << 8) | data[9]);
                
                blue_ctrl.vx = vx_raw * BLUE_SPEED_SCALE;
                blue_ctrl.vy = vy_raw * BLUE_SPEED_SCALE;
                blue_ctrl.yaw = yaw_raw * BLUE_YAW_SCALE;
                blue_ctrl.world_frame = true;
                blue_ctrl.yaw_hold = true;
            }
            break;
            
        case BLUE_CMD_YAW_ABS:
            // 数据: [yaw_h][yaw_l]
            if (data_len >= 2)
            {
                int16_t yaw_raw = (int16_t)((data[4] << 8) | data[5]);
                blue_ctrl.yaw = yaw_raw * BLUE_YAW_SCALE;
                chassis_cmd_set_yaw_abs(blue_ctrl.yaw);
            }
            break;
            
        case BLUE_CMD_YAW_REL:
            // 数据: [yaw_h][yaw_l]
            if (data_len >= 2)
            {
                int16_t yaw_raw = (int16_t)((data[4] << 8) | data[5]);
                fp32 yaw_delta = yaw_raw * BLUE_YAW_SCALE;
                chassis_cmd_set_yaw_rel(yaw_delta);
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief 解析简单ASCII命令（调试用）
 * 格式: "W" / "S" / "A" / "D" / "Q" / "E" / "X" 等单字符
 *       或 "Vx,Vy,Wz\n" 格式的数值
 */
static void Blue_Parse_Simple(uint8_t *data, uint16_t len)
{
    if (len == 0) return;

    // 先复制为可写字符串
    char buf[64];
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    memcpy(buf, data, len);
    buf[len] = '\0';

    // 支持一帧中携带多行命令：逐行解析
    char *line = buf;
    while (line != NULL && *line != '\0')
    {
        char *next = strpbrk(line, "\r\n");
        if (next != NULL)
        {
            *next = '\0';
            next++;
            while (*next == '\r' || *next == '\n')
            {
                next++;
            }
        }

        line = Blue_Trim(line);
        if (*line != '\0')
        {
            Blue_Parse_Text_Line(line);
        }

        line = next;
    }
}

static void Blue_Parse_Text_Line(char *line)
{
    char *colon = strchr(line, ':');
    if (colon != NULL)
    {
        *colon = '\0';
        char *key = Blue_Trim(line);
        char *val = Blue_Trim(colon + 1);

        if (strcmp(key, "chassis") == 0 || strcmp(key, "joy") == 0)
        {
            fp32 x = 0.0f, y = 0.0f, w = 0.0f;
            int n = sscanf(val, "%f,%f,%f", &x, &y, &w);
            if (n >= 2)
            {
                blue_joy_x_norm = Blue_Clamp(x, -1.0f, 1.0f);
                blue_joy_y_norm = Blue_Clamp(y, -1.0f, 1.0f);
                blue_joy_w_norm = (n >= 3) ? Blue_Clamp(w, -1.0f, 1.0f) : 0.0f;
                Blue_Update_From_Normalized_Joy();
            }
            return;
        }

        if (strcmp(key, "位置环开关") == 0 || strcmp(key, "pos_loop") == 0 || strcmp(key, "sw_pos") == 0)
        {
            Blue_Set_Position_Loop((atof(val) > 0.5f));
            return;
        }

        if (strcmp(key, "角度控制") == 0 || strcmp(key, "yaw_ctrl") == 0 || strcmp(key, "sw_yaw") == 0)
        {
            Blue_Set_Yaw_Ctrl((atof(val) > 0.5f));
            return;
        }

        if (strcmp(key, "angle") == 0 || strcmp(key, "yaw") == 0 || strcmp(key, "目标角") == 0)
        {
            blue_ctrl.yaw = (fp32)atof(val);
            if (blue_yaw_ctrl_enable)
            {
                blue_ctrl.yaw_hold = true;
            }
            return;
        }
    }

    // 单字符命令（方向键式遥控）
    if (line[1] == '\0')
    {
        fp32 speed = 0.3f;   // 默认速度 0.3 m/s
        fp32 turn = 0.5f;    // 默认转向速度 0.5 rad/s

        switch (line[0])
        {
            case 'W': case 'w':  // 前进（vx负 = 实际前进）
                blue_ctrl.vx = -speed;
                blue_ctrl.vy = 0;
                blue_ctrl.wz = 0;
                break;
            case 'S': case 's':  // 后退
                blue_ctrl.vx = speed;
                blue_ctrl.vy = 0;
                blue_ctrl.wz = 0;
                break;
            case 'A': case 'a':  // 左移（vy负 = 实际左移）
                blue_ctrl.vx = 0;
                blue_ctrl.vy = -speed;
                blue_ctrl.wz = 0;
                break;
            case 'D': case 'd':  // 右移（vy正 = 实际右移）
                blue_ctrl.vx = 0;
                blue_ctrl.vy = speed;
                blue_ctrl.wz = 0;
                break;
            case 'Q': case 'q':  // 左转
                blue_ctrl.vx = 0;
                blue_ctrl.vy = 0;
                blue_ctrl.wz = turn;
                break;
            case 'E': case 'e':  // 右转
                blue_ctrl.vx = 0;
                blue_ctrl.vy = 0;
                blue_ctrl.wz = -turn;
                break;
            case 'X': case 'x':  // 停止
            case ' ':
                blue_ctrl.vx = 0;
                blue_ctrl.vy = 0;
                blue_ctrl.wz = 0;
                break;
            default:
                break;
        }
        blue_ctrl.world_frame = true;
        blue_ctrl.yaw_hold = false;
        return;
    }

    // 数值格式解析: "vx,vy,wz"
    // 例如: "0.3,0.2,0.1"
    fp32 vx = 0, vy = 0, wz = 0;
    
    int parsed = 0;
    char *token = line;
    char *next = NULL;
    
    // 解析 vx
    next = strchr(token, ',');
    if (next) { *next = '\0'; vx = (fp32)atof(token); token = next + 1; parsed++; }
    // 解析 vy
    next = strchr(token, ',');
    if (next) { *next = '\0'; vy = (fp32)atof(token); token = next + 1; parsed++; }
    // 解析 wz
    if (parsed == 2) { wz = (fp32)atof(token); parsed++; }
    
    if (parsed == 3)
    {
        blue_ctrl.vx = vx;
        blue_ctrl.vy = vy;
        blue_ctrl.wz = wz;
        blue_ctrl.world_frame = true;
        blue_ctrl.yaw_hold = false;
    }
}

static char* Blue_Trim(char *s)
{
    while (*s == ' ' || *s == '\t')
    {
        s++;
    }

    char *end = s + strlen(s);
    while (end > s && (end[-1] == ' ' || end[-1] == '\t'))
    {
        end--;
    }
    *end = '\0';

    return s;
}

static fp32 Blue_Clamp(fp32 v, fp32 min_v, fp32 max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static void Blue_Update_From_Normalized_Joy(void)
{
    // 归一化摇杆映射:
    // x轴(右正) -> 横移vy（左负右正）
    // y轴(上正) -> 前后vx（前进为负）
    blue_ctrl.vx = -blue_joy_y_norm * BLUE_JOY_MAX_SPEED_MPS;
    blue_ctrl.vy =  blue_joy_x_norm * BLUE_JOY_MAX_SPEED_MPS;
    blue_ctrl.wz =  blue_joy_w_norm * BLUE_JOY_MAX_WZ_RAD_S;

    blue_ctrl.world_frame = true;
    blue_ctrl.yaw_hold = blue_yaw_ctrl_enable;
}

static void Blue_Set_Position_Loop(bool enable)
{
    if (enable)
    {
        const chassis_odometry_t *odom = get_behaviour_data();
        if (odom != NULL)
        {
            blue_pos_origin_x = odom->position_x;
            blue_pos_origin_y = odom->position_y;
        }
        blue_pos_loop_enable = true;
        behaviour_enable(1);
    }
    else
    {
        blue_pos_loop_enable = false;
        behaviour_disable();
    }
}

static void Blue_Set_Yaw_Ctrl(bool enable)
{
    blue_yaw_ctrl_enable = enable;

    if (enable)
    {
        const chassis_move_t *chassis = get_chassis_move_data();
        if (chassis != NULL)
        {
            blue_ctrl.yaw = chassis->chassis_yaw;
        }
        blue_ctrl.yaw_hold = true;
    }
    else
    {
        blue_ctrl.yaw_hold = false;
    }
}

/**
 * @brief 计算校验和（异或校验）
 */
static uint8_t Blue_Checksum(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        sum ^= data[i];
    }
    return sum;
}


