#include "uart_rx_task.h"
#include "cmsis_os2.h"
#include "command.h"

extern osMessageQueueId_t uart1_queueHandle;

void uart_rx_task(void *argument)
{
    osDelay(100); // 任务启动延时，等待系统稳定
    uint8_t command[50];
    int commandLength = 0;
    uint32_t received_msg; // 用来存放从队列里取出来的消息
    float rx_yaw_deg = 0.0f;
    float rx_yaw_rad = 0.0f;
    
    for (;;)
    {
        if (osMessageQueueGet(uart1_queueHandle, &received_msg, NULL, osWaitForever) == osOK) 
        {
          // 2. 程序能走到这里，说明中断发消息过来了，任务被唤醒了！
          // 既然唤醒了，说明你的环形缓冲区里必定有新数据，立刻全速去掏缓冲区
          while ((commandLength = Command_GetCommand(command)) > 0) 
          {
              // 3. 如果成功提取出 11 字节的完整数据包，进行解码
              if (commandLength == 11) 
              {
                  int16_t yaw_raw = (int16_t)(((uint16_t)command[7] << 8) | command[6]);
                  rx_yaw_deg = (float)yaw_raw * 180.0f / 32768.0f;
                  rx_yaw_rad = rx_yaw_deg * 0.0174532925f;
              }
          }
        }
    }
}