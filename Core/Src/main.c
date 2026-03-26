/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_buzzer_pwm.h"
#include <stdint.h>
#include <stdio.h>
#include "bsp_music.h"
#include "bsp_led.h"
#include "bsp_delay.h"
#include "bsp_key.h"
#include "bsp_servo_pwm.h"
#include "struct_typedef.h"
#include "gm65.h"
#include "bsp_encoder.h"
#include "bsp_usart.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "dc_motor.h"
#include "OLED.h"
#include "test_task.h"
#include <string.h>
#include "rc_ibus.h"
#include "mission_fsm.h"
#include "blue_control.h"
#include "trajectory_planner.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char msg[256] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void chassis_data_send(UART_HandleTypeDef *huart)// uart句柄
{
    static uint32_t last_send_tick = 0;
    uint32_t now_tick = HAL_GetTick();

    if (huart == NULL)
    {
      return;
    }

    // 10Hz发送，避免占满串口带宽
    if ((now_tick - last_send_tick) < 100U)
    {
      return;
    }
    last_send_tick = now_tick;

    const chassis_odometry_t *behaviour = get_behaviour_data();
    const chassis_move_t *chassis = get_chassis_move_data();

    if ((behaviour == NULL) || (chassis == NULL))
    {
      return;
    }

    int len = snprintf(msg, sizeof(msg),
                       "chassis:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                       behaviour->position_x,      // ch0: 里程计X
                       behaviour->position_y,      // ch1: 里程计Y
                       behaviour->position_x_set,  // ch2: 目标X
                       behaviour->position_y_set,  // ch3: 目标Y
                       behaviour->yaw_set,         // ch4: 外环目标角度(yaw)
                       chassis->chassis_yaw,       // ch5: 当前底盘航向角(yaw)
                       chassis->vx,                // ch6: 底盘X轴速度(vx)
                       chassis->vy,                // ch7: 底盘Y轴速度(vy)
                       chassis->wz);               // ch8: 底盘角速度(wz)

    if ((len > 0) && (huart->gState == HAL_UART_STATE_READY))
    {
      HAL_UART_Transmit_DMA(huart, (uint8_t *)msg, (uint16_t)len);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */

  delay_ms(2000);
  can_filter_init(); // 初始化 CAN 过滤器
  delay_ms(1);
  buzzer_init(); // 初始化蜂鸣器 PWM
  bsp_servo_pwm_init(); // 初始化云台和夹爪舵机 PWM
  dc_motor_pwm_init(); // 初始化电机 PWM
  HAL_TIM_Base_Start_IT(&htim7); // 启动 TIM7 中断,用于周期任务调度,1ms

  uart_init_dma(&huart4); // 初始化 UART4 DMA 接收（空闲模式）
  uart_init_dma(&huart1); // 初始化 UART1 DMA 接收
  uart_init_it(&huart5); // 初始化 UART 接收中断
  Blue_Control_Init(&huart3);

  // UART/DMA 初始化后留 10ms 稳定时间
  delay_ms(10);
  
  global_behaviour_init(); // 全局行为初始化
  global_chassis_task_init(); // 底盘初始化
  bsp_encoder_init(); // 初始化编码器

  OLED_Init();
  Mission_FSM_Init();
  //RC_IBUS_Init(&rc_ibus, &huart3, 0.06f, 300);
  //global_trajectory_planner_init(); // 轨迹规划器初始化

  
  delay_ms(1500); // 等待任务开始后系统稳定
  bsp_led_on(CORE_ONE);
 
  // 启动任务
  //Mission_Start(pickup_mission, sizeof(pickup_mission)/sizeof(pickup_mission[0]));

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //Key_Handle_Loop(); // 按键处理循环
    //process_data(&huart5); // 处理gm65接收到的数据
    //GPIOD->BSRR = GPIO_PIN_7; 
    //behaviour_task_cycle(); // 底盘行为控制
    //chassis_task_cycle(); // 底盘速度控制
    //GPIOD->BRR = GPIO_PIN_7; 
    //chassis_data_send(&huart3); // 发送底盘状态数据

    //Blue_Control_Process();
    //pwm_cmd_chassis(9, 9, 9, 9);
    //delay_ms(1000);
    //pwm_cmd_chassis(30, 30, 30, 30);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
