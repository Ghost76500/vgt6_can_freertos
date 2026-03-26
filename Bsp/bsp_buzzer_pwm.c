#include "bsp_buzzer_pwm.h"
#include "tim.h"

void buzzer_init(void)
{
    // 假设使用 TIM14 的通道 1 作为蜂鸣器 PWM 输出
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
    
    // 初始设置为静音
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
}

/* * 该函数用于设置蜂鸣器的频率和音量
 * * 参数:
 * frequency: 声音频率 (Hz)，比如 2000 代表 2kHz
 * volume:    音量 (0 - 100)，0为静音，100为最大音量
 */
void Buzzer_Control(uint16_t frequency, uint8_t volume)
{
    if (frequency == 0 || volume == 0)
    {
        // 静音处理
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0); 
        return;
    }

    // 计算 ARR (自动重装载值)
    // PSC 是 71，时钟源 72MHz，计数频率是 1MHz
    uint32_t arr_value = (1000000 / frequency) - 1;

    // 修改自动重装载寄存器的值
    __HAL_TIM_SET_AUTORELOAD(&htim14, arr_value);

    // 计算 CCR (占空比)
    float duty_ratio = (float)volume / 200.0f; 
    uint32_t ccr_value = (uint32_t)((arr_value + 1) * duty_ratio);

    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, ccr_value);
}


void gimbal_warn_buzzer_on(void)
{
    Buzzer_Control(2000, 50); // 2kHz, 50% 音量
}

void gimbal_warn_buzzer_off(void)
{
    Buzzer_Control(0, 0); // 静音
}

void chassis_warn_buzzer_on(void)
{
    Buzzer_Control(1500, 50); // 2kHz, 50% 音量
}

void chassis_warn_buzzer_off(void)
{
    Buzzer_Control(0, 0); // 静音
}