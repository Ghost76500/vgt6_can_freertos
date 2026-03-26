#ifndef BSP_BUZZER_PWM_H
#define BSP_BUZZER_PWM_H

#include "stdint.h"

extern void buzzer_init(void);
extern void Buzzer_Control(uint16_t frequency, uint8_t volume);
extern void gimbal_warn_buzzer_on(void);
extern void gimbal_warn_buzzer_off(void);
extern void chassis_warn_buzzer_on(void);   
extern void chassis_warn_buzzer_off(void);


#endif