#ifndef BSP_LED_H
#define BSP_LED_H

#include "gpio.h"

/* LED 引脚定义 */
#define CORE_ONE GPIO_PIN_15
#define CORE_TWO GPIO_PIN_0
#define CORE_THREE GPIO_PIN_1

/*仅限GPIOC端口*/
extern void LED_Init(void);
extern void bsp_led_on(uint16_t GPIO_Pin);  
extern void bsp_led_off(uint16_t GPIO_Pin);
extern void bsp_led_toggle(uint16_t GPIO_Pin);

#endif
