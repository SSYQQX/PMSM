/*
 * led.h
 *
 *  Created on: 2022��7��7��
 *      Author: Lenovo
 */

#ifndef BSP_LED_LED_H_
#define BSP_LED_LED_H_

#include "F28x_Project.h"

void GPIO_TogglePin(Uint16 gpioNumber);
void bsp_led_init(void);

#define LED2            89//板上
#define LED3            90//板上


#define LED2_ON()         GPIO_WritePin(LED2, 0)
#define LED3_ON()         GPIO_WritePin(LED3, 0)


#define LED2_OFF()         GPIO_WritePin(LED2, 1)
#define LED3_OFF()         GPIO_WritePin(LED3, 1)

#define LED2_TOGGLE()      GPIO_TogglePin(LED2)
#define LED3_TOGGLE()      GPIO_TogglePin(LED3)



#endif /* BSP_LED_LED_H_ */
