/*
 * bsp_gpio.h
 *
 *  Created on: 2024年12月3日
 *      Author: XUQQ
 */

#ifndef BSP_GPIO_BSP_GPIO_H_
#define BSP_GPIO_BSP_GPIO_H_

#include "F28x_Project.h"
extern Uint16 CHARGE_FLAG;//电池充电控制标志
void Battery_Charge_GpioInit(void);
void Battery_Charge_EN(Uint16 charge_flag);
#endif /* BSP_GPIO_BSP_GPIO_H_ */
