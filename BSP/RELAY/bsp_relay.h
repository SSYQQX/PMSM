#ifndef BSP_RELAY_H_
#define BSP_RELAY_H_

#include "F28x_Project.h"

void bsp_relay_init(void);

#define RELAY_1            70//板上
#define RELAY_2            69//板上

#define RELAY_1_ON()         GPIO_WritePin(RELAY_1, 1)
#define RELAY_2_ON()         GPIO_WritePin(RELAY_2, 1)


#define RELAY_1_OFF()         GPIO_WritePin(RELAY_1, 0)
#define RELAY_2_OFF()         GPIO_WritePin(RELAY_2, 0)

#endif /* BSP_RELAY_H_ */
