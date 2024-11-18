/*
 * led.c
 *
 *  Created on: 2022��7��7��
 *      Author: Lenovo
 */

#include "bsp_led.h"




void bsp_led_init(void)
{
//    GPIO_SetupPinMux(RUN_LED, GPIO_MUX_CPU1, 0x0);
//	GPIO_SetupPinOptions(RUN_LED, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//    GPIO_SetupPinMux(FLT_LED, GPIO_MUX_CPU1, 0x0);
//	GPIO_SetupPinOptions(FLT_LED, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//    GPIO_SetupPinMux(DCBUS_LED, GPIO_MUX_CPU1, 0x0);
//	GPIO_SetupPinOptions(DCBUS_LED, GPIO_OUTPUT, GPIO_PUSHPULL);
//板上LED
    GPIO_SetupPinMux(LED2, GPIO_MUX_CPU1, 0x0);
	GPIO_SetupPinOptions(LED2, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(LED3, GPIO_MUX_CPU1, 0x0);
	GPIO_SetupPinOptions(LED3, GPIO_OUTPUT, GPIO_PUSHPULL);

	//PWM缓冲器使能GPIO
    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 0x0);
    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_WritePin(LED2, 1);
    GPIO_WritePin(LED3, 1);

}

void GPIO_TogglePin(Uint16 gpioNumber)
{
    static Uint16 outVal_led2;
    static Uint16 outVal_led3;
    if(gpioNumber == LED2) {
        GPIO_WritePin(gpioNumber, outVal_led2);
        outVal_led2 = !outVal_led2;
    }
    if(gpioNumber == LED3) {
        GPIO_WritePin(gpioNumber, outVal_led3);
        outVal_led3 = !outVal_led3;
    }
}

