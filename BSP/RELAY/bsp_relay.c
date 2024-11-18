#include "bsp_relay.h"


void bsp_relay_init(void)
{
    //继电器1初始化
    GPIO_SetupPinMux(RELAY_1, GPIO_MUX_CPU1, 0x0);
    GPIO_SetupPinOptions(RELAY_1, GPIO_OUTPUT, GPIO_PUSHPULL);
    //继电器2初始化
    GPIO_SetupPinMux(RELAY_2, GPIO_MUX_CPU1, 0x0);
    GPIO_SetupPinOptions(RELAY_2, GPIO_OUTPUT, GPIO_PUSHPULL);

    //初始保持关闭
   RELAY_1_OFF();
//    RELAY_1_ON();
    RELAY_2_OFF();

//    RELAY_1_ON();
//    RELAY_2_ON();


}
