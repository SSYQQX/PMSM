


#include "bsp_can.h"
#include "can.h"

#define CANTXB_GPIO     20
#define CANRXB_GPIO     21


void bsp_can_init(void)
{
    GPIO_SetupPinMux(CANTXB_GPIO, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(CANTXB_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(CANRXB_GPIO, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(CANRXB_GPIO, GPIO_INPUT, GPIO_ASYNC);

    CANInit(CANB_BASE);
    CANClkSourceSelect(CANB_BASE, 0);
    CANBitRateSet(CANB_BASE, 200000000, 500000);/* 波特率500k */
    CANIntEnable(CANB_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    CANEnable(CANB_BASE);
    //CANGlobalIntEnable(CANB_BASE, CAN_GLB_INT_CANINT0);

    // CAN_initModule(CANB_BASE);
    // CAN_selectClockSource(CANB_BASE, 0);
    // CAN_enableController(CANB_BASE);
}




