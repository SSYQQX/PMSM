

#include "bsp_timer.h"
#include "cputimer.h"

extern __interrupt void Timer_isr(void);

void bsp_timer_init(void)
{
    EALLOW;
    PieVectTable.TIMER0_INT = &Timer_isr;
    EDIS;

    InitCpuTimers();

    ConfigCpuTimer(&CpuTimer0, 200, 50);/* 周期50us  20KHz */
    CpuTimer0Regs.TCR.all = 0x4001;
    // Enable CPU INT1 which is connected to CPU-Timer 0:
    IER |= M_INT1;
    // Enable TINT0 in the PIE: Group 1 __interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;


    EINT;
    ERTM;
}






