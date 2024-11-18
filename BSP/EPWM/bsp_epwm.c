#include "bsp_epwm.h"

Uint16 EPWM1_TBPRD=2500;//20khZ
Uint16 EPWM1_CMPA=1250;
Uint16 EPWM1_CMPB=1250;
Uint16 EPWM2_TBPRD=2500;
Uint16 EPWM2_CMPA=1250;
Uint16 EPWM2_CMPB=1250;
Uint16 EPWM3_TBPRD=2500;
Uint16 EPWM3_CMPA=1250;
Uint16 EPWM3_CMPB=1250;
Uint16 DEADTIME=60;

void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
void TZProtect_Int(void);
void bsp_epwm_init(void)
{
    // enable PWM1, PWM2 and PWM3
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM3=1;

    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();

    //配置EPWM
        EALLOW;
        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;//时钟先开后关，保持EPWM时钟同步
        EDIS;
    //    初始化寄存器
        InitEPwm1Example();
        InitEPwm2Example();
        InitEPwm3Example();

        ///TZ模块配置
        TZProtect_Int();
//开时钟
        EALLOW;
        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
        EDIS;


}

//系统时钟200M,周期5ns
//EPWM时钟频率是系统时钟频率的一半，为100M，周期10ns
void InitEPwm1Example()
{
    //
    // Setup TBCLK
    //
    EPwm1Regs.TBPRD =EPWM1_TBPRD;       // Set timer period 801 TBCLKs，1000分频，开关频率100K
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter

    //
    // Set Compare values
    //
    EPwm1Regs.CMPA.bit.CMPA = EPWM1_CMPA;    // Set compare A value
    EPwm1Regs.CMPB.bit.CMPB = EPWM1_CMPA;    // Set Compare B value

    //
    // Setup counter mode
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;//过零时发同步信号


    // Setup shadowing
    //
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero，过零时装载
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM1A on event A, up
                                                  // count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;          // Clear PWM1A on event A,
                                                  // down count

    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;            // Set PWM1B on event B, up
                                                  // count
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // Clear PWM1B on event B,
                                                  // down count
    // Active Low PWMs - Setup Deadband
     //
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;//AB均不需要延时
     EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;//极性控制，决定是否在信号输出前，对经过延时的信号进行取反操作
     EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;//边缘控制的触发信号源
     EPwm1Regs.DBRED.bit.DBRED = DEADTIME;//上升延时
     EPwm1Regs.DBFED.bit.DBFED = DEADTIME;//下降延时

    // Interrupt where we will change the Compare Values
    //EPWM中断
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 0;                // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_DISABLE; //不中断          // Generate INT on 1 event

    //EPWM触发ADC采样
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;            // ENable SOC on A group  ,A 组SOC失能
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;            // 过零点触发ADC采样
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;  //1           // Generate pulse on 1st event,第一个事件产生脉冲

}

//
// InitEPwm2Example - Initialize EPWM2 configuration
//

//系统时钟200M,周期5ns
//EPWM时钟频率是系统时钟频率的一半，为100M，周期10ns
void InitEPwm2Example()
{
    //
    // Setup TBCLK
    //
    EPwm2Regs.TBPRD = EPWM2_TBPRD;         // Set timer period 801 TBCLKs
    EPwm2Regs.TBPHS.bit.TBPHS = 0;          // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                    // Clear counter

    //
    // Set Compare values
    //
    EPwm2Regs.CMPA.bit.CMPA = EPWM2_CMPA;    // Set compare A value
    EPwm2Regs.CMPB.bit.CMPB = EPWM2_CMPB;    // Set Compare B value

    //
    // Setup counter mode
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading

    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
   // EPwm2Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_IN;//EPWM1过零时同步,装载TBPHS的值
    EPwm2Regs.TBCTL.bit.PHSDIR=1;//指定同步后的计数方向，0：同步时减计数，1:同步时增计数
    //
    // Setup shadowing
    //
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero  0时刻装载比较值
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;//三角波大于调制波2A输出高电平

    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;


    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;//三角波大于调制波2B输出低电平

    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    // Active Low PWMs - Setup Deadband
     //
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;//AB均需要延时
     EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;//极性控制，决定是否在信号输出前，对经过延时的信号进行取反操作
     EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;//边缘控制的触发信号源，A作为上升下降沿延时的信号源，A上升沿延时后输出，B由A下降沿延时后，再取反得到。
     EPwm2Regs.DBRED.bit.DBRED = DEADTIME;//上升延时
     EPwm2Regs.DBFED.bit.DBFED = DEADTIME;//下降延时
    //
    // Interrupt where we will change the Compare Values
    //EPWM2中断
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 0; //当前失能              // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 1 event
//触发ADC采样
    EPwm2Regs.ETSEL.bit.SOCAEN  = 0; //当前失能           // ENable SOC on A group  ,A 组SOC失能
    EPwm2Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;            // Select SOC on up-count,选择SOC向上计数
    EPwm2Regs.ETPS.bit.SOCAPRD = ET_1ST;  //1           // Generate pulse on 1st event,第一个事件产生脉冲
    //
    // Information this example uses to keep track
    // of the direction the CMPA/CMPB values are
    // moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
//    epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;  // Start by increasing CMPA
//    epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_UP;  // & increasing CMPB
//    epwm2_info.EPwmTimerIntCount = 0;              // Zero the interrupt counter
//    epwm2_info.EPwmRegHandle = &EPwm2Regs;         // Set the pointer to the
//                                                   // ePWM module
//    epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;       // Setup min/max CMPA/CMPB
//                                                   // values
//    epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
//    epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
//    epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;
}

//
// InitEPwm3Example - Initialize EPWM3 configuration
//
void InitEPwm3Example(void)
{
    //
    // Setup TBCLK
    //
    EPwm3Regs.TBPRD = EPWM3_TBPRD;       // Set timer period 801 TBCLKs，1000分频，开关频率100K
    EPwm3Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                  // Clear counter

    //
    // Set Compare values
    //
    EPwm3Regs.CMPA.bit.CMPA = EPWM3_CMPA;    // Set compare A value
    EPwm3Regs.CMPB.bit.CMPB = EPWM3_CMPB;    // Set Compare B value

    //
    // Setup counter mode
    //
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;


    // Setup shadowing
    //
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero，过零时装载
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set actions
    ////三角波大于调制波3A输出高电平
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM1A on event A, up
                                                  // count
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;          // Clear PWM1A on event A,
                                                  // down count
    ////三角波大于调制波3B输出低电平
    EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;            // Set PWM1B on event B, up
                                                  // count
    EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;          // Clear PWM1B on event B,
                                                  // down count
    // Active Low PWMs - Setup Deadband
     //
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;//AB均需要延时
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;//极性控制，决定是否在信号输出前，对经过延时的信号进行取反操作
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;//边缘控制的触发信号源
    EPwm3Regs.DBRED.bit.DBRED = DEADTIME;//上升延时
    EPwm3Regs.DBFED.bit.DBFED = DEADTIME;//下降延时

    // Interrupt where we will change the Compare Values
    //EPWM中断
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm3Regs.ETSEL.bit.INTEN = 0;                // Enable INT，当前失能
    EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1 event
}

void TZProtect_Int(void)
{
    EALLOW;
    //////触发信号GPIO配置
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0; // 将GPIO19设置为上拉模式
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // 设置输入量化，对外部输入信号进行采样识别
    InputXbarRegs.INPUT1SELECT = 19;//INPUT1与TZ1相连
    //One-Shot模式
    EPwm1Regs.TZSEL.bit.OSHT1 = 1;//TZ1,触发源
    // Set TZA     Trip Zone Action  错误联防动作
    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;//当TZ事件触发时，强制将EPWMxA拉低
    EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;//当TZ事件触发时，强制将EPWMxB拉低
    // Enable TZ interrupt
    //---------OSHT触发事件中断--------//
//中断清除寄存器
    EPwm1Regs.TZCLR.bit.INT = 1;  //for tz trip int clear;清楚全局中断
    EPwm1Regs.TZCLR.bit.OST = 1;  //for tz trip OSHT event clear;清楚单次触发标志
    EPwm1Regs.TZEINT.bit.OST = 1;//使能中断


    //One-Shot模式
    EPwm2Regs.TZSEL.bit.OSHT1 = 1;//TZ1
    // Set TZA     Trip Zone Action  错误联防动作
    EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;//当TZ事件触发时，强制将EPWMxA拉低
    EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;//当TZ事件触发时，强制将EPWMxB拉低
    EPwm2Regs.TZCLR.bit.INT = 1;  //for tz trip int clear;清楚全局中断
    EPwm2Regs.TZCLR.bit.OST = 1;  //for tz trip OSHT event clear;清楚单次触发标志
    EPwm2Regs.TZEINT.bit.OST = 0;//使能中断
    //One-Shot模式
    EPwm3Regs.TZSEL.bit.OSHT1 = 1;//TZ1
    // Set TZA     Trip Zone Action  错误联防动作
    EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO;//当TZ事件触发时，强制将EPWMxA拉低
    EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO;//当TZ事件触发时，强制将EPWMxB拉低
    EPwm3Regs.TZCLR.bit.INT = 1;  //for tz trip int clear;清楚全局中断
    EPwm3Regs.TZCLR.bit.OST = 1;  //for tz trip OSHT event clear;清楚单次触发标志
    EPwm3Regs.TZEINT.bit.OST = 0;//使能中断



    //对应的PIE中断使能
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1; // EPWM1_TZ_INT
    IER |= M_INT2;// CPU Interrupt Enable PIE Group 2

    EDIS;
}
