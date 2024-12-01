//#include "device.h"
//永磁同步电机控制  200W发电机
#include "F28x_Project.h"
#include<stdlib.h>
#include "interrupt.h"
#include "bsp_emif.h"
#include "bsp_led.h"
#include "bsp_timer.h"
#include "bsp_epwm.h"
#include "sysctl.h"
#include "emif.h"
#include "bsp_adc.h"
#include "bsp_relay.h"
#include "bsp_pid_ctrl.h"
#include "SPLL_1ph.h"
#include "Solar_F.h"
#include "spll_1ph_sogi_fll.h"

#include "F2837xD_Examples.h"
#include "bsp_I2C.h"
#include "485_ModobusRTU.h"

#define PI     3.14159265f
#define ld     103.85e-6
//#define lq      61e-6   表贴式Ld=Lq
#define lq     95.78e-6
/**
 * main.c
 */

Uint16 APhase_Current=0;
Uint16 BPhase_Current=0;
Uint16 CPhase_Current=0;
Uint16 flag=0;
Uint16 flag2=0;

Uint16 Bus_Voltage_AD=0;
Uint16 Bus_Current_AD=0;
Uint16 Supercapacitor_Voltage_AD=0;

float32 K_norm=0.05;


float32 Ua_Voltage=0;
float32 Ub_Voltage=0;
float32 Uc_Voltage=0;
float32 Ud_Voltage=0;
float32 Uq_Voltage=0;

float32 Ia_Current=0;
float32 Ib_Current=0;
float32 Ic_Current=0;
float32 Id_Current=0;
float32 Iq_Current=0;

float32 Ia_Current_norm=0;//归一化
float32 Ib_Current_norm=0;
float32 Ic_Current_norm=0;

float32 Bus_Voltage=0;
float32 Bus_Current=0;
float32 Supercapacitor_Voltage=0;

//系统上电、下电控制
int Turn_on_off=0;//上电，下电标志
int state_flag=0;//控制器状态
int zhuanziDw_flag=0;//转子定位标志 =1需要进行定位。=0不需要

int RELAY2_flag=0;//继电器使能
int RELAY1_flag=0;
int RELAY2_flag_last=0;//继电器使能
int RELAY1_flag_last=0;

int PEM_ENABLE=1;//pwm输出使能。0输出，1关断。

float32 Vac[1000]={0};
float32 iSin[400]={0};

float M=0.95;//调制比

float32 Sin_the=0;//A相相位
float32 Cos_the=0;
//PID控制
//内环
PID_CTRL current_pid_d;/* 电流控制 */
PID_CTRL current_pid_q;
float Id_pid_kp=ld*5543.22;//
float Iq_pid_kp=lq*5543.22;//
float Id_pid_ki=0.0002342;//0.0001
float Iq_pid_ki=0.0002342;//0.0001
float current_out_limit=150;//内环电流PID输出限幅
//外环,转速环
PID_CTRL speed_pid;/* 转速控制 */
float speed_pid_kp=0.035;//0.035
float speed_pid_ki=0.0000875;//0.001
float speed_out_limit=15;//转速环输出限幅

//直流电流环
PID_CTRL currentDC_pid;
float currentDC_pid_kp=0;
float currentDC_pid_ki=0;
float currentDC_out_limit=0;
//直流电流给定
float currentDC_ref=2.5;
float currentDC_ref_ctr=2.5;
//给定
float speed_ref=0;   //转速参考
float speed_ref_ctr=0;
float32 speed_normK=150;//调制前归一化系数
//电机参数
float32 Ld=103.85e-6;
float32 Lq=95.78e-6;
float32 Rs=0.0845;//定子电阻
float Pn=5;   //极对数
float32 flux=0.0057135;//磁链 flux=0.0057135
float32 Kt=0.042851;//转矩常数，Te=Kt*iq。
float32 Te=0;//转矩
//耦合项
float32 w1_d=0;
float32 w1_q=0;

float32 Id_out=0;
float32 Iq_out=0;
float32 Id_out_norm=0;
float32 Iq_out_norm=0;
float32 Varef=0;
float32 Vbref=0;
float32 Vcref=0;
__interrupt void adca1_isr(void);
__interrupt void adcd1_isr(void);

__interrupt void cpu_timer1_isr(void);

__interrupt void INTERRUPT_ISR_TZProtect(void);
//按键中断
interrupt void xint1_isr(void);
interrupt void xint2_isr(void);

extern FILTE VBus_filte;
extern FILTE Vab_filte;
extern FILTE temp_filte;


//dq变换
ABC_DQ0_POS_F abc_dq0_pos1_speed;

ABC_DQ0_POS_F abc_dq0_pos1_cur;

DQ0_ABC_F dq0_abc1_cur;

void PID_Parameter_Init(void);

__interrupt void myEQEP1_ISR(void);
void Init_Variables(void);
void Init_EQEP1_Gpio(void);
void Init_EQEP1(void);
void POSSPEED_Calc();

int motorspeed_rpm=0;//




int F_flag=0;
Uint32 led_count=0;
Uint32 led_T=5000000;

struct Motor_Para
{
    int DirectionQep;           //电机旋转方向
    float32 mech_position; //实时机械位置
    int Speed_N;
    int Speed_H;
    int Speed_L;
    unsigned int long Now_position;         //变量：当前位置
    unsigned int long Last_position;           //变量：上一次位置
    float32 mech_scaler;    //一转的总脉冲数的倒数  1/10000
    float32 theta_mech;
    float32 theta_elec;
    float32 w_elec;//电角速度
    int pole_pairs;//极对数
}motor;


float32 SPEED_cal_K=0.0001*1000*60;//转速计算系数=1/10000*编码器采样频率*60秒。

void key_Init(void);//按键中断初始化。

int TZ_flag=3;
int TZ=0;
/////////////////I2C////////////////////
__interrupt void i2c_int1a_isr(void);
int COM_flag=100;//读写设备号
int COM_Allow=0;
int I2C_ERROR_FLAG=0;//I2C故障标志

int zl_flag=0;
int gd_f=0;
int gd_count=1500;
int main(void)
{

	InitSysCtrl();
	InitGpio();

	DINT;
	//关闭ＰＩＥ功能,清空PIEIER 和PIEIFR寄存器；用以清除所有的cpu中断响应组
	InitPieCtrl();
	//关闭CPU中断响应；CPU寄存器中也有两个寄存器用于设置中断
    IER = 0x0000;
    IFR = 0x0000;
    //清空中断向量表，即清空所有的中断地址表
    InitPieVectTable();

    //I2C初始化
    I2CB_GpioInit();//I2C io初始
    I2CB_Init();
    //485
    Modobus_485_GpioInit();
    scia_fifo_init();  // 初始化SCI-A
    //编码器
     Init_Variables();
     Init_EQEP1_Gpio();
     Init_EQEP1();

//    //编码器中断
//    EALLOW;  // This is needed to write to EALLOW protected registers
//    PieVectTable.EQEP1_INT = &myEQEP1_ISR;
//    EDIS;    // This is needed to disable write to EALLOW protected registers
//    PieCtrlRegs.PIEIER5.bit.INTx1 = 1;//EQEP1中断
//    IER |= M_INT5;

	bsp_led_init();//LED
	bsp_relay_init();//继电器
	//bsp_emif_init();
	//按键
	key_Init();
	//pid参数初始化
	PID_Parameter_Init();
///////ADC
	bsp_adc_init();//adc中断配置在其中
///////////PWM
    GPIO_WritePin(12, PEM_ENABLE);//PWM输出使能，低电平有效
	bsp_epwm_init();
	//SPLL_1ph_init(50,(0.00005),&spll1); //锁相环初始化,20kHz

	//I2C中断函数
    EALLOW;    // This is needed to write to EALLOW protected registers
    PieVectTable.I2CB_INT = &i2c_int1a_isr;
    EDIS;      // This is needed to disable write to EALLOW protected registers
    //485中断函数
       EALLOW;  // 允许写入EALLOW保护寄存器
       PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;  // SCI-A接收中断映射到接收ISR
       EDIS;    // 禁用写入EALLOW保护寄存器
    //485 启用中断
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // 启用PIE模块
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE组9，INT1
    IER |= M_INT9;                         // 启用CPU INT中断
    EALLOW;
//    PieVectTable.TIMER0_INT = &cpu_timer0_isr;//地址赋值给 TIMER0_INT 中断向量
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;//地址赋值给 TIMER1_INT 中断向量
    PieVectTable.EPWM1_TZ_INT = &INTERRUPT_ISR_TZProtect;//TZ中断函数地址
    EDIS;
//////////   定时器配置    /////////
    //   初始化CPU定时器0/1/2
    InitCpuTimers();
    //  配置CPU定时器1中断发生时间
    // 200MHz CPU Freq, 1 second Period (in uSeconds)选择定时器，CPU频率，定时器周期（单位us）
    ConfigCpuTimer(&CpuTimer1, 200, 1000000);/*不分频：200*1000000/200*/     //1s
    CpuTimer1Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0表示CPU计时器正在运行。
    //   Enable CPU int13 which is connected to CPU-Timer 1
    IER |= M_INT13;
    // Enable CPU INT8 which is connected to PIE group 8
    IER |= M_INT8;



	//dq变换初始化
	ABC_DQ0_POS_F_init(&abc_dq0_pos1_speed);
    ABC_DQ0_POS_F_init(&abc_dq0_pos1_cur);
    DQ0_ABC_F_init(&dq0_abc1_cur);

    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

	for(;;) {

//	    PWM封锁控制
	    EALLOW;
	    //软件强制触发
        //TZ_flag  PWM封锁控制信号
        if(TZ_flag==0&&EPwm1Regs.TZFLG.bit.OST==1)//关闭封锁，打开PWM
        {
        EPwm1Regs.TZCLR.bit.OST=1;//1:清除触发事件标志
        EPwm2Regs.TZCLR.bit.OST = 1;
        EPwm3Regs.TZCLR.bit.OST = 1;
        EPwm1Regs.TZCLR.bit.INT = 1;    //clear INT flag
        LED3_OFF();
        TZ_flag=3;

        }
        if(TZ_flag==1&&EPwm1Regs.TZFLG.bit.OST==0)//封锁PWM
        {//软件强制封锁
            EPwm1Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            EPwm2Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            EPwm3Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            TZ_flag=3;
        }
	    EDIS;


	   //电容接入控制
	    if(Turn_on_off==0&&state_flag==1)//下电
	    {
	        speed_ref_ctr=0;//关断时速度置零
	        DELAY_US(10);//10us

	        RELAY_1_OFF();//关继电器1
	        DELAY_US(10*1000);//10ms
	        RELAY1_flag=0;
	        RELAY1_flag_last=RELAY1_flag;//更新继电器状态

	        RELAY_2_OFF();//关继电器2
	        RELAY2_flag=0;
	        RELAY2_flag_last=RELAY2_flag;

	        DELAY_US(8000*1000);//8s，关PWM
	        GPIO_WritePin(12, 1);//PWM输出使能，0开1关

	        state_flag=0;//置状态为关。
	        LED3_OFF();//指示灯灭
	    }

        if(Turn_on_off==1&&state_flag==0)//上电
        {
            static int on_flag=0;
            if(on_flag==0)
            {
            speed_ref_ctr=0;//启动时速度置零
            GPIO_WritePin(12, 1);//PWM输出使能，0开1关  关PWM
            DELAY_US(10*1000);//10ms

            RELAY_2_ON();//开继电器2，预充电
            RELAY2_flag=1;
            RELAY2_flag_last=RELAY2_flag;//更新继电器状态

            zhuanziDw_flag=1;//使能转子定位
            PID_Parameter_Init();//pid参数置零，初始化
            DELAY_US(10*1000);//10ms
            GPIO_WritePin(12, 0);//PWM输出使能，0开1关  开PWM
            DELAY_US(1000*1000);//1s
            zhuanziDw_flag=0;//关闭转子定位

            DELAY_US(2000*1000);//2s,等待直流电容充电。
            on_flag++;
            }
            else
            {
                if(fabsf(Supercapacitor_Voltage-Bus_Voltage)<0.1) on_flag++;
                    if(on_flag==50)//判断直流电容是否充电完成。如果充电完成，开继电器1
                    {
                    DELAY_US(500*1000);//200ms
                    RELAY_1_ON();//开继电器1
                    DELAY_US(1*1000);//1ms
                    RELAY1_flag=1;
                    RELAY1_flag_last=RELAY1_flag;//更新继电器状态

                    RELAY_2_OFF();//关继电器2
                    RELAY2_flag=0;
                    RELAY2_flag_last=RELAY2_flag;//更新继电器状态

                    on_flag=0;//开启成功
                    while(motor.Speed_N!=0);
                    state_flag=1;//置状态为开。

                    LED3_ON();//指示灯亮
                    }
            }
        }

	    //继电器控制
	    if(RELAY2_flag!=RELAY2_flag_last||RELAY1_flag!=RELAY1_flag_last)
	    {
	    if(RELAY2_flag==1)//25欧电阻支路
	    RELAY_2_ON();
	    else
	    RELAY_2_OFF();
	    if(RELAY1_flag==1)//电感支路
	    RELAY_1_ON();
	    else
	    RELAY_1_OFF();
	    RELAY2_flag_last=RELAY2_flag;//更新继电器状态
	    RELAY1_flag_last=RELAY1_flag;
	    }
	//if(RELAY2_flag==1&&Bus_Voltage>(Supercapacitor_Voltage-0.3))
	//{
	//    RELAY_1_ON();
	//    RELAY1_flag=1;
	//}
	    //读电池信息
	    if(COM_Allow==2)
	    {//读BMS信息
	        if(I2C_ERROR_FLAG!=0)//I2C故障
	        {
	        I2cbRegs.I2CMDR.bit.IRS=0;//I2C复位
	        DELAY_US(100000);//500us
            I2cbRegs.I2CMDR.bit.IRS=1;//I2C使能
	        }
	    Read_BMS_Information(COM_flag);
	    }

	    if(slave_addr_Rerr!=0||receive_Rerr!=0)
	    {
        //485通信接受错误处理，恢复
        Modobus_485_ReceiveErr_handle();
	    }

	}
}

void PID_Parameter_Init(void)
{
    //转速外环参数
      bsp_pid_init(&speed_pid);
      speed_pid.Kp=speed_pid_kp;
      speed_pid.Ki=speed_pid_ki;
      speed_pid.PIDmax=speed_out_limit;
      speed_pid.PIDmin=-speed_out_limit;
      speed_pid.Imax=speed_out_limit;
      speed_pid.Imin=-speed_out_limit;
      //电流内环参数
      bsp_pid_init(&current_pid_d);
      current_pid_d.Kp=Ld*5543.22;
      current_pid_d.Ki=Id_pid_ki;
      current_pid_d.PIDmax=current_out_limit;
      current_pid_d.PIDmin=-current_out_limit;
      current_pid_d.Imax=current_out_limit;
      current_pid_d.Imin=-current_out_limit;

      bsp_pid_init(&current_pid_q);
      current_pid_q.Kp=Lq*5543.22;
      current_pid_q.Ki=Iq_pid_ki;
      current_pid_q.PIDmax=current_out_limit;
      current_pid_q.PIDmin=-current_out_limit;
      current_pid_q.Imax=current_out_limit;
      current_pid_q.Imin=-current_out_limit;
}


__interrupt void adca1_isr(void)
{

    if(state_flag==1)//开启状态
    {
//        if(speed_ref_ctr==0&&zl_flag<100)//检测到转矩电流，开机
//          {
//            //zl_flag++;
//            if(Iq_Current>1.5) zl_flag++;
//            else zl_flag=0;
//
//            if(zl_flag==100)
//            {
//            speed_ref_ctr=1000;//给定转速
//            }
//          }
//        //zl_flag>=20&&
//        if((motor.Speed_N<-900) && (speed_ref_ctr>900))//检测到电流连续小于0达gd_count次，则关断
//        {
//            if(Iq_Current<0) zl_flag++;
//            else zl_flag=100;
//
//            if(zl_flag==gd_count)
//            {
//              if(abs(speed_ref_ctr+motor.Speed_N)<50)//处于稳态时
//                speed_ref_ctr=0;//给定转速
//                zl_flag=100;
//                gd_f++;//记录关断次数
//            }
//        }
//
//        if(motor.Speed_N==0&&Iq_Current<0) //如果转速过低，电流为转动，则置为初始状态
//        {
//           // GPIO_WritePin(12, 1);//关
//            speed_ref_ctr=0;//给定转速
//            zl_flag=0;
//        }

    //电流采样
    APhase_Current=AdcaResultRegs.ADCRESULT0;//y = 0.015x - 31.077
    BPhase_Current=AdcbResultRegs.ADCRESULT0;//y = 0.0146x - 30.119
    CPhase_Current=AdccResultRegs.ADCRESULT0;//y = 0.0154x - 31.893
//计算实际值
    Ia_Current=0.015*APhase_Current - 31.077;//A相电流
    Ib_Current=0.0146*BPhase_Current - 30.119;//B相电流
    Ic_Current=0.0154*CPhase_Current - 31.893;//C相电流
//归一化
    Ia_Current_norm=Ia_Current*K_norm;
    if(Ia_Current_norm>1.0)Ia_Current_norm=1.0;
    if(Ia_Current_norm<-1.0)Ia_Current_norm=-1.0;

    Ib_Current_norm=Ib_Current*K_norm;
    if(Ib_Current_norm>1.0)Ib_Current_norm=1.0;
    if(Ib_Current_norm<-1.0)Ib_Current_norm=-1.0;

    Ic_Current_norm=Ic_Current*K_norm;
    if(Ic_Current_norm>1.0)Ic_Current_norm=1.0;
    if(Ic_Current_norm<-1.0)Ic_Current_norm=-1.0;


    //编码器解码
    POSSPEED_Calc();
    //////电角度
    Sin_the=sinf(motor.theta_elec);
    Cos_the=cosf(motor.theta_elec);
    //dq变换，输入为cos形式，需要归一化（-1,1）//基于余弦的变换
    //交流电流DQ
    abc_dq0_pos1_cur.a = -Ia_Current_norm;//注意电流方向！！！传感器方向问题
    abc_dq0_pos1_cur.b = -Ib_Current_norm;
    abc_dq0_pos1_cur.c = -Ic_Current_norm;
    abc_dq0_pos1_cur.sin = Sin_the;
    abc_dq0_pos1_cur.cos = Cos_the;
    ABC_DQ0_POS_F_FUNC(&abc_dq0_pos1_cur);

    Id_Current=abc_dq0_pos1_cur.d/K_norm;//与A轴重合。反归一化
    Iq_Current=abc_dq0_pos1_cur.q/K_norm;//幅值

    Te=Kt*Iq_Current;//转矩计算

    //转速外环参数
   if(speed_ref!=speed_ref_ctr)
   {
    //判断给定指令是否正确
    if(speed_ref_ctr>=0.0&&speed_ref_ctr<=3000.0)
    {
//        if(speed_ref_ctr>speed_ref) speed_ref++;
//        else  speed_ref=(float)speed_ref_ctr;
        speed_ref=(float)speed_ref_ctr;
    }
    else
    {speed_ref_ctr=(float)speed_ref;

    }

    speed_pid.PIDmax=speed_out_limit;
    speed_pid.PIDmin=-speed_out_limit;
    speed_pid.Imax=speed_out_limit;
    speed_pid.Imin=-speed_out_limit;

//    //电流内环参数
//    current_pid_d.PIDmax=speed_ref+2;
//    current_pid_d.PIDmin=-speed_ref-2;
//    current_pid_d.Imax=speed_ref+2;
//    current_pid_d.Imin=-speed_ref-2;
//
//    current_pid_q.PIDmax=speed_ref+2;
//    current_pid_q.PIDmin=-speed_ref-2;
//    current_pid_q.Imax=speed_ref+2;
//    current_pid_q.Imin=-speed_ref-2;
   }
   speed_pid.Kp=speed_pid_kp;
   speed_pid.Ki=speed_pid_ki;
//   current_pid_q.Kp=Iq_pid_kp;
//   current_pid_q.Ki=Iq_pid_ki;
//   current_pid_d.Kp=Id_pid_kp;
//   current_pid_d.Ki=Id_pid_ki;
//       current_pid_q.PIDmax=current_out_limit;
//       current_pid_q.PIDmin=-current_out_limit;
//       current_pid_q.Imax=current_out_limit;
//       current_pid_q.Imin=-current_out_limit;
//       current_pid_d.PIDmax=current_out_limit;
//       current_pid_d.PIDmin=-current_out_limit;
//       current_pid_d.Imax=current_out_limit;
//       current_pid_d.Imin=-current_out_limit;

    //PID控制
    //转速外环
    speed_pid.ref=-speed_ref;//转速参考
    speed_pid.fb=(float)motor.Speed_N;//转速反馈
    bsp_pid_ctrl(&speed_pid);

    //电流内环
    //d轴
    current_pid_d.ref=0;//电流给定，id=0控制
    current_pid_d.fb=(float)Id_Current;
    bsp_pid_ctrl(&current_pid_d);
    //q轴
    current_pid_q.ref=speed_pid.PIDout;//电流给定，即转速环输出
    current_pid_q.fb=(float)Iq_Current;
    bsp_pid_ctrl(&current_pid_q);
    //耦合项
    w1_d=motor.w_elec*(flux+Ld*Id_Current);
    w1_q=-motor.w_elec*Lq*Iq_Current;
    //解耦
    Id_out=(float32)current_pid_d.PIDout+w1_q;
    Iq_out=(float32)current_pid_q.PIDout+w1_d;
    //归一化
    Id_out_norm=Id_out*2/speed_normK;
    Iq_out_norm=Iq_out*2/speed_normK;
    //限幅
    if(Id_out_norm>1)Id_out_norm=1;
    if(Id_out_norm<-1)Id_out_norm=-1;
    if(Iq_out_norm>1)Iq_out_norm=1;
    if(Iq_out_norm<-1)Iq_out_norm=-1;

//    //dq--abc
//    dq0_abc1_cur.d =Id_out_norm;
//    dq0_abc1_cur.q =Iq_out_norm;
//    dq0_abc1_cur.z =0;
//    dq0_abc1_cur.sin =Sin_the;
//    dq0_abc1_cur.cos =Cos_the;
//    DQ0_ABC_F_FUNC(&dq0_abc1_cur);
//
//    Varef=dq0_abc1_cur.a;
//    if(Varef>1)Varef=1;
//    if(Varef<-1)Varef=-1;
//
//    Vbref=dq0_abc1_cur.b;
//    if(Vbref>1)Vbref=1;
//    if(Vbref<-1)Vbref=-1;
//
//    Vcref=dq0_abc1_cur.c;
//    if(Vcref>1)Vcref=1;
//    if(Vcref<-1)Vcref=-1;
////
//    EPwm1Regs.CMPA.bit.CMPA =  EPwm1Regs.TBPRD*((1.0+M*Varef)/2.0);
//    EPwm1Regs.CMPB.bit.CMPB =  EPwm1Regs.TBPRD*((1.0+M*Varef)/2.0);
//    EPwm2Regs.CMPA.bit.CMPA =  EPwm2Regs.TBPRD*((1.0+M*Vbref)/2.0);
//    EPwm2Regs.CMPB.bit.CMPB =  EPwm2Regs.TBPRD*((1.0+M*Vbref)/2.0);
//    EPwm3Regs.CMPA.bit.CMPA =  EPwm3Regs.TBPRD*((1.0+M*Vcref)/2.0);
//    EPwm3Regs.CMPB.bit.CMPB =  EPwm3Regs.TBPRD*((1.0+M*Vcref)/2.0);

    }

    //进行转子定位
    if(zhuanziDw_flag==1&&state_flag==0)
    {
      Id_out_norm=0.1;//d轴给定一个电压吸附转子的d轴
      Iq_out_norm=0;
      Sin_the=0;
      Cos_the=1;
      motor.theta_elec=0;//电角度置零
      motor.mech_position=0;//机械角度归零
      EQep1Regs.QPOSCNT=0;//编码器归零

    }
    //结束转子定位
    if(zhuanziDw_flag==0&&state_flag==0)
    {
      Id_out_norm=0;//给定置零
      Iq_out_norm=0;
      Sin_the=0;
      Cos_the=1;
      motor.theta_elec=0;//电角度置零
      motor.mech_position=0;//机械角度归零
      EQep1Regs.QPOSCNT=0;//编码器归零
    }

    //dq--abc
    dq0_abc1_cur.d =Id_out_norm;
    dq0_abc1_cur.q =Iq_out_norm;
    dq0_abc1_cur.z =0;
    dq0_abc1_cur.sin =Sin_the;
    dq0_abc1_cur.cos =Cos_the;
    DQ0_ABC_F_FUNC(&dq0_abc1_cur);

    Varef=dq0_abc1_cur.a;
    if(Varef>1)Varef=1;
    if(Varef<-1)Varef=-1;

    Vbref=dq0_abc1_cur.b;
    if(Vbref>1)Vbref=1;
    if(Vbref<-1)Vbref=-1;

    Vcref=dq0_abc1_cur.c;
    if(Vcref>1)Vcref=1;
    if(Vcref<-1)Vcref=-1;
//
    EPwm1Regs.CMPA.bit.CMPA =  EPwm1Regs.TBPRD*((1.0+M*Varef)/2.0);
    EPwm1Regs.CMPB.bit.CMPB =  EPwm1Regs.TBPRD*((1.0+M*Varef)/2.0);
    EPwm2Regs.CMPA.bit.CMPA =  EPwm2Regs.TBPRD*((1.0+M*Vbref)/2.0);
    EPwm2Regs.CMPB.bit.CMPB =  EPwm2Regs.TBPRD*((1.0+M*Vbref)/2.0);
    EPwm3Regs.CMPA.bit.CMPA =  EPwm3Regs.TBPRD*((1.0+M*Vcref)/2.0);
    EPwm3Regs.CMPB.bit.CMPB =  EPwm3Regs.TBPRD*((1.0+M*Vcref)/2.0);

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//直流量采样
__interrupt void adcd1_isr(void)
{
    Bus_Voltage_AD=AdcdResultRegs.ADCRESULT1;//2k欧：y = 0.0083x - 0.0463   1k欧y = 0.0159x - 0.1435
    Bus_Current_AD=AdcdResultRegs.ADCRESULT2;//y = 0.0062x + 0.0212
    Supercapacitor_Voltage_AD=AdcdResultRegs.ADCRESULT3;//y = 0.0083x - 0.065

//    Bus_Voltage=0.0083*Bus_Voltage_AD - 0.0463;
//    //直流母线电压
//    Bus_Voltage=VBus_filte.alpha * Bus_Voltage + (1 - VBus_filte.alpha) *(0.0159*Bus_Voltage_AD - 0.1435);//输出直流电压，滤波后
//    //直流电流
//    Bus_Current=VBus_filte.alpha *Bus_Current+(1 - VBus_filte.alpha) *(0.0062*Bus_Current_AD + 0.0212);
    //直流母线电压
    Bus_Voltage=0.0159*Bus_Voltage_AD - 0.1435;//输出直流电压
    //直流电流
    Bus_Current=0.0062*Bus_Current_AD + 0.0212;
    //超级电容电压
    Supercapacitor_Voltage=0.0159*Supercapacitor_Voltage_AD - 0.1435;

    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//定时器1中断
__interrupt void cpu_timer1_isr(void)
{
    LED2_TOGGLE();
    if(COM_Allow++>2) COM_Allow=1;//I2C 每两秒读一次
   // Acknowledge承认 this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;//中断响应标志位置 1，防止其他PIE组应答

}

__interrupt void INTERRUPT_ISR_TZProtect(void)
{
   // LED3_TOGGLE();
    TZ++;
 //   LED3_ON();
//    EALLOW;
////    EPwm1Regs.TZCLR.bit.OST=1;      //for tz trip One-shot Flag clear;
//    EPwm1Regs.TZCLR.bit.INT = 1;    //clear INT flag
//    EDIS;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2; // Acknowledge interrupt to PIE
}

//编码器计算
void POSSPEED_Calc()
{
    float32 temp_CapturePeriod=0;//捕获周期临时寄存器
    // QPOSCNT
     motor.mech_position=EQep1Regs.QPOSCNT;//读取当前位置
     motor.DirectionQep=EQep1Regs.QEPSTS.bit.QDF;//旋转方向
     //机械角度与电角度采样频率与控制频率相同，20K
     motor.theta_mech=motor.mech_position*motor.mech_scaler*2*PI;//机械角度
     motor.theta_elec=motor.theta_mech*motor.pole_pairs;//旋转角度，即电角度

//     SPEED_N=30*(motor.theta_mech-theta_mech_last)/(PI*5e-5);
//     theta_mech_last=motor.theta_mech;
     // Check an index occurrence
     ////索引事件锁存中断，即Z信号。发生时将位置计数器锁存到QPOSILAT寄存器。注意不是QPOSLAT
     if(EQep1Regs.QFLG.bit.IEL == 1)
     {
     //   p->index_sync_flag = 0x00F0;
        EQep1Regs.QCLR.bit.IEL = 1;    // Clear __interrupt flag
     }
     //转速采样频率为100Hz
     //高速计算
     //单位计时器中断
     if(EQep1Regs.QFLG.bit.UTO == 1)
     {
         motor.Now_position=EQep1Regs.QPOSLAT;//进入中断时的脉冲计数值，由QPOSCNT锁存进来
        if (motor.DirectionQep)    //正向旋转，增计数
        {
           if(motor.Last_position>motor.Now_position)
               motor.Speed_H=(10000-motor.Last_position+motor.Now_position)*SPEED_cal_K;    //0.01根据UTO的时间周期计算,0.6=1/10000*100*60
           else
               motor.Speed_H=(motor.Now_position-motor.Last_position)*SPEED_cal_K;     //0.01根据UTO的时间周期计算

        //   motor.w_elec=(float32)motor.Speed_H*PI*5.0/30.0;//电角速度计算
        }
        else
        {
           if(motor.Last_position>=motor.Now_position)
               motor.Speed_H=-(int)(motor.Last_position-motor.Now_position)*SPEED_cal_K; //0.01根据UTO的时间周期计算
           else
               motor.Speed_H=-(int)(10000-motor.Now_position+motor.Last_position)*SPEED_cal_K;    //0.01根据UTO的时间周期计算
        //   motor.w_elec=(float32)motor.Speed_H*PI*5.0/30.0;//电角速度计算
        }
        motor.Last_position=motor.Now_position;
        EQep1Regs.QCLR.bit.UTO=1;                   //清除中断标志
     }


//     //低速计算，与UTO无关
//     if(EQep1Regs.QEPSTS.bit.UPEVNT == 1)               // Unit position event，单位位置事件中断，
//     {
//         if(EQep1Regs.QEPSTS.bit.COEF == 0)             // No Capture overflow，捕获定时器溢出标志
//         {
//             temp_CapturePeriod = (unsigned long)EQep1Regs.QCPRDLAT; // temp1 = t2-t1，捕获周期值
//         }
//         else   // Capture overflow, saturate the result
//         {
//             temp_CapturePeriod = 0xFFFF;//计时器溢出，赋值给满 65535
//         }
//         motor.Speed_L=600000/temp_CapturePeriod;//转化为rpm
//         if(motor.Speed_L>6000) motor.Speed_L=6000;//当中断频率为100hZ时。最大转速/10000*100*60
//         if(motor.DirectionQep)
//         {
//             motor.Speed_L=motor.Speed_L;
//         }
//         else
//         {
//             motor.Speed_L=-motor.Speed_L;
//         }
//         EQep1Regs.QEPSTS.all = 0x88; // Clear Unit position event flag
//                                      // Clear overflow error flag
//     }

     motor.Speed_N=motor.Speed_H;
     motorspeed_rpm=motor.Speed_N;
     motor.w_elec=(float32)motor.Speed_N*PI*5.0/30.0;//电角速度计算
}




void Init_Variables(void)
{
    motor.Now_position=0;
    motor.Last_position=0;
    motor.mech_position=0;
    motor.Speed_N=0;
    motor.Speed_H=0;
    motor.Speed_L=0;
    motor.mech_scaler=1.0/10000.0;
    motor.theta_mech=0.0;
    motor.theta_elec=0.0;
    motor.w_elec=0;
    motor.pole_pairs=5;
}
void Init_EQEP1_Gpio()
{
    EALLOW;
    //    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;// 开启GPIO时钟
   // CpuSysRegs.PCLKCR1.bit.EQEP1ENCLK = 1;  // eQEP1
    CpuSysRegs.PCLKCR4.bit.EQEP1 = 1;  // eQEP1

     GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0; // Enable pull-up on GPIO50 (EQEP1A)
     GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0; // Enable pull-up on GPIO51 (EQEP1B)
     GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0; // Enable pull-up on GPIO53 (EQEP1I)

     GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 0; // Sync to SYSCLKOUT GPIO50 (EQEP1A)
     GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 0; // Sync to SYSCLKOUT GPIO51 (EQEP1B)
     GpioCtrlRegs.GPBQSEL2.bit.GPIO53 = 0; // Sync to SYSCLKOUT GPIO53 (EQEP1I)

     GpioCtrlRegs.GPBMUX2.bit.GPIO50=1; //QEPA
     GpioCtrlRegs.GPBMUX2.bit.GPIO51=1; //QEPB
     GpioCtrlRegs.GPBMUX2.bit.GPIO53=1; //QEPI

     EDIS;
}


//编码器配置
void Init_EQEP1()
{
    //停止EQEP模块
    EQep1Regs.QEPCTL.bit.QPEN = 0;
    //清除计数器和中断标志
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QCLR.all = 0xFFFF;
    //配置输入属性
    EQep1Regs.QDECCTL.bit.QSRC=0; //设定eQep的计数模式为正交模式
    EQep1Regs.QDECCTL.bit.SWAP=0; //QEPA和QEPB信号不交换
    EQep1Regs.QDECCTL.bit.QAP=0;  //QEPA信号不取反
    EQep1Regs.QDECCTL.bit.QBP=0;  //QEPB信号不取反
    EQep1Regs.QDECCTL.bit.QIP=0;  //QIP信号不取反
    EQep1Regs.QEPCTL.bit.FREE_SOFT=2;//仿真控制位：位置计数器不受影响
    //配置位置计数器运行模式、初始化方式、最大值
    EQep1Regs.QEPCTL.bit.PCRM=00; //设定PCRM=00，即QPOSCNT在每次Index脉冲都复位
    //正转复位到0，反转复位到QPOSMAX。
    EQep1Regs.QEPCTL.bit.IEI=2;     //在QEPI上升沿初始化位置计数器
    EQep1Regs.QPOSMAX =9999;
   //EQep1Regs.QPOSMAX = 0xFFFFFFFF;
    //QEP捕捉锁存模式设置为单位时间事件发生时将QPOSCNT的值锁存到QPOSLAT中
    EQep1Regs.QEPCTL.bit.QCLM=1;
    //配置UTE单元时间、中断使能、使能
    EQep1Regs.QUPRD=200000; //分频数。当SYSCLKOUT=200MHz时，2e6设定Unit Timer溢出频率为100Hz,2e5为1000hz；
    EQep1Regs.QEINT.bit.UTO=1;//使能UTO中断
    EQep1Regs.QEPCTL.bit.UTE=1;   //使能UTE,使能单位定时器
//    QFRC
//    EQep1Regs.QFRC.bit.UTO=1;//QEP的中断源

    //若要用QCAP单元进行精确的速度测量，可以配置QCAP单元
    EQep1Regs.QCAPCTL.bit.UPPS = 5;    // 1/32 预分频器//单位位置事件预分频器  n个边沿脉冲为一个事件
    EQep1Regs.QCAPCTL.bit.CCPS = 6;    // 1/64 预分频器//捕获时钟预分频  sysclk/n
    EQep1Regs.QCAPCTL.bit.CEN=1;  //使能eQEP的捕获功能
    //开启EQEP模块，并用软件初始化位置计数器
    EQep1Regs.QEPCTL.bit.SWI = 1;
    EQep1Regs.QEPCTL.bit.QPEN = 1;
}
//
//void  Init_EQEP1(void)
//{
//    EQep1Regs.QUPRD = 2000000;            // Unit Timer for 100Hz at 200 MHz
//                                          // SYSCLKOUT
//    EQep1Regs.QDECCTL.bit.QSRC = 00;      // QEP quadrature count mode
//    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
//    EQep1Regs.QEPCTL.bit.PCRM = 00;       // PCRM=00 mode - QPOSCNT reset on
//                                          // index event
//    EQep1Regs.QEPCTL.bit.UTE = 1;         // Unit Timeout Enable
//    EQep1Regs.QEPCTL.bit.QCLM = 1;        // Latch on unit time out
//    EQep1Regs.QPOSMAX =9999;
//    EQep1Regs.QEPCTL.bit.QPEN = 1;        // QEP enable
//    EQep1Regs.QCAPCTL.bit.UPPS = 5;       // 1/32 for unit position
//    EQep1Regs.QCAPCTL.bit.CCPS = 6;       // 1/64 for CAP clock
//    EQep1Regs.QCAPCTL.bit.CEN = 1;        // QEP Capture Enable
//}

void key_Init(void)
{
    //按键中断配置
        // GPIO26 and GPIO27 are inputs，作为中断触发源
        //
           EALLOW;
           GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;         // GPIO，功能复用选择
           GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;          // input
           GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0;        // XINT1 Synch to SYSCLKOUT only

           GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;         // GPIO
           GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;          // input
           GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 2;        // XINT2 Qual using 6 samples
           GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0xFF;   // Each sampling window
                                                       // is 510*SYSCLKOUT
           EDIS;
           // GPIO26 is XINT1, GPIO27 is XINT2
           //复用引脚为中断
              GPIO_SetupXINT1Gpio(26);
              GPIO_SetupXINT2Gpio(27);
        // Configure XINT1
        //上升沿触发
          XintRegs.XINT1CR.bit.POLARITY = 1;          //0 Falling edge interrupt.1 Rising edge interrupt
          XintRegs.XINT2CR.bit.POLARITY = 1;          // Falling edge interrupt
          //// Enable XINT1 and XINT2
                     //
        XintRegs.XINT1CR.bit.ENABLE = 1;            // Enable XINT1
        XintRegs.XINT2CR.bit.ENABLE = 1;            // Enable XINT2

        //配置中断函数地址
            EALLOW; // This is needed to write to EALLOW protected registers
            PieVectTable.XINT1_INT = &xint1_isr;
            PieVectTable.XINT2_INT = &xint2_isr;
            EDIS;   // This is needed to disable write to EALLOW protected registers
            //开CPU中断
            IER |= M_INT1;//开中断1,中断1.INT4、1.INT5在其中，即按键中断。
            //开PIE中断
            PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4，外部中断1，XINT1
            PieCtrlRegs.PIEIER1.bit.INTx5 = 1;          // Enable PIE Group 1 INT5，外部中断2,XINT2

}

interrupt void xint1_isr(void)///KEY1 关
{
//    GpioDataRegs.GPBCLEAR.all = 0x4;   // GPIO34 is low
//    Xint1Count++;

    //
    // Acknowledge this interrupt to get more from group 1
    //
    if(GpioDataRegs.GPADAT.bit.GPIO26==1)
    {
        DELAY_US(50);
        if(GpioDataRegs.GPADAT.bit.GPIO26==1)
        {
            while(GpioDataRegs.GPADAT.bit.GPIO26==1);

            Turn_on_off=0;//电容断开

            RELAY_1_OFF();//关继电器1
            DELAY_US(1*1000);//1ms
            RELAY_2_OFF();//关继电器2
        }

    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// xint2_isr - External Interrupt 2 ISR
//
interrupt void xint2_isr(void)///KEY2 开
{

    if(GpioDataRegs.GPADAT.bit.GPIO27==1)
    {
        DELAY_US(50);
        if(GpioDataRegs.GPADAT.bit.GPIO27==1)
        {
            while(GpioDataRegs.GPADAT.bit.GPIO27==1);

            Turn_on_off=1;//电容接入
        }

    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// i2c_int1a_isr - I2CA ISR
//中断服务程序会检查CurrentMsgPtr指向的消息结构体的MsgStatus字段，以确定当前操作的状态
__interrupt void i2c_int1a_isr(void)
{
    Uint16 IntSource, i;
    //
    // Read __interrupt source
    //
    IntSource = I2cbRegs.I2CISRC.all;//中断源

    //
    // Interrupt source = stop condition detected
    //检查是否检测到了停止条件中断
    //在I2C协议中，停止条件是由主设备生成的，用来通知总线上的所有设备通信即将结束。
    //停止条件的特点是SCL（时钟线）为高电平期间SDA（数据线）从低电平变为高电平。
    if(IntSource == I2C_SCD_ISRC)//STOP//先判断停止条件！！！！表明当前写、读操作已经结束
    {
        //
        // If completed message was writing data, reset msg to inactive state
        //如果当前消息状态是I2C_MSGSTAT_WRITE_BUSY（正在写入），则将其重置为非活动状态
        if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_WRITE_BUSY)
        {
            CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;//写完成

        }
        else
        {
            //
            // If a message receives a NACK during the address setup portion of
            // the EEPROM read, the code further below included in the register
            // access ready __interrupt source code will generate a stop
            // condition. After the stop condition is received (here), set the
            // message status to try again. User may want to limit the number of
            // retries before generating an error.

            //如果当前消息状态是I2C_MSGSTAT_SEND_NOSTOP_BUSY（正在发送地址，不停止），
            //则将其重置为准备发送状态（I2C_MSGSTAT_SEND_NOSTOP），以便重试。
            if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
            {
                CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;
            }
            //
            // If completed message was reading EEPROM data, reset msg to
            // inactive state and read data from FIFO.
            //
            //如果当前消息状态是I2C_MSGSTAT_READ_BUSY（正在读取），则将其重置为非活动状态，
            //并从FIFO中读取数据。
            else if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_READ_BUSY)//读完成
            {
                CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;

                for(i=0; i < I2cMsgIn1.NumOfBytes; i++)
                {
                    CurrentMsgPtr->MsgBuffer[i] = I2cbRegs.I2CDRR.all;
                }

               // COM_flag=3;//读完成
                I2cbRegs.I2CFFRX.bit.RXFFRST = 0; //复位 RXFIFO
                I2cbRegs.I2CFFRX.bit.RXFFRST = 1;    // Enable RXFIFO, clear RXFFINT

            }
        }
    }

    //
    // Interrupt source = Register Access Ready
    // This __interrupt is used to determine when the EEPROM address setup
    // portion of the read data communication is complete. Since no stop bit is
    // commanded, this flag tells us when the message has been sent instead of
    // the SCD flag. If a NACK is received, clear the NACK bit and command a
    // stop. Otherwise, move on to the read data portion of the communication.
    //处理寄存器访问就绪中断
    else if(IntSource == I2C_ARDY_ISRC)
    {
        if(I2cbRegs.I2CSTR.bit.NACK == 1)//如果检测到NACK,则发送停止条件并清除NACK位。
        {
            I2cbRegs.I2CMDR.bit.STP = 1;
            I2cbRegs.I2CSTR.all = I2C_CLR_NACK_BIT;
        }
        else if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
        {
            CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_RESTART;//更新为准备重启状态,以便进行数据读取
        }
    }
    else
    {
        //处理无效中断源:
        // Generate some error due to invalid __interrupt source
        //
        __asm("   ESTOP0");
    }

    //
    // Enable future I2C (PIE Group 8) __interrupts
    //重新启用I2C中断
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

   // 在中断处理程序中可以通过I2caRegs.I2CMDR.bit.IRS位清零从而复位I2CSTR状态寄存器，实现I2C模块的复位。
}


