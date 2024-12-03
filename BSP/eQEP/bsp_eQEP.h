/*
 * bsp_eQEP.h
 *
 *  Created on: 2024年12月2日
 *      Author: XUQQ
 */

#ifndef BSP_EQEP_BSP_EQEP_H_
#define BSP_EQEP_BSP_EQEP_H_

#define PI     3.14159265f

//extern struct Motor_Para motor;
#include "F28x_Project.h"



typedef struct
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
}Motor_Para;

// 声明外部 motor 变量
extern Motor_Para motor;

__interrupt void myEQEP1_ISR(void);
void Init_Variables(void);
void Init_EQEP1_Gpio(void);
void Init_EQEP1(void);
void POSSPEED_Calc();


#endif /* BSP_EQEP_BSP_EQEP_H_ */
