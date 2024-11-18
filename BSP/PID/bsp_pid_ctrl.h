#ifndef _PID_CTRL_H
#define _PID_CTRL_H


typedef struct pid_ctrl
{
    float Kp;       /*比例*/
    float Ki;       /*积分*/
    float Kd;       /*微分*/

    float ref;      /*给定值*/
    float fb;       /*反馈值*/
    float err;      /*当前误差值*/
    float errl;     /*最后一次误差值*/
    float errd;     /*误差差值*/
    float errs;     /*误差累加*/

    float T;        /*控制周期*/

    float Imax;     /*积分项最大值*/
    float Imin;     /*积分项最小值*/
    float PIDmax;   /*PID输出最大值*/
    float PIDmin;   /*PID输出最小值*/

    float Pout;
    float Iout;
    float Dout;
    float PIDout;
}PID_CTRL;

void bsp_pid_ctrl(PID_CTRL *pid); 
void bsp_pid_init(PID_CTRL *pid);





#endif /*_PID_CTRL_H*/
