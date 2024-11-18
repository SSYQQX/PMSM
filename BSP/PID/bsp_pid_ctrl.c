#include "bsp_pid_ctrl.h"



void bsp_pid_ctrl(PID_CTRL *pid)
{
    /* ----------------------位置式PID------------------------- */
     pid->err =pid->ref-pid->fb ;
//     pid->errs += pid->err;
//
//     /* 误差限幅 */
//     if(pid->errs > pid->Imax*2000) {
//         pid->errs = pid->Imax*2000;
//     }else if(pid->errs < pid->Imin*2000) {
//         pid->errs = pid->Imin*2000;
//     }

     /*微分系数为0时不做微分运算*/
     if(pid->Kd != 0) {
         pid->errd = pid->err - pid->errl;
         pid->errl = pid->err;
         pid->Dout = pid->Kd * pid->errd;
     }

     pid->Pout = pid->Kp * pid->err;
     pid->Iout = pid->Ki * pid->err + pid->Iout;

     /* 积分限幅 */
     if(pid->Iout > pid->Imax) {
         pid->Iout = pid->Imax;
     }else if(pid->Iout < pid->Imin) {
         pid->Iout = pid->Imin;
     }

     pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
     /* PID输出限幅 */
     if(pid->PIDout > pid->PIDmax) {
         pid->PIDout = pid->PIDmax;
     }else if(pid->PIDout < pid->PIDmin) {
         pid->PIDout = pid->PIDmin;
     }
    /* ----------------------位置式PID------------------------- */



    /* ----------------------增量式PID------------------------- */
//    pid->err =  pid->ref-pid->fb;
//    pid->Pout = pid->Kp * pid->errl;
//    pid->Iout = pid->Ki * pid->err;
//    pid->PIDout += (pid->Pout + pid->Iout);
//    if(pid->PIDout > pid->PIDmax) {
//        pid->PIDout = pid->PIDmax;
//    }else if(pid->PIDout < pid->PIDmin) {
//        pid->PIDout = pid->PIDmin;
//    }
//    pid->errl = pid->err;

    /* ----------------------增量式PID------------------------- */
}
void bsp_pid_init(PID_CTRL *pid)
{
    pid->Kp = 0;
    pid->Ki = 0;
    pid->Kd = 0;
    pid->ref = 0;
    pid->fb = 0;
    pid->err = 0;
    pid->errl = 0;
    pid->errd = 0;
    pid->errs = 0;
    pid->T = 0;
    pid->Imax = 0;
    pid->Imin = 0;
    pid->PIDmax = 0;
    pid->PIDmin = 0;
    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->PIDout = 0;
}
