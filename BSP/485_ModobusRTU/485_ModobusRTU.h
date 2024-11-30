/*
 * 485.h
 *
 *  Created on: 2024年11月30日
 *      Author: XUQQ
 */
#ifndef MODOBUSRTU_485_H_
#define MODOBUSRTU_485_H_
//
// 包含的头文件
//
#include "F28x_Project.h"

//
// 宏定义
//
#define CPU_FREQ        200E6      // CPU频率为60 MHz
#define LSPCLK_FREQ     CPU_FREQ/4 // LSP时钟频率为CPU频率的四分之一
#define SCI_FREQ        115200      // SCI波特率115200 bps
#define SCI_PRD         (LSPCLK_FREQ/(SCI_FREQ*8))-1 // SCI时钟周期

#define MODBUS_SLAVE_ADDR 0x01   // 从机地址
//
// 函数原型声明
//
interrupt void sciaRxFifoIsr(void);      // SCI-A 接收FIFO中断服务程序
void modbus_process_request(void);       // 处理MODBUS请求
void modbus_send_response(Uint16 data_len);         // 发送MODBUS响应
void scia_fifo_init(void);               // SCI-A FIFO初始化
Uint16 modbus_calculate_crc(Uint16 *data, Uint16 length); // 计算MODBUS CRC
void Modobus_485_ReceiveErr_handle(void);// //485通信接受错误处理，恢复
void Modobus_485_GpioInit(void);
//错误标志
extern int crc_err;
extern int func_code_err;

extern int receive_Rerr;
extern int slave_addr_Rerr;
extern Uint16 MODBUS_REG_VALUE[15];  // 寄存器值

#endif /* MODOBUSRTU_485_H_ */
