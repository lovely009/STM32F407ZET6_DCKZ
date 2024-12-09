#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "sys.h"

#define UART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_UART5_RX 			1		//使能（1）/禁止（0）串口1接收

extern u8  UART5_RX_BUF[UART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符

extern u16 UART5_RX_STA;         		//接收状态标记


//如果想串口中断接收，请不要注释以下宏定义
void uart5_init(u32 bound);

//定义subs信号的最小值 最大值 中值 死区 以及希望转换的范围
#define SBUS_RANGE_MIN    200     //最小值
#define SBUS_RANGE_MAX    1800    //最大值
#define SBUS_RANGE_MIDDLE 1000    //中值
#define SBUS_DEAD_RANGE   100      //死区
#define SBUS_TARGET_RANGE 100     //转换范围
#define SBUS_SCALE_FACTOR 0.125   //比例因子

//SBUS信号解析相关函数
extern u8 SBUS[25];
extern u16 sbus[16];
void update_sbus(u16 *CH, u8 *buf);
signed char sbus_to(u16 sbus_value);

#endif


