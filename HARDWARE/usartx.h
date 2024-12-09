#ifndef __USRATX_H
#define __USRATX_H

#include "stdio.h"
#include "sys.h"
#include "system.h"

#define DATA_STK_SIZE   512
#define DATA_TASK_PRIO  4

#define FRAME_HEADER      0X01 //帧头
#define FRAME_TAIL        0X80 //帧尾
#define SEND_DATA_SIZE    49   //发送缓冲区大小
#define RECEIVE_DATA_SIZE 12   //接受缓冲区大小

extern volatile u8 UART3_Flag ;

/*******发送数据的结构体*************************************/
typedef struct _SEND_DATA_
{
    unsigned char buffer[SEND_DATA_SIZE];
    struct _Send_Str_
    {
        unsigned char Frame_Header;      //帧头
        signed char Speed;               //档位
        volatile signed char A_speed;    //A轮速度
        volatile signed char B_speed;    //B轮速度
        signed char R_number;            //继电器序号
        volatile signed char R_status[9];//继电器状态
        unsigned char Frame_Tail;        //帧尾
    } Send_Str;
} SEND_DATA;

extern SEND_DATA Send_Data;

/*******接受数据的结构体*************************************/
typedef struct _RECEIVE_DATA_
{
    unsigned char buffer[RECEIVE_DATA_SIZE];
    struct _Receive_Str_
    {
        unsigned char Frame_Header;     //帧头
        signed char Instruction;        //指令
        signed char A_speed;            //A轮速度
        signed char B_speed;            //B轮速度
        unsigned char Frame_Tail;       //帧尾
    } Receive_Str;
} RECEIVE_DATA;

extern RECEIVE_DATA Receive_Data;

void data_task(void *pvParameters);
void data_transition(void);

void USART1_SEND(void);
void USART2_SEND(void);
void USART3_SEND(void);
void UART4_SEND(void);
void CAN_SEND(void);
void CAN_SEND1(void);

void usart1_init(u32 bound);
void usart2_init(u32 bound);
void usart3_init(u32 bound);
void uart4_init(u32 bound);

int USART1_IRQHandler(void);
int USART2_IRQHandler(void);
int USART3_IRQHandler(void);
int UART4_IRQHandler(void);

void usart1_send(u8 data);
void usart2_send(u8 data);
void usart3_send(u8 data);
void uart4_send(u8 data);

u8 Check_Sum(void);
#endif

