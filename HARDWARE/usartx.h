#ifndef __USRATX_H
#define __USRATX_H

#include "stdio.h"
#include "sys.h"
#include "system.h"

#define DATA_STK_SIZE   512
#define DATA_TASK_PRIO  4

#define FRAME_HEADER      0X01 //֡ͷ
#define FRAME_TAIL        0X80 //֡β
#define SEND_DATA_SIZE    49   //���ͻ�������С
#define RECEIVE_DATA_SIZE 12   //���ܻ�������С

extern volatile u8 UART3_Flag ;

/*******�������ݵĽṹ��*************************************/
typedef struct _SEND_DATA_
{
    unsigned char buffer[SEND_DATA_SIZE];
    struct _Send_Str_
    {
        unsigned char Frame_Header;      //֡ͷ
        signed char Speed;               //��λ
        volatile signed char A_speed;    //A���ٶ�
        volatile signed char B_speed;    //B���ٶ�
        signed char R_number;            //�̵������
        volatile signed char R_status[9];//�̵���״̬
        unsigned char Frame_Tail;        //֡β
    } Send_Str;
} SEND_DATA;

extern SEND_DATA Send_Data;

/*******�������ݵĽṹ��*************************************/
typedef struct _RECEIVE_DATA_
{
    unsigned char buffer[RECEIVE_DATA_SIZE];
    struct _Receive_Str_
    {
        unsigned char Frame_Header;     //֡ͷ
        signed char Instruction;        //ָ��
        signed char A_speed;            //A���ٶ�
        signed char B_speed;            //B���ٶ�
        unsigned char Frame_Tail;       //֡β
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

