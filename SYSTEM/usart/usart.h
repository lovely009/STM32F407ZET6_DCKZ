#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "sys.h"

#define UART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_UART5_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

extern u8  UART5_RX_BUF[UART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�

extern u16 UART5_RX_STA;         		//����״̬���


//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart5_init(u32 bound);

//����subs�źŵ���Сֵ ���ֵ ��ֵ ���� �Լ�ϣ��ת���ķ�Χ
#define SBUS_RANGE_MIN    200     //��Сֵ
#define SBUS_RANGE_MAX    1800    //���ֵ
#define SBUS_RANGE_MIDDLE 1000    //��ֵ
#define SBUS_DEAD_RANGE   100      //����
#define SBUS_TARGET_RANGE 100     //ת����Χ
#define SBUS_SCALE_FACTOR 0.125   //��������

//SBUS�źŽ�����غ���
extern u8 SBUS[25];
extern u16 sbus[16];
void update_sbus(u16 *CH, u8 *buf);
signed char sbus_to(u16 sbus_value);

#endif


