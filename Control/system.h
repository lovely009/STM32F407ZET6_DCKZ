#ifndef __SYSTEM_H
#define __SYSTEM_H

//����������Ҫ�õ���ͷ�ļ�
#include "FreeRTOSConfig.h"

//FreeRTOS���ͷ�ļ�
#include "FreeRTOS.h"
#include "event_groups.h"
#include "stm32f4xx.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

//��������ͷ�ļ�
#include "sys.h"
#include "delay.h"
#include "control.h"
#include "usart.h"
#include "usartx.h"
#include "can.h"
#include "timer.h"
#include "led.h"
#include "oled.h"
#include "show.h"

/****** �ⲿ�������壬������c�ļ�����system.hʱ��Ҳ����ʹ��system.c����ı��� ******/
extern u8 SBUS_ON_Flag;

extern EventGroupHandle_t EventGroupHandler;

void systemInit(void);

/***�궨��***/
#define RATE_1_HZ		1
#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***�궨��***/
#define EVENTBIT0  			(1 << 0)
#define EVENTBIT1  			(1 << 1)
#define EVENTBIT2  			(1 << 2)
#define EVENTBIT3  			(1 << 3)
#define EVENTBIT4  			(1 << 4)

//C�⺯�������ͷ�ļ�
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "stdarg.h"
#endif
