#ifndef __TIMER_H
#define __TIMER_H
#include "system.h"

#define IWDG_TASK_PRIO		1     //�������ȼ�
#define IWDG_STK_SIZE 		256   //�����ջ��С

void iwdg_task(void *pvParameters);

void Tim_PUL_Init(u16 arr, u16 psc);
void IWDG_Config(void);

#endif
