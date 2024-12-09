#ifndef __TIMER_H
#define __TIMER_H
#include "system.h"

#define IWDG_TASK_PRIO		1     //任务优先级
#define IWDG_STK_SIZE 		256   //任务堆栈大小

void iwdg_task(void *pvParameters);

void Tim_PUL_Init(u16 arr, u16 psc);
void IWDG_Config(void);

#endif
