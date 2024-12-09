#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "system.h"
#include <stdbool.h>

#define CONTROL_TASK_PRIO		2     //�������ȼ�
#define CONTROL_STK_SIZE 		512   //�����ջ��С

extern u8 command_lost_count;
void Control_task(void *pvParameters);
void Remote_control(void);
void Ros_control(void);
bool Variable_detection(u8 index, u16 New_Value);
void DIR_Init(void);
#endif

