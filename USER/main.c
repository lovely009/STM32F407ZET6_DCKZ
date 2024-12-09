#include "system.h"

//�������ȼ�
#define START_TASK_PRIO	1

//�����ջ��С
#define START_STK_SIZE 	256

//������
TaskHandle_t StartTask_Handler;

//������
void start_task(void *pvParameters);

//�¼�
EventGroupHandle_t EventGroupHandler;

//������
int main(void)
{
    systemInit(); //Ӳ����ʼ��

    //������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������
    vTaskStartScheduler();  //�����������
}

//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //�����ٽ���
    
    // �����¼���
    EventGroupHandler = xEventGroupCreate();

    //��������
    xTaskCreate(Control_task,  "Control_task",  CONTROL_STK_SIZE,  NULL, CONTROL_TASK_PRIO,  NULL);	//��������

    xTaskCreate(show_task,     "show_task",     SHOW_STK_SIZE,     NULL, SHOW_TASK_PRIO,     NULL);	//OLED��ʾ����ʾ����

    xTaskCreate(led_task,      "led_task",      LED_STK_SIZE,      NULL, LED_TASK_PRIO,      NULL);	//LED����˸����

    xTaskCreate(data_task,     "DATA_task",     DATA_STK_SIZE,     NULL, DATA_TASK_PRIO,     NULL);	//����&CAN������������

    xTaskCreate(iwdg_task,     "iwdg_task",     IWDG_STK_SIZE,     NULL, IWDG_TASK_PRIO,     NULL);	//���Ź�����

    vTaskDelete(StartTask_Handler); //ɾ����ʼ����

    taskEXIT_CRITICAL();            //�˳��ٽ���
}






