#include "led.h"

int Led_Count = 500; //LED��˸ʱ�����

/**************************************************************************
�������ܣ�LED�ӿڳ�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIOBʱ��
    GPIO_InitStructure.GPIO_Pin =  LED_R_PIN | LED_G_PIN | LED_B_PIN; //LED��ӦIO��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIO
    GPIO_SetBits(GPIOD, LED_R_PIN);
    GPIO_SetBits(GPIOD, LED_G_PIN);
    GPIO_SetBits(GPIOD, LED_B_PIN);
}
/**************************************************************************
�������ܣ��������ӿڳ�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void Buzzer_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIOBʱ��
    GPIO_InitStructure.GPIO_Pin =  Buzzer_PIN;//LED��ӦIO��
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIO
}
/**************************************************************************
�������ܣ�LED����˸����
��ڲ�������
����  ֵ����
**************************************************************************/
void led_task(void *pvParameters)
{

    while(1)
    {
        if(SBUS_ON_Flag == 0)
        {
            Send_Data.Send_Str.R_status[8] = 0x01;
            vTaskDelay(Led_Count);
            Send_Data.Send_Str.R_status[8] = 0x00;
        }

        LED_R = ~LED_R;

        //LED��˸����ǳ��򵥣���Ƶ�ʾ���Ҫ��ͣ�ʹ�������ʱ����
        vTaskDelay(Led_Count);

        xEventGroupSetBits(EventGroupHandler, EVENTBIT2);
    }
}

/**************************************************************************
�������ܣ�LED��˸
��ڲ�������˸ʱ��
�� �� ֵ����
**************************************************************************/
void Led_Flash(u16 time)
{
    static int temp;

    if(0 == time) Send_Data.Send_Str.R_status[8] = 0x00;
    else if(++temp == time) Send_Data.Send_Str.R_status[8] = 0x01, temp = 0;

    if(0 == time) LED_R = 0;
    else if(++temp == time)	LED_G = ~LED_G, temp = 0;
}

