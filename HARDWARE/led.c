#include "led.h"

int Led_Count = 500; //LED闪烁时间控制

/**************************************************************************
函数功能：LED接口初始化
入口参数：无
返回  值：无
**************************************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
    GPIO_InitStructure.GPIO_Pin =  LED_R_PIN | LED_G_PIN | LED_B_PIN; //LED对应IO口
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO
    GPIO_SetBits(GPIOD, LED_R_PIN);
    GPIO_SetBits(GPIOD, LED_G_PIN);
    GPIO_SetBits(GPIOD, LED_B_PIN);
}
/**************************************************************************
函数功能：蜂鸣器接口初始化
入口参数：无
返回  值：无
**************************************************************************/
void Buzzer_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
    GPIO_InitStructure.GPIO_Pin =  Buzzer_PIN;//LED对应IO口
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO
}
/**************************************************************************
函数功能：LED灯闪烁任务
入口参数：无
返回  值：无
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

        //LED闪烁任务非常简单，对频率精度要求低，使用相对延时函数
        vTaskDelay(Led_Count);

        xEventGroupSetBits(EventGroupHandler, EVENTBIT2);
    }
}

/**************************************************************************
函数功能：LED闪烁
入口参数：闪烁时间
返 回 值：无
**************************************************************************/
void Led_Flash(u16 time)
{
    static int temp;

    if(0 == time) Send_Data.Send_Str.R_status[8] = 0x00;
    else if(++temp == time) Send_Data.Send_Str.R_status[8] = 0x01, temp = 0;

    if(0 == time) LED_R = 0;
    else if(++temp == time)	LED_G = ~LED_G, temp = 0;
}

