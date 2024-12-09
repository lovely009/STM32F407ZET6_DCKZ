#include "timer.h"

/**************************************************************************
�������ܣ��������PWM�������ų�ʼ��
��ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ��
����  ֵ����
**************************************************************************/
void Tim_PUL_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  	 //TIM2ʱ��ʹ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTBʱ��

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;           //GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;    //
    GPIO_Init(GPIOA, &GPIO_InitStructure);             //��ʼ��PB��

    //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Period = arr;
    //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_ClockDivision = 1;
    //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

    //�������:TIM����Ƚϼ��Ը�
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    //�߼���ʱ���������ʹ�����
    TIM_CtrlPWMOutputs(TIM2, ENABLE);

    //CHԤװ��ʹ��
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM2->CCR1 = 0;

    //ʹ��TIM3
    TIM_Cmd(TIM2, ENABLE);

}
//�������Ź�����
void IWDG_Config(void)
{
    // ʹ�ܿ��Ź�ʱ��
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    // ����Ԥ��Ƶϵ��Ϊ 4
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    // ���ó�ʱʱ����Ϊ 3 ��
    IWDG_SetReload(1500);
    // �������Ź�
    IWDG_Enable();
}

void iwdg_task(void *pvParameters)
{
    while (1)
    {
        if (xEventGroupWaitBits(EventGroupHandler, EVENTBIT0 | EVENTBIT1 | EVENTBIT2 | EVENTBIT3,
                                pdTRUE, pdTRUE, portMAX_DELAY) == (EVENTBIT0 | EVENTBIT1 | EVENTBIT2 | EVENTBIT3))
        {
            IWDG_ReloadCounter();
        }
    }
}

