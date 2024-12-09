#include "timer.h"

/**************************************************************************
函数功能：步进电机PWM脉冲引脚初始化
入口参数：arr：自动重装值  psc：时钟预分频数
返回  值：无
**************************************************************************/
void Tim_PUL_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  	 //TIM2时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTB时钟

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;           //GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;    //
    GPIO_Init(GPIOA, &GPIO_InitStructure);             //初始化PB口

    //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Period = arr;
    //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_ClockDivision = 1;
    //向上计数模式
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    //选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    //比较输出使能
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

    //输出极性:TIM输出比较极性高
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    //高级定时器输出必须使能这句
    TIM_CtrlPWMOutputs(TIM2, ENABLE);

    //CH预装载使能
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    //使能TIMx在ARR上的预装载寄存器
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM2->CCR1 = 0;

    //使能TIM3
    TIM_Cmd(TIM2, ENABLE);

}
//独立看门狗配置
void IWDG_Config(void)
{
    // 使能看门狗时钟
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    // 设置预分频系数为 4
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    // 设置超时时间大概为 3 秒
    IWDG_SetReload(1500);
    // 启动看门狗
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

