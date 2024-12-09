#include "system.h"

//航模SBUS通信控制标志位。
u8 SBUS_ON_Flag = 1;

void systemInit(void)
{
    //中断优先级分组设置
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //延时函数初始化
    delay_init(168);

    //初始化与LED灯连接的硬件接口
    LED_Init();

    //初始化与蜂鸣器连接的硬件接口
    Buzzer_Init();

    //初始化与OLED显示屏连接的硬件接口
    OLED_Init();

    //串口1初始化，通信波特率115200，用于PC
    usart1_init(115200);

    //串口3初始化，通信波特率115200，用于ROS
    usart3_init(115200);

    //串口4初始化，通信波特率115200，用与蓝牙
    //uart4_init(115200);

    //串口5初始化，通信波特率100000，用于航模SBUS接口
    uart5_init(100000);

    //CAN通信接口初始化,用于与驱动器通信
    CAN1_Mode_Init(1, 5, 15, 8, 0); //250kbps

    //初始化定时器，用于步进电机脉冲，频率10KHZ
    Tim_PUL_Init(1049, 24); //通用定时器TIM2的频率为84M，频率=84M/((1049+1)*(99+1))=800

    //初始化步进电机引脚
    DIR_Init();

    //初始化独立看门狗
    IWDG_Config();
}

