#include "control.h"

u16 sbus_Value[16];
int Time_count = 0;
u8 command_lost_count = 0; //SBUS、ros控制命令丢失时间计数，丢失一秒后停止控制

/**************************************************************************
函数功能：FreeRTOS任务，核心控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Control_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while(1)
    {
        //此任务以100Hz的频率运行（10ms控制一次）
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

        if(Time_count < 100)Time_count++;//1秒

        if(SBUS_ON_Flag)Remote_control();

        else            Ros_control();

        xEventGroupSetBits(EventGroupHandler, EVENTBIT0);
    }
}

void Remote_control(void)
{
    //速度调节中高低
    if(Variable_detection(5, sbus[5]))
    {
        if(sbus[5] < 1500 && sbus[5] > 500)
        {
            Send_Data.Send_Str.Speed = 2;
        }
        else if(sbus[5] > 1500)
        {
            Send_Data.Send_Str.Speed = 1;
        }
        else if(sbus[5] < 500)
        {
            Send_Data.Send_Str.Speed = 4;
        }
    }

    //运动控制
    if(Variable_detection(0, sbus[0]) || Variable_detection(2, sbus[2]))
    {
        int forwardSpeed = sbus_to(sbus[2]) / Send_Data.Send_Str.Speed;  // 前后速度
        int turnSpeed = sbus_to(sbus[0]) / Send_Data.Send_Str.Speed;     // 左右速度
        Send_Data.Send_Str.A_speed = forwardSpeed + turnSpeed;
        Send_Data.Send_Str.B_speed = forwardSpeed - turnSpeed;
    }


    //前臂步进电机控制
    if (Variable_detection(4, sbus[4]))
    {
        if(sbus[4] < 1500 && sbus[4] > 500)
        {
            TIM_SetCompare1(TIM2, 0);
        }

        else
        {
            if(sbus[4] > 1500)
            {
                GPIO_ResetBits(GPIOB, GPIO_Pin_3);
            }
            else
            {
                GPIO_SetBits(GPIOB, GPIO_Pin_3);
            }
            TIM_SetCompare1(TIM2, 500);
        }
    }

    //后臂电推杆控制
    if (Variable_detection(6, sbus[6]))
    {
        if(sbus[6] < 1500 && sbus[6] > 500)
        {
            Send_Data.Send_Str.R_status[4] = 0x00;
            Send_Data.Send_Str.R_status[5] = 0x00;
            Send_Data.Send_Str.R_status[6] = 0x00;
            Send_Data.Send_Str.R_status[7] = 0x00;

        }
        else if(sbus[6] > 1500)
        {
            Send_Data.Send_Str.R_status[4] = 0x01;
            Send_Data.Send_Str.R_status[5] = 0x01;
            Send_Data.Send_Str.R_status[6] = 0x00;
            Send_Data.Send_Str.R_status[7] = 0x00;
        }
        else if(sbus[6] < 500)
        {
            Send_Data.Send_Str.R_status[4] = 0x00;
            Send_Data.Send_Str.R_status[5] = 0x00;
            Send_Data.Send_Str.R_status[6] = 0x01;
            Send_Data.Send_Str.R_status[7] = 0x01;
        }
    }

    //旋耕电机控制
    if (Variable_detection(9, sbus[9]))
    {
        if(sbus[9] > 1500)
        {
            Send_Data.Send_Str.R_status[1] = 0x01;
        }
        else if(sbus[9] < 500)
        {
            Send_Data.Send_Str.R_status[1] = 0x00;
        }
    }

    //旋耕机电机转速控制
    if (Variable_detection(7, sbus[7]))
    {
        if(sbus[7] < 1500 && sbus[7] > 500)
        {
            Send_Data.Send_Str.R_status[2] = 0x00;
            Send_Data.Send_Str.R_status[3] = 0x00;
        }
        else if(sbus[7] > 1500)//高速
        {
            Send_Data.Send_Str.R_status[2] = 0x01;
            Send_Data.Send_Str.R_status[3] = 0x00;
        }
        else if(0 < sbus[7] && sbus[7] < 500)
        {
            Send_Data.Send_Str.R_status[2] = 0x00;
            Send_Data.Send_Str.R_status[3] = 0x01;
        }
    }

    //记录经纬度
    if (Variable_detection(11, sbus[11]))
    {
        if(sbus[11] > 1500||sbus[11] < 500)
        {
            usart3_send(0x01);
            usart3_send(0x33);
            usart3_send(0x80);
        }
    }

    //清除航点
    if (Variable_detection(12, sbus[12]))
    {
            usart3_send(0x01);
            usart3_send(0x66);
            usart3_send(0x80);
    }
}

void Ros_control()
{
    if(USART3_IRQHandler()== 0x11)
    {
        switch(Receive_Data.Receive_Str.Instruction)
        {
            case 0x10:
                Send_Data.Send_Str.A_speed = 10;
                Send_Data.Send_Str.B_speed = 10;
                break;//前进

            case 0x11:
                Send_Data.Send_Str.A_speed = -10;
                Send_Data.Send_Str.B_speed = -10;
                break;//后退

            case 0x12:
                Send_Data.Send_Str.A_speed = -1;
                Send_Data.Send_Str.B_speed = 1;
                break;//左旋

            case 0x13:
                Send_Data.Send_Str.A_speed = 1;
                Send_Data.Send_Str.B_speed = -1;
                break;//右旋

            case 0x14:
                Send_Data.Send_Str.A_speed = 0;
                Send_Data.Send_Str.B_speed = 0;
                break;//刹车

            case 0x15:
                GPIO_ResetBits(GPIOB, GPIO_Pin_3);
                TIM_SetCompare1(TIM2, 500);
                break;//前臂升

            case 0x16:
                GPIO_SetBits(GPIOB, GPIO_Pin_3);
                TIM_SetCompare1(TIM2, 500);
                break;//前臂降

            case 0x17:
                TIM_SetCompare1(TIM2, 0);
                break;//前臂停止

            case 0x18:
                Send_Data.Send_Str.R_status[4] = 0x01;
                Send_Data.Send_Str.R_status[5] = 0x01;
                Send_Data.Send_Str.R_status[6] = 0x00;
                Send_Data.Send_Str.R_status[7] = 0x00;
                break;//后臂升

            case 0x19:
                Send_Data.Send_Str.R_status[4] = 0x00;
                Send_Data.Send_Str.R_status[5] = 0x00;
                Send_Data.Send_Str.R_status[6] = 0x01;
                Send_Data.Send_Str.R_status[7] = 0x01;
                break;//后臂降

            case 0x20:
                Send_Data.Send_Str.R_status[4] = 0x00;
                Send_Data.Send_Str.R_status[5] = 0x00;
                Send_Data.Send_Str.R_status[6] = 0x00;
                Send_Data.Send_Str.R_status[7] = 0x00;
                break;//后臂停止

            case 0x21:
                Send_Data.Send_Str.R_status[1] = 0x01;
                break;//旋耕电机启动

            case 0x22:
                Send_Data.Send_Str.R_status[1] = 0x00;
                break;//旋耕电机停止

            default :
                break;
        }
    }

    else if(USART3_IRQHandler()== 0x22)
    {
        Send_Data.Send_Str.A_speed = Receive_Data.Receive_Str.A_speed;
        Send_Data.Send_Str.B_speed = Receive_Data.Receive_Str.B_speed;
    }

}

//变量检测函数
bool Variable_detection(u8 index, u16 New_Value)
{
    u16 diff = abs((int)New_Value - (int)sbus_Value[index]);

    if (diff <= 10)
    {
        return false; // 如果值相同，返回false
    }
    else
    {
        sbus_Value[index] = New_Value; // 更新值
        return true; // 如果值不同，返回true
    }
}

void DIR_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; //DIR对应IO口
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB3
}

