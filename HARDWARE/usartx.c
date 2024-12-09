#include "usartx.h"
#include "system.h"

volatile u8 UART3_Flag = 0;
extern int Time_count;

SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
/*************************************************************************
函数功能：串口，CAN发送数据任务
入口参数：无
返回  值：无
**************************************************************************/
void data_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while(1)
    {
        //此任务以50Hz的频率运行(20ms)
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));
        //对要进行发送的数据进行赋值
        data_transition();
        //开启发送数据
        CAN_SEND();           //CAN发送数据
        //CAN_SEND1();          //CAN读取驱动器
        //USART3_SEND();        //串口3发送数据->ROS
        //USART1_SEND();        //串口1发送数据->PC
        xEventGroupSetBits(EventGroupHandler, EVENTBIT3);
    }
}
/**************************************************************************
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{
    //can通信
    Send_Data.buffer[0] = 0x2B;
    Send_Data.buffer[1] = 0x00;
    Send_Data.buffer[2] = 0x00;
    Send_Data.buffer[3] = 0x00;
    Send_Data.buffer[4] = Send_Data.Send_Str.A_speed;
    Send_Data.buffer[5] = Send_Data.Send_Str.B_speed;
    Send_Data.buffer[6] = 0x00;
    Send_Data.buffer[7] = 0x00;

    //继电器单路控制指令
    Send_Data.buffer[8] = 0x48;
    Send_Data.buffer[9] = 0x3a;
    Send_Data.buffer[10] = 0x01;
    Send_Data.buffer[11] = 0x70;
    Send_Data.buffer[12] = Send_Data.Send_Str.R_number;
    Send_Data.buffer[13] = Send_Data.Send_Str.R_status[Send_Data.Send_Str.R_number];
    Send_Data.buffer[14] = 0x00;
    Send_Data.buffer[15] = 0x00;
    Send_Data.buffer[16] = 0x45;
    Send_Data.buffer[17] = 0x44;

    //继电器集中控制指令
    Send_Data.buffer[18] = 0x48;
    Send_Data.buffer[19] = 0x3a;
    Send_Data.buffer[20] = 0x01;
    Send_Data.buffer[21] = 0x57;
    //继电器CAN控制
    Send_Data.buffer[22] = Send_Data.Send_Str.R_status[1];
    Send_Data.buffer[23] = Send_Data.Send_Str.R_status[2];
    Send_Data.buffer[24] = Send_Data.Send_Str.R_status[3];
    Send_Data.buffer[25] = Send_Data.Send_Str.R_status[4];
    Send_Data.buffer[26] = Send_Data.Send_Str.R_status[5];
    Send_Data.buffer[27] = Send_Data.Send_Str.R_status[6];
    Send_Data.buffer[28] = Send_Data.Send_Str.R_status[7];
    Send_Data.buffer[29] = Send_Data.Send_Str.R_status[8];
    //
    Send_Data.buffer[30] = Check_Sum();
    Send_Data.buffer[31] = 0x45;
    Send_Data.buffer[32] = 0x44;

    //读A、B电机转速
    Send_Data.buffer[33] = 0x40;
    Send_Data.buffer[34] = 0x03;
    Send_Data.buffer[35] = 0x02;
    Send_Data.buffer[36] = 0x00;
    Send_Data.buffer[37] = 0x00;
    Send_Data.buffer[38] = 0x00;
    Send_Data.buffer[39] = 0x00;
    Send_Data.buffer[40] = 0x00;

    Send_Data.buffer[41] = 0x40;
    Send_Data.buffer[42] = 0x04;
    Send_Data.buffer[43] = 0x02;
    Send_Data.buffer[44] = 0x00;
    Send_Data.buffer[45] = 0x00;
    Send_Data.buffer[46] = 0x00;
    Send_Data.buffer[47] = 0x00;
    Send_Data.buffer[48] = 0x00;
}
/**************************************************************************
函数功能：串口1发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART1_SEND(void)
{


}
/**************************************************************************
函数功能：串口2发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART2_SEND(void)
{

}
/**************************************************************************
函数功能：串口3发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART3_SEND(void)
{
    u8 i, USART3_SEND[4] = {FRAME_HEADER, Receive_Data.Receive_Str.A_speed, Receive_Data.Receive_Str.B_speed, FRAME_TAIL};

    for (i = 0; i < 4; i++)
    {
        usart3_send(USART3_SEND[i]);
    }
}
/**************************************************************************
函数功能：串口4发送数据
入口参数：无
返回  值：无
**************************************************************************/
void UART4_SEND(void)
{

}
/**************************************************************************
函数功能：CAN发送数据
入口参数：无
返 回 值：无
**************************************************************************/
void CAN_SEND(void)
{
    u8 CAN_SENT[8], CAN_SENT1[8], CAN_SENT2[8], CAN_SENT3[8], i;

    for(i = 0; i < 8; i++)
    {
        CAN_SENT[i] = Send_Data.buffer[i];
        CAN_SENT1[i] = Send_Data.buffer[i + 22];
//        CAN_SENT2[i] = Send_Data.buffer[i + 33];
//        CAN_SENT3[i] = Send_Data.buffer[i + 41];
    }

    CAN1_Send_Num(0x601, CAN_SENT);
    CAN1_Send_Num(0x701, CAN_SENT1);
    //CAN1_Send_Num(0x601, CAN_SENT2);
    //CAN1_Send_Num(0x601, CAN_SENT3);

}
/**************************************************************************
函数功能：串口1初始化
入口参数：无
返 回 值：无
**************************************************************************/
void usart1_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //使能GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //使能USART时钟

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          //输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //高速50MHZ
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                //初始化

    //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;
    //子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //IRQ通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //根据指定的参数初始化VIC寄存器
    NVIC_Init(&NVIC_InitStructure);

    //初始化设置
    USART_InitStructure.USART_BaudRate = bound; //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口1

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启串口接受中断
    USART_Cmd(USART1, ENABLE);                     //使能串口1
}
/**************************************************************************
函数功能：串口2初始化
入口参数：无
返 回 值：无
**************************************************************************/
void usart2_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //使能GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //使能USART时钟

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          //输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //高速50MHZ
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //初始化

    //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4 ;
    //子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //IRQ通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //根据指定的参数初始化VIC寄存器
    NVIC_Init(&NVIC_InitStructure);

    //初始化设置
    USART_InitStructure.USART_BaudRate = bound; //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART1, &USART_InitStructure); //初始化串口1

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启串口接受中断
    USART_Cmd(USART1, ENABLE);                     //使能串口1
}
/**************************************************************************
函数功能：串口3初始化
入口参数：无
返回  值：无
**************************************************************************/
void usart3_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	 //使能GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //使能USART时钟

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          //输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //高速50MHZ
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          //上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);  		          //初始化

    //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ;
    //子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //IRQ通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //根据指定的参数初始化VIC寄存器
    NVIC_Init(&NVIC_InitStructure);

    //初始化设置
    USART_InitStructure.USART_BaudRate = bound; //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //一个停止
    USART_InitStructure.USART_Parity = USART_Parity_No; //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART3, &USART_InitStructure);      //初始化串口3

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //开启串口接受中断
    USART_Cmd(USART3, ENABLE);                     //使能串口3
}
/**************************************************************************
函数功能：串口4初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart4_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //使能GPIOC时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //使能USART时钟

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          //输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //高速50MHZ
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          //上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //初始化

    //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4 ;
    //子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //IRQ通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //根据指定的参数初始化VIC寄存器
    NVIC_Init(&NVIC_InitStructure);

    //初始化设置
    USART_InitStructure.USART_BaudRate = bound; //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止
    USART_InitStructure.USART_Parity = USART_Parity_No; //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(UART4, &USART_InitStructure);      //初始化串口2

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);  //开启串口接受中断
    USART_Cmd(UART4, ENABLE);                      //使能串口2
}

/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
函数功能：串口1接收中断
入口参数：无
返 回 值：无
**************************************************************************/
int USART1_IRQHandler(void)
{
    return 0;
}
/**************************************************************************
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART2_IRQHandler(void)
{
    return 0;
}
/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART3_IRQHandler(void)
{
    static u8 Count = 0;
    u8 CAN_SENT[8];

    // 检查接收中断是否触发以及接收是否被允许
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        Receive_Data.buffer[Count] = USART_ReceiveData(USART3); // 存储接收到的数据

        // 检查接收到的数据是否为帧头，或当前计数大于0
        if (Receive_Data.buffer[Count] == FRAME_HEADER || Count > 0)
        {
            // 验证数据包的帧尾
            if (Receive_Data.buffer[Count] == FRAME_TAIL)
            {
                // 根据标志位
                if (Receive_Data.buffer[1] == 0x11)
                {
                    Receive_Data.Receive_Str.Instruction = Receive_Data.buffer[1];
                }
                else if (Receive_Data.buffer[1] == 0x22)
                {
                    Receive_Data.Receive_Str.A_speed = Receive_Data.buffer[1];
                    Receive_Data.Receive_Str.B_speed = Receive_Data.buffer[2];
                }
                else if(Receive_Data.buffer[1] == 0x88)
                {
                    for(int i = 2; i < Count; i++)
                    {
                        CAN_SENT[i - 2] = Receive_Data.buffer[i];
                    }

                    CAN1_Send_Num(0x600, CAN_SENT);

                    for(int i = 0; i < 8; i++)usart3_send(CAN_SENT[i]);
                }

                Count = 0; // 重置计数
            }
            else
            {
                Count++; // 增加计数
            }
        }
        else
        {
            Count = 0; // 重置计数
        }
    }

    return Receive_Data.buffer[1];
}

/**************************************************************************
函数功能：串口4接收中断
入口参数：无
返回  值：无
**************************************************************************/
int UART4_IRQHandler(void)
{
    return 0;
}
/**************************************************************************
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
    USART1->DR = data;

    while((USART1->SR & 0x40) == 0);
}
/**************************************************************************
函数功能：串口2发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data)
{
    USART2->DR = data;

    while((USART2->SR & 0x40) == 0);
}
/**************************************************************************
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart3_send(u8 data)
{
    USART3->DR = data;

    while((USART3->SR & 0x40) == 0);
}
/**************************************************************************
函数功能：串口4发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void uart4_send(u8 data)
{
    UART4->DR = data;

    while((UART4->SR & 0x40) == 0);
}
/**************************************************************************
函数功能：计算要发送的数据校验结果
入口参数：无
返回  值：校验结果低8位
**************************************************************************/
u8 Check_Sum(void)
{
    unsigned char check_sum = 0, k;

    //对要发送的数据进行校验
    for(k = 0; k < 12 ; k++)
    {
        check_sum += Send_Data.buffer[k + 18];
    }

    check_sum = check_sum & 0xFF;

    return check_sum;
}



