#include "usart.h"
#include "sys.h"
#include "string.h"
#include "system.h"

u8 SBUS[25];
u16 sbus[16];

//////////////////////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
    #include "FreeRTOS.h"					//FreeRTOS使用
#endif


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while((USART1->SR & 0X40) == 0); //循环发送,直到发送完毕

    USART1->DR = (u8) ch;
    return ch;
}
#endif

#if EN_UART5_RX   //如果使能了接收
//串口5中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误
u8 UART5_RX_BUF[UART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 UART5_RX_STA = 0;     //接收状态标记

//初始化IO 串口5
//bound:波特率

void uart5_init(u32 bound)
{
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;//定义io初始化结构体
    USART_InitTypeDef USART_InitStructure;//定义串口结构体
    NVIC_InitTypeDef NVIC_InitStructure;//定义中断结构体

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //使能GPIOD 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);//使能USART5时钟

    //串口5对应引脚复用映射
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5); //GPIOD2复用为USART5

    //USART端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOD2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure); //初始化PD2

    //UART 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;//字长为9位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_2;//两个停止位
    USART_InitStructure.USART_Parity = USART_Parity_Even ;//偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx;	//接收模式
    USART_Init(UART5, &USART_InitStructure); //初始化串口5

    USART_Cmd(UART5, ENABLE);//使能串口5

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启相关中断

    //Usart5 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;//串口5中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}
#endif

void UART5_IRQHandler(void)
{
    int i;

    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        uint8_t receivedData = USART_ReceiveData(UART5);

        if (UART5_RX_STA == 0 && receivedData != 0X0F)
        {
            // 帧头不对，丢掉
            return; // 直接返回，不再继续处理此数据
        }

        UART5_RX_BUF[UART5_RX_STA] = receivedData;
        UART5_RX_STA++;

        if (UART5_RX_STA > 25)
        {
            // 接收数据错误，重新开始接收
            UART5_RX_STA = 0;
        }

        if (UART5_RX_BUF[0] == 0x0F && UART5_RX_BUF[24] == 0x00 && UART5_RX_STA == 25)//接受完一帧数据

        {
            for (i = 0; i < 25; i++)
            {
                // 清空缓存区
                SBUS[i] = UART5_RX_BUF[i];
                UART5_RX_BUF[i] = 0;

            }

            UART5_RX_STA = 0;

            update_sbus(sbus, SBUS);

            if (Variable_detection(8, sbus[8]))
            {
                if  (sbus[8] > 1000)
                {
                    usart3_send(0x01);
                    usart3_send(0x55);
                    usart3_send(0x80);
                    SBUS_ON_Flag = 1;
                }
                else
                {
                    usart3_send(0x01);
                    usart3_send(0x44);
                    usart3_send(0x80);
                    SBUS_ON_Flag = 0;
                }

                Send_Data.Send_Str.A_speed = 0;
                Send_Data.Send_Str.B_speed = 0;
                TIM_SetCompare1(TIM2, 0);
                memset((void *)Send_Data.Send_Str.R_status, 0, sizeof(Send_Data.Send_Str.R_status));

            }

            for (i = 0; i < 25; i++)
            {
                SBUS[i] = 0;
            }
        }
    }
}

void update_sbus(u16 *CH, u8 *buf)
{
    CH[ 0] = ((int16_t)SBUS[ 1] >> 0 | ((int16_t)SBUS[ 2] << 8 )) & 0x07FF;
    CH[ 1] = ((int16_t)SBUS[ 2] >> 3 | ((int16_t)SBUS[ 3] << 5 )) & 0x07FF;
    CH[ 2] = ((int16_t)SBUS[ 3] >> 6 | ((int16_t)SBUS[ 4] << 2 ) | (int16_t)SBUS[ 5] << 10 ) & 0x07FF;
    CH[ 3] = ((int16_t)SBUS[ 5] >> 1 | ((int16_t)SBUS[ 6] << 7 )) & 0x07FF;
    CH[ 4] = ((int16_t)SBUS[ 6] >> 4 | ((int16_t)SBUS[ 7] << 4 )) & 0x07FF;
    CH[ 5] = ((int16_t)SBUS[ 7] >> 7 | ((int16_t)SBUS[ 8] << 1 ) | (int16_t)SBUS[9] << 9 ) & 0x07FF;
    CH[ 6] = ((int16_t)SBUS[ 9] >> 2 | ((int16_t)SBUS[10] << 6 )) & 0x07FF;
    CH[ 7] = ((int16_t)SBUS[10] >> 5 | ((int16_t)SBUS[11] << 3 )) & 0x07FF;
    CH[ 8] = ((int16_t)SBUS[12] << 0 | ((int16_t)SBUS[13] << 8 )) & 0x07FF;
    CH[ 9] = ((int16_t)SBUS[13] >> 3 | ((int16_t)SBUS[14] << 5 )) & 0x07FF;
    CH[10] = ((int16_t)SBUS[14] >> 6 | ((int16_t)SBUS[15] << 2 ) | (int16_t)SBUS[16] << 10 ) & 0x07FF;
    CH[11] = ((int16_t)SBUS[16] >> 1 | ((int16_t)SBUS[17] << 7 )) & 0x07FF;
    CH[12] = ((int16_t)SBUS[17] >> 4 | ((int16_t)SBUS[18] << 4 )) & 0x07FF;
    CH[13] = ((int16_t)SBUS[18] >> 7 | ((int16_t)SBUS[19] << 1 ) | (int16_t)SBUS[20] << 9 ) & 0x07FF;
    CH[14] = ((int16_t)SBUS[20] >> 2 | ((int16_t)SBUS[21] << 6 )) & 0x07FF;
    CH[15] = ((int16_t)SBUS[21] >> 5 | ((int16_t)SBUS[22] << 3 )) & 0x07FF;
}

//转换sbus信号通道值
signed char sbus_to(u16 sbus_value)
{
    short s = sbus_value - SBUS_RANGE_MIDDLE;

    if (s > -SBUS_DEAD_RANGE && s < SBUS_DEAD_RANGE) s = 0;

    signed char k = (signed char)(s * SBUS_SCALE_FACTOR);

    if (k > SBUS_TARGET_RANGE) k = SBUS_TARGET_RANGE;

    if (k < -SBUS_TARGET_RANGE) k = -SBUS_TARGET_RANGE;

    return k;
}
