#include "usartx.h"
#include "system.h"

volatile u8 UART3_Flag = 0;
extern int Time_count;

SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
/*************************************************************************
�������ܣ����ڣ�CAN������������
��ڲ�������
����  ֵ����
**************************************************************************/
void data_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while(1)
    {
        //��������50Hz��Ƶ������(20ms)
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));
        //��Ҫ���з��͵����ݽ��и�ֵ
        data_transition();
        //������������
        CAN_SEND();           //CAN��������
        //CAN_SEND1();          //CAN��ȡ������
        //USART3_SEND();        //����3��������->ROS
        //USART1_SEND();        //����1��������->PC
        xEventGroupSetBits(EventGroupHandler, EVENTBIT3);
    }
}
/**************************************************************************
�������ܣ����ڷ��͵����ݽ��и�ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void data_transition(void)
{
    //canͨ��
    Send_Data.buffer[0] = 0x2B;
    Send_Data.buffer[1] = 0x00;
    Send_Data.buffer[2] = 0x00;
    Send_Data.buffer[3] = 0x00;
    Send_Data.buffer[4] = Send_Data.Send_Str.A_speed;
    Send_Data.buffer[5] = Send_Data.Send_Str.B_speed;
    Send_Data.buffer[6] = 0x00;
    Send_Data.buffer[7] = 0x00;

    //�̵�����·����ָ��
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

    //�̵������п���ָ��
    Send_Data.buffer[18] = 0x48;
    Send_Data.buffer[19] = 0x3a;
    Send_Data.buffer[20] = 0x01;
    Send_Data.buffer[21] = 0x57;
    //�̵���CAN����
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

    //��A��B���ת��
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
�������ܣ�����1��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART1_SEND(void)
{


}
/**************************************************************************
�������ܣ�����2��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART2_SEND(void)
{

}
/**************************************************************************
�������ܣ�����3��������
��ڲ�������
����  ֵ����
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
�������ܣ�����4��������
��ڲ�������
����  ֵ����
**************************************************************************/
void UART4_SEND(void)
{

}
/**************************************************************************
�������ܣ�CAN��������
��ڲ�������
�� �� ֵ����
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
�������ܣ�����1��ʼ��
��ڲ�������
�� �� ֵ����
**************************************************************************/
void usart1_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //ʹ��GPIOʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //ʹ��USARTʱ��

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          //���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //����50MHZ
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);                //��ʼ��

    //UsartNVIC����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;
    //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //IRQͨ��ʹ��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //����ָ���Ĳ�����ʼ��VIC�Ĵ���
    NVIC_Init(&NVIC_InitStructure);

    //��ʼ������
    USART_InitStructure.USART_BaudRate = bound; //���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No; //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure); //��ʼ������1

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //�������ڽ����ж�
    USART_Cmd(USART1, ENABLE);                     //ʹ�ܴ���1
}
/**************************************************************************
�������ܣ�����2��ʼ��
��ڲ�������
�� �� ֵ����
**************************************************************************/
void usart2_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //ʹ��GPIOʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //ʹ��USARTʱ��

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          //���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //����50MHZ
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          //����
    GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //��ʼ��

    //UsartNVIC����
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4 ;
    //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //IRQͨ��ʹ��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //����ָ���Ĳ�����ʼ��VIC�Ĵ���
    NVIC_Init(&NVIC_InitStructure);

    //��ʼ������
    USART_InitStructure.USART_BaudRate = bound; //���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No; //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART1, &USART_InitStructure); //��ʼ������1

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //�������ڽ����ж�
    USART_Cmd(USART1, ENABLE);                     //ʹ�ܴ���1
}
/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void usart3_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	 //ʹ��GPIOʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //ʹ��USARTʱ��

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          //���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //����50MHZ
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          //����
    GPIO_Init(GPIOB, &GPIO_InitStructure);  		          //��ʼ��

    //UsartNVIC����
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ;
    //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //IRQͨ��ʹ��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //����ָ���Ĳ�����ʼ��VIC�Ĵ���
    NVIC_Init(&NVIC_InitStructure);

    //��ʼ������
    USART_InitStructure.USART_BaudRate = bound; //���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //һ��ֹͣ
    USART_InitStructure.USART_Parity = USART_Parity_No; //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART3, &USART_InitStructure);      //��ʼ������3

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //�������ڽ����ж�
    USART_Cmd(USART3, ENABLE);                     //ʹ�ܴ���3
}
/**************************************************************************
�������ܣ�����4��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart4_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //ʹ��GPIOCʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //ʹ��USARTʱ��

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          //���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //����50MHZ
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          //����
    GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //��ʼ��

    //UsartNVIC����
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4 ;
    //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //IRQͨ��ʹ��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //����ָ���Ĳ�����ʼ��VIC�Ĵ���
    NVIC_Init(&NVIC_InitStructure);

    //��ʼ������
    USART_InitStructure.USART_BaudRate = bound; //���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  //һ��ֹͣ
    USART_InitStructure.USART_Parity = USART_Parity_No; //����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(UART4, &USART_InitStructure);      //��ʼ������2

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);  //�������ڽ����ж�
    USART_Cmd(UART4, ENABLE);                      //ʹ�ܴ���2
}

/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
�������ܣ�����1�����ж�
��ڲ�������
�� �� ֵ����
**************************************************************************/
int USART1_IRQHandler(void)
{
    return 0;
}
/**************************************************************************
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART2_IRQHandler(void)
{
    return 0;
}
/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART3_IRQHandler(void)
{
    static u8 Count = 0;
    u8 CAN_SENT[8];

    // �������ж��Ƿ񴥷��Լ������Ƿ�����
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        Receive_Data.buffer[Count] = USART_ReceiveData(USART3); // �洢���յ�������

        // �����յ��������Ƿ�Ϊ֡ͷ����ǰ��������0
        if (Receive_Data.buffer[Count] == FRAME_HEADER || Count > 0)
        {
            // ��֤���ݰ���֡β
            if (Receive_Data.buffer[Count] == FRAME_TAIL)
            {
                // ���ݱ�־λ
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

                Count = 0; // ���ü���
            }
            else
            {
                Count++; // ���Ӽ���
            }
        }
        else
        {
            Count = 0; // ���ü���
        }
    }

    return Receive_Data.buffer[1];
}

/**************************************************************************
�������ܣ�����4�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int UART4_IRQHandler(void)
{
    return 0;
}
/**************************************************************************
�������ܣ�����1��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart1_send(u8 data)
{
    USART1->DR = data;

    while((USART1->SR & 0x40) == 0);
}
/**************************************************************************
�������ܣ�����2��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart2_send(u8 data)
{
    USART2->DR = data;

    while((USART2->SR & 0x40) == 0);
}
/**************************************************************************
�������ܣ�����3��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart3_send(u8 data)
{
    USART3->DR = data;

    while((USART3->SR & 0x40) == 0);
}
/**************************************************************************
�������ܣ�����4��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void uart4_send(u8 data)
{
    UART4->DR = data;

    while((UART4->SR & 0x40) == 0);
}
/**************************************************************************
�������ܣ�����Ҫ���͵�����У����
��ڲ�������
����  ֵ��У������8λ
**************************************************************************/
u8 Check_Sum(void)
{
    unsigned char check_sum = 0, k;

    //��Ҫ���͵����ݽ���У��
    for(k = 0; k < 12 ; k++)
    {
        check_sum += Send_Data.buffer[k + 18];
    }

    check_sum = check_sum & 0xFF;

    return check_sum;
}



