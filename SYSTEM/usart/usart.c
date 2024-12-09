#include "usart.h"
#include "sys.h"
#include "string.h"
#include "system.h"

u8 SBUS[25];
u16 sbus[16];

//////////////////////////////////////////////////////////////////////////////////
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
    #include "FreeRTOS.h"					//FreeRTOSʹ��
#endif


//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;
};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
    x = x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while((USART1->SR & 0X40) == 0); //ѭ������,ֱ���������

    USART1->DR = (u8) ch;
    return ch;
}
#endif

#if EN_UART5_RX   //���ʹ���˽���
//����5�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���
u8 UART5_RX_BUF[UART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 UART5_RX_STA = 0;     //����״̬���

//��ʼ��IO ����5
//bound:������

void uart5_init(u32 bound)
{
    //GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;//����io��ʼ���ṹ��
    USART_InitTypeDef USART_InitStructure;//���崮�ڽṹ��
    NVIC_InitTypeDef NVIC_InitStructure;//�����жϽṹ��

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //ʹ��GPIOD ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);//ʹ��USART5ʱ��

    //����5��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5); //GPIOD2����ΪUSART5

    //USART�˿�����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOD2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
    GPIO_Init(GPIOD, &GPIO_InitStructure); //��ʼ��PD2

    //UART ��ʼ������
    USART_InitStructure.USART_BaudRate = bound;//����������
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;//�ֳ�Ϊ9λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_2;//����ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_Even ;//żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx;	//����ģʽ
    USART_Init(UART5, &USART_InitStructure); //��ʼ������5

    USART_Cmd(UART5, ENABLE);//ʹ�ܴ���5

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//��������ж�

    //Usart5 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;//����5�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
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
            // ֡ͷ���ԣ�����
            return; // ֱ�ӷ��أ����ټ������������
        }

        UART5_RX_BUF[UART5_RX_STA] = receivedData;
        UART5_RX_STA++;

        if (UART5_RX_STA > 25)
        {
            // �������ݴ������¿�ʼ����
            UART5_RX_STA = 0;
        }

        if (UART5_RX_BUF[0] == 0x0F && UART5_RX_BUF[24] == 0x00 && UART5_RX_STA == 25)//������һ֡����

        {
            for (i = 0; i < 25; i++)
            {
                // ��ջ�����
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

//ת��sbus�ź�ͨ��ֵ
signed char sbus_to(u16 sbus_value)
{
    short s = sbus_value - SBUS_RANGE_MIDDLE;

    if (s > -SBUS_DEAD_RANGE && s < SBUS_DEAD_RANGE) s = 0;

    signed char k = (signed char)(s * SBUS_SCALE_FACTOR);

    if (k > SBUS_TARGET_RANGE) k = SBUS_TARGET_RANGE;

    if (k < -SBUS_TARGET_RANGE) k = -SBUS_TARGET_RANGE;

    return k;
}
