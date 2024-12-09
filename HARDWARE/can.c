#include "can.h"
#include "system.h"

/**************************************************************************
�������ܣ�CAN1��ʼ��
��ڲ�����tsjw������ͬ����Ծʱ�䵥Ԫ����Χ:1~3;
 		  tbs2��ʱ���2��ʱ�䵥Ԫ����Χ:1~8;
 		  tbs1��ʱ���1��ʱ�䵥Ԫ����Χ:1~16;
 		  brp �������ʷ�Ƶ������Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
 		  mode��0,��ͨģʽ;1,�ػ�ģʽ;
����  ֵ��0-��ʼ���ɹ�; ����-��ʼ��ʧ��
ע�⣺��ڲ���(����mode)������Ϊ0
������/Baud rate=Fpclk1/((tbs1+tbs2+1)*brp)��Fpclk1Ϊ36M
                =42M/((3+2+1)*6)
				=1M
**************************************************************************/
u8 CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    u16 i = 0;

    if(tsjw == 0 || tbs2 == 0 || tbs1 == 0 || brp == 0) return 1; // ������Ч�����س�ʼ��ʧ��

    tsjw -= 1; // �ȼ�ȥ1������������
    tbs2 -= 1;
    tbs1 -= 1;
    brp  -= 1;

    // ʹ�����ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // ʹ��PORTAʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); // ʹ��CAN1ʱ��
    // ��ʼ��GPIO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // ���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // ����
    GPIO_Init(GPIOA, &GPIO_InitStructure); // ��ʼ��PA11 PA12

    // ���Ÿ���ӳ������
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1); // GPIOA11����ΪCAN1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); // GPIOA12����ΪCAN1

    CAN1->MCR = 0x0000; // �˳�˯��ģʽ(ͬʱ��������λΪ0)
    CAN1->MCR |= 1 << 0; // ����CAN1�����ʼ��ģʽ

    while((CAN1->MSR & 1 << 0) == 0)
    {
        i++;

        if(i > 100) return 2; // �����ʼ��ģʽʧ��
    }

    // ��ʱ�䴥��ͨ��ģʽ
    CAN1->MCR |= 0 << 7;
    // ����Զ����߹���
    CAN1->MCR |= 0 << 6;
    // ˯��ģʽͨ���������(���CAN1->MCR��SLEEPλ)
    CAN1->MCR |= 0 << 5;
    // ��ֹ�����Զ�����
    CAN1->MCR |= 1 << 4;
    // ���Ĳ�����,�µĸ��Ǿɵ�
    CAN1->MCR |= 0 << 3;
    // ���ȼ��ɱ��ı�ʶ������
    CAN1->MCR |= 0 << 2;
    // ���ԭ��������
    CAN1->BTR = 0x00000000;
    // ģʽ���� 0,��ͨģʽ;1,�ػ�ģʽ;
    CAN1->BTR |= mode << 30;
    // ����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ
    CAN1->BTR |= tsjw << 24;
    // Tbs2=tbs2+1��ʱ�䵥λ
    CAN1->BTR |= tbs2 << 20;
    // Tbs1=tbs1+1��ʱ�䵥λ
    CAN1->BTR |= tbs1 << 16;
    // ��Ƶϵ��(Fdiv)Ϊbrp+1��������:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
    CAN1->BTR |= brp << 0;
    // ����CAN1�˳���ʼ��ģʽ
    CAN1->MCR &= ~(1 << 0);

    while((CAN1->MSR & 1 << 0) == 1)
    {
        i++;

        if(i > 0XFFF0) return 3; // �˳���ʼ��ģʽʧ��
    }

    /***��������ʼ�� ***/
    // �������鹤���ڳ�ʼ��ģʽ
    CAN1->FMR |= 1 << 0;
    // ������0������
    CAN1->FA1R &= ~(1 << 0);
    // ������λ��Ϊ32λ
    CAN1->FS1R |= 1 << 0;
    // ������0�����ڱ�ʶ������λģʽ
    CAN1->FM1R |= 0 << 0;
    // ������0������FIFO0
    CAN1->FFA1R |= 0 << 0;
    // 32λID
    CAN1->sFilterRegister[0].FR1 = 0X00000000;
    // 32λMASK
    CAN1->sFilterRegister[0].FR2 = 0X00000000;
    // ���������0
    CAN1->FA1R |= 1 << 0;
    // ���������������ģʽ
    CAN1->FMR &= 0 << 0;

    CAN_FilterInitStructure.CAN_FilterNumber = 1; // ָ��������Ϊ1
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; // ָ��������Ϊ��ʶ������λģʽ
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; // ������λ��Ϊ32λ
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000; // ��������ʶ���ĸ�16λֵ
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000; // ��������ʶ���ĵ�16λֵ
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; // ���������α�ʶ���ĸ�16λֵ
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000; // ���������α�ʶ���ĵ�16λֵ
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0; // ���ù���������FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; // ʹ�ܹ�����
    CAN_FilterInit(&CAN_FilterInitStructure); // ������������ʼ��������

    #if CAN1_RX0_INT_ENABLE
    // ʹ�ܽ����ж�
    CAN1->IER |= 1 << 1; // FIFO0��Ϣ�Һ��ж�����

    // ����CAN�����ж�
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    // IRQͨ��ʹ��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // ��ʼ��VIC�Ĵ���
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    #endif
    return 0;
}

/**************************************************************************
�������ܣ�CAN��������
��ڲ�����id:��׼ID(11λ)/��չID(11λ+18λ)
			    ide:0,��׼֡;1,��չ֡
			    rtr:0,����֡;1,Զ��֡
			    len:Ҫ���͵����ݳ���(�̶�Ϊ8���ֽ�,��ʱ�䴥��ģʽ��,��Ч����Ϊ6���ֽ�)
			    *dat:����ָ��.
����  ֵ��0~3,������.0XFF,����Ч����
**************************************************************************/
u8 CAN1_Tx_Msg(u32 id, u8 ide, u8 rtr, u8 len, u8 *dat)
{
    u8 mbox;

    if(CAN1->TSR & (1 << 26)) mbox = 0; // ����0Ϊ��
    else if(CAN1->TSR & (1 << 27)) mbox = 1; // ����1Ϊ��
    else if(CAN1->TSR & (1 << 28)) mbox = 2; // ����2Ϊ��
    else return 0xFF; // û�п�����

    // ����ID
    if(ide)
    {
        CAN1->sTxMailBox[mbox].TIR = (id << 3) | 1 << 2 | rtr;
    }
    else
    {
        CAN1->sTxMailBox[mbox].TIR = (id << 21) | rtr;
    }

    // �������ݳ���
    CAN1->sTxMailBox[mbox].TDTR = len;

    // д������
    CAN1->sTxMailBox[mbox].TDLR = ((u32)dat[3] << 24) | ((u32)dat[2] << 16) | ((u32)dat[1] << 8) | ((u32)dat[0]);
    CAN1->sTxMailBox[mbox].TDHR = ((u32)dat[7] << 24) | ((u32)dat[6] << 16) | ((u32)dat[5] << 8) | ((u32)dat[4]);

    // ������Ϣ
    CAN1->sTxMailBox[mbox].TIR |= (1 << 0); // ��������

    return mbox; // ����������
}
/**************************************************************************
�������ܣ���÷���״̬
��ڲ�����mbox��������
����  ֵ��0,����;0X05,����ʧ��;0X07,���ͳɹ�
**************************************************************************/
u8 CAN1_Tx_Staus(u8 mbox)
{
    u8 sta = 0;

    switch (mbox)
    {
        case 0:
            sta |= CAN1->TSR & (1 << 0);			 // RQCP0��������ɱ�־
            sta |= CAN1->TSR & (1 << 1);			 // TXOK0�����ͳɹ���־
            sta |= ((CAN1->TSR & (1 << 26)) >> 24); // TME0������0���б�־
            break;

        case 1:
            sta |= CAN1->TSR & (1 << 8) >> 8;		 // RQCP1
            sta |= CAN1->TSR & (1 << 9) >> 8;		 // TXOK1
            sta |= ((CAN1->TSR & (1 << 27)) >> 25); // TME1
            break;

        case 2:
            sta |= CAN1->TSR & (1 << 16) >> 16;	 // RQCP2
            sta |= CAN1->TSR & (1 << 17) >> 16;	 // TXOK2
            sta |= ((CAN1->TSR & (1 << 28)) >> 26); // TME2
            break;

        default:
            sta = 0X05; // �������Ų��ԣ�����ʧ��
            break;
    }

    return sta;
}

/**************************************************************************
�������ܣ��õ���FIFO0/FIFO1�н��յ��ı��ĸ���
��ڲ�����fifox��FIFO��ţ�0��1��
����  ֵ��FIFO0/FIFO1�еı��ĸ���
**************************************************************************/
u8 CAN1_Msg_Pend(u8 fifox)
{
    if(fifox == 0)
        return CAN1->RF0R & 0x03; // ��ȡFIFO0�еı��ĸ���
    else if(fifox == 1)
        return CAN1->RF1R & 0x03; // ��ȡFIFO1�еı��ĸ���
    else
        return 0; // FIFO��Ų���ȷ
}

/**************************************************************************
�������ܣ���������
��ڲ�����fifox�������
		id:��׼ID(11λ)/��չID(11λ+18λ)
		ide:0,��׼֡;1,��չ֡
		rtr:0,����֡;1,Զ��֡
		len:���յ������ݳ���(�̶�Ϊ8���ֽ�,��ʱ�䴥��ģʽ��,��Ч����Ϊ6���ֽ�)
		dat:���ݻ�����
����  ֵ����
**************************************************************************/
void CAN1_Rx_Msg(u8 fifox, u32 *id, u8 *ide, u8 *rtr, u8 *len, u8 *dat)
{
    *ide = CAN1->sFIFOMailBox[fifox].RIR & 0x04; // �õ���ʶ��ѡ��λ��ֵ

    if(*ide == 0) // ����Ǳ�׼��ʶ��
    {
        *id = CAN1->sFIFOMailBox[fifox].RIR >> 21; // ��ȡ��׼ID
    }
    else	      // ��չ��ʶ��
    {
        *id = CAN1->sFIFOMailBox[fifox].RIR >> 3; // ��ȡ��չID
    }

    *rtr = CAN1->sFIFOMailBox[fifox].RIR & 0x02;   // ��ȡԶ�̷�������ֵ
    *len = CAN1->sFIFOMailBox[fifox].RDTR & 0x0F; // ��ȡDLC�����ݳ����룩

    // �������ݲ����뻺��
    dat[0] = CAN1->sFIFOMailBox[fifox].RDLR & 0XFF;
    dat[1] = (CAN1->sFIFOMailBox[fifox].RDLR >> 8) & 0XFF;
    dat[2] = (CAN1->sFIFOMailBox[fifox].RDLR >> 16) & 0XFF;
    dat[3] = (CAN1->sFIFOMailBox[fifox].RDLR >> 24) & 0XFF;
    dat[4] = CAN1->sFIFOMailBox[fifox].RDHR & 0XFF;
    dat[5] = (CAN1->sFIFOMailBox[fifox].RDHR >> 8) & 0XFF;
    dat[6] = (CAN1->sFIFOMailBox[fifox].RDHR >> 16) & 0XFF;
    dat[7] = (CAN1->sFIFOMailBox[fifox].RDHR >> 24) & 0XFF;

    if(fifox == 0)
        CAN1->RF0R |= 0X20;  // �ͷ�FIFO0����
    else if(fifox == 1)
        CAN1->RF1R |= 0X20; // �ͷ�FIFO1����
}

/**************************************************************************
�������ܣ�CAN�����жϷ���������������
��ڲ�������
����  ֵ����
**************************************************************************/
#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
void CAN1_RX0_IRQHandler(void)
{
    u32 id;
    u8 ide, rtr, len, i;
    u8 temp_rxbuf[8];
    command_lost_count = 0;

    CAN1_Rx_Msg(0, &id, &ide, &rtr, &len, temp_rxbuf);

    if(temp_rxbuf[0] == 0x60 && temp_rxbuf[1] != 0x00 && temp_rxbuf[2] != 0x00)
    {
        for (i = 0; i < 8; i++)
        {
            usart3_send(temp_rxbuf[i]);
        }
    }

//    if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0)  // FIFO0��Ϊ��
//    {
//        CAN1_Rx_Msg(0, &id, &ide, &rtr, &len, temp_rxbuf);
//    }

//    else if((CAN1->RF1R & CAN_RF1R_FMP1) != 0) //
//    {
//        CAN1_Rx_Msg(1, &id, &ide, &rtr, &len, temp_rxbuf);
//    }

//    if(temp_rxbuf[0] == 0x60 && temp_rxbuf[1] == 0x03 && temp_rxbuf[2] == 0x02)
//    {
//        Receive_Data.Receive_Str.A_speed = (temp_rxbuf[5] << 8) + temp_rxbuf[4];
//    }
//    else if(temp_rxbuf[0] == 0x60 && temp_rxbuf[1] == 0x04 && temp_rxbuf[2] == 0x02)
//    {
//        Receive_Data.Receive_Str.B_speed = (temp_rxbuf[5] << 8) + temp_rxbuf[4];
//    }
}
#endif

/**************************************************************************
�������ܣ�CAN1����һ������(�̶���ʽ:IDΪ0X601,��׼֡,����֡)
��ڲ�����msg:����ָ��
    	len:���ݳ���(���Ϊ8)
����  ֵ��0,�ɹ�������,ʧ�ܣ�
**************************************************************************/
u8 CAN1_Send_Msg(u8* msg, u8 len)
{
    u8 mbox;
    u16 i = 0;
    mbox = CAN1_Tx_Msg(0X601, 0, 0, len, msg); // ������Ϣ

    // �ȴ����ͽ���
    while((CAN1_Tx_Staus(mbox) != 0X07) && (i < 0XFFF)) i++;

    if(i >= 0XFFF) return 1; // ����ʧ��

    return 0; // ���ͳɹ�
}

/**************************************************************************
�������ܣ�CAN1�ڽ������ݲ�ѯ
��ڲ�����buf:���ݻ�����
����  ֵ��0,�����ݱ��յ�������,���յ����ݳ���
**************************************************************************/
u8 CAN1_Receive_Msg(u8 *buf)
{
    u32 id;
    u8 ide, rtr, len;

    // ���FIFO0�Ƿ�������
    if(CAN1_Msg_Pend(0) == 0)
        return 0; // û�н��յ����ݣ�ֱ�ӷ���

    // �������ݲ����뻺��
    CAN1_Rx_Msg(0, &id, &ide, &rtr, &len, buf);

    // ���յ�����ʱ�����ID��֡�����Ƿ����Ԥ��
    if(id != 0x601 || ide != 0 || rtr != 0)
        len = 0; // ������յ������ݲ�����Ҫ���򳤶���Ϊ0

    return len; // ���ؽ��յ������ݳ���
}

/**************************************************************************
�������ܣ�CAN1����һ�����ݲ���
��ڲ�����msg:����ָ��
			    len:���ݳ���(���Ϊ8)
����  ֵ��0,�ɹ���1,ʧ��
**************************************************************************/
u8 CAN1_Send_MsgTEST(u8* msg, u8 len)
{
    u8 mbox;
    u16 i = 0;
    mbox = CAN1_Tx_Msg(0X701, 0, 0, len, msg); // ���Ͳ�����Ϣ

    while((CAN1_Tx_Staus(mbox) != 0X07) && (i < 0XFFF)) i++; // �ȴ����ͽ���

    if(i >= 0XFFF) return 1; // ����ʧ��

    return 0; // ���ͳɹ�
}

/**************************************************************************
�������ܣ�������ID����һ�����������
��ڲ�����id��ID��
         msg�������͵�����ָ��
����  ֵ��0,�ɹ���1,ʧ��
**************************************************************************/
u8 CAN1_Send_Num(u32 id, u8* msg)
{
    u8 mbox;
    u16 i = 0;
    mbox = CAN1_Tx_Msg(id, 0, 0, 8, msg); // ����ָ��ID������

    while((CAN1_Tx_Staus(mbox) != 0X07) && (i < 0XFFF)) i++; // �ȴ����ͽ���

    if(i >= 0XFFF) return 1; // ����ʧ��

    return 0; // ���ͳɹ�
}

/**************************************************************************
�������ܣ�������չID������
��ڲ�����id����չID��
         msg������ָ��
����  ֵ��0,�ɹ���1,ʧ��
**************************************************************************/
u8 CAN1_Send_E_Num(u32 id, u8* msg)
{
    u8 mbox;
    u16 i = 0;
    mbox = CAN1_Tx_Msg(id, 1, 0, 8, msg); // ������չID������

    while((CAN1_Tx_Staus(mbox) != 0X07) && (i < 0XFFF)) i++; // �ȴ����ͽ���

    if(i >= 0XFFF) return 1; // ����ʧ��

    return 0; // ���ͳɹ�
}
