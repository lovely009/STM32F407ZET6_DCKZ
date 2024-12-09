#include "show.h"

u8 oled_page = 0;//��Ļʹ��
u8 oled_refresh_flag;
extern int Time_count;
extern u8 QB_UP_Flag;
extern SEND_DATA Send_Data;
/**************************************************************************
�������ܣ������������������Լ졢��APP�������ݡ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
int Buzzer_count = 25;
void show_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while(1)
    {
        static int LowVoltage_1 = 0, LowVoltage_2 = 0;
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //��������10Hz��Ƶ������

        //����ʱ���������ݷ�������������
        if(Time_count < 50)Buzzer = 1;
        else if(Time_count >= 51 && Time_count < 100)Buzzer = 0;

        if(LowVoltage_1 == 1 || LowVoltage_2 == 1)Buzzer_count = 0;

        if(Buzzer_count < 5)Buzzer_count++;

        if(Buzzer_count < 5)Buzzer = 1; //����������
        else if(Buzzer_count == 5)Buzzer = 0;

        if(LowVoltage_1 == 1)LowVoltage_1++; //ȷ��������ֻ��0.5��

        if(LowVoltage_2 == 1)LowVoltage_2++; //ȷ��������ֻ��0.5��

        //APP_Show();	 //��APP��������

        if(oled_refresh_flag) OLED_Clear(), oled_refresh_flag = 0;
        else oled_show(); //��ʾ����ʾ����
        
        xEventGroupSetBits(EventGroupHandler, EVENTBIT1);
    }
}

/**************************************************************************
�������ܣ�OLED��ʾ����ʾ����
��ڲ �������
����  ֵ����
**************************************************************************/
void oled_show(void)
{
    if(oled_page == 1)
    {
        //��ʾ����1����ʾ����//
        //��ʾ���A�ĵ�ǰ�ٶ�
        OLED_ShowString(0, 00, "A");

        if(Send_Data.Send_Str.A_speed > 0) OLED_ShowString(15, 00, "-"), OLED_ShowNumber(20, 00, Send_Data.Send_Str.A_speed, 5, 12);
        else                               OLED_ShowString(15, 00, "+"), OLED_ShowNumber(20, 00, -Send_Data.Send_Str.A_speed, 5, 12);


        //��ʾ����2����ʾ����//
        //��ʾ���B�ĵ�ǰ�ٶ�
        OLED_ShowString(0, 10, "B");

        if( Send_Data.Send_Str.B_speed > 0) OLED_ShowString(15, 13, "-"), OLED_ShowNumber(20, 13, Send_Data.Send_Str.B_speed, 5, 12);
        else                                OLED_ShowString(15, 13, "+"), OLED_ShowNumber(20, 13, -Send_Data.Send_Str.B_speed, 5, 12);


        //��ʾ����3����ʾ����//
        OLED_ShowString(0, 26, "Q");

        if(sbus[4] > 1500)										OLED_ShowString(15, 26, "Up  ");

        if(sbus[4] < 1500 && sbus[4] > 500)						OLED_ShowString(15, 26, "Stop");

        if(sbus[4] < 500)										OLED_ShowString(15, 26, "Down");

        OLED_ShowString(60, 26, "H");

        if(Send_Data.Send_Str.R_status[4] == 0x01 && Send_Data.Send_Str.R_status[5] == 0x01)
            OLED_ShowString(75, 26, "Down");

        if(Send_Data.Send_Str.R_status[4] == 0x00 && Send_Data.Send_Str.R_status[5] == 0x00 && Send_Data.Send_Str.R_status[6] == 0x00 && Send_Data.Send_Str.R_status[7] == 0x00)
            OLED_ShowString(75, 26, "Stop");

        if(Send_Data.Send_Str.R_status[6] == 0x01 && Send_Data.Send_Str.R_status[7] == 0x01)
            OLED_ShowString(75, 26, "Up  ");

        //��ʾ����4����ʾ����//
        OLED_ShowString(0, 39, "D");

        if(Send_Data.Send_Str.R_status[1] == 0x00)
            OLED_ShowString(15, 39, "Stop");

        if(Send_Data.Send_Str.R_status[1] == 0x01)
            OLED_ShowString(15, 39, "Run ");

        OLED_ShowString(60, 39, "X");

        if(Send_Data.Send_Str.R_status[2] == 0x00 && Send_Data.Send_Str.R_status[3] == 0x00) 	OLED_ShowString(75, 39, "Middle");

        if(Send_Data.Send_Str.R_status[2] == 0x01)  OLED_ShowString(75, 39, "High  ");

        if(Send_Data.Send_Str.R_status[3] == 0x01)  OLED_ShowString(75, 39, "Low   ");

        //��ʾ����5����ʾ����
        OLED_ShowString(0, 52, "Car");

        if(Send_Data.Send_Str.Speed == 2)  OLED_ShowString(30, 52, "Middle");

        if(Send_Data.Send_Str.Speed == 1)  OLED_ShowString(30, 52, "High  ");

        if(Send_Data.Send_Str.Speed == 4)  OLED_ShowString(30, 52, "Low   ");


        if(SBUS_ON_Flag == 1)     OLED_ShowString(90, 52, "R-C  ");

        else                      OLED_ShowString(90, 52, "ROS  ");

        //ˢ����Ļ
        OLED_Refresh_Gram();
    }
}

