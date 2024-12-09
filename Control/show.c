#include "show.h"

u8 oled_page = 0;//ÆÁÄ»Ê¹ÄÜ
u8 oled_refresh_flag;
extern int Time_count;
extern u8 QB_UP_Flag;
extern SEND_DATA Send_Data;
/**************************************************************************
º¯Êı¹¦ÄÜ£º·äÃùÆ÷±¨¾¯¡¢¿ªÆô×Ô¼ì¡¢ÏòAPP·¢ËÍÊı¾İ¡¢OLEDÏÔÊ¾ÆÁÏÔÊ¾ÈÎÎñ
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
int Buzzer_count = 25;
void show_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while(1)
    {
        static int LowVoltage_1 = 0, LowVoltage_2 = 0;
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //´ËÈÎÎñÒÔ10HzµÄÆµÂÊÔËĞĞ

        //¿ª»úÊ±·äÃùÆ÷¶ÌÔİ·äÃù£¬¿ª»úÌáĞÑ
        if(Time_count < 50)Buzzer = 1;
        else if(Time_count >= 51 && Time_count < 100)Buzzer = 0;

        if(LowVoltage_1 == 1 || LowVoltage_2 == 1)Buzzer_count = 0;

        if(Buzzer_count < 5)Buzzer_count++;

        if(Buzzer_count < 5)Buzzer = 1; //·äÃùÆ÷·äÃù
        else if(Buzzer_count == 5)Buzzer = 0;

        if(LowVoltage_1 == 1)LowVoltage_1++; //È·±£·äÃùÆ÷Ö»Ïì0.5Ãë

        if(LowVoltage_2 == 1)LowVoltage_2++; //È·±£·äÃùÆ÷Ö»Ïì0.5Ãë

        //APP_Show();	 //ÏòAPP·¢ËÍÊı¾İ

        if(oled_refresh_flag) OLED_Clear(), oled_refresh_flag = 0;
        else oled_show(); //ÏÔÊ¾ÆÁÏÔÊ¾ÈÎÎñ
        
        xEventGroupSetBits(EventGroupHandler, EVENTBIT1);
    }
}

/**************************************************************************
º¯Êı¹¦ÄÜ£ºOLEDÏÔÊ¾ÆÁÏÔÊ¾ÈÎÎñ
Èë¿Ú² ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void oled_show(void)
{
    if(oled_page == 1)
    {
        //ÏÔÊ¾ÆÁµÚ1ĞĞÏÔÊ¾ÄÚÈİ//
        //ÏÔÊ¾µç»úAµÄµ±Ç°ËÙ¶È
        OLED_ShowString(0, 00, "A");

        if(Send_Data.Send_Str.A_speed > 0) OLED_ShowString(15, 00, "-"), OLED_ShowNumber(20, 00, Send_Data.Send_Str.A_speed, 5, 12);
        else                               OLED_ShowString(15, 00, "+"), OLED_ShowNumber(20, 00, -Send_Data.Send_Str.A_speed, 5, 12);


        //ÏÔÊ¾ÆÁµÚ2ĞĞÏÔÊ¾ÄÚÈİ//
        //ÏÔÊ¾µç»úBµÄµ±Ç°ËÙ¶È
        OLED_ShowString(0, 10, "B");

        if( Send_Data.Send_Str.B_speed > 0) OLED_ShowString(15, 13, "-"), OLED_ShowNumber(20, 13, Send_Data.Send_Str.B_speed, 5, 12);
        else                                OLED_ShowString(15, 13, "+"), OLED_ShowNumber(20, 13, -Send_Data.Send_Str.B_speed, 5, 12);


        //ÏÔÊ¾ÆÁµÚ3ĞĞÏÔÊ¾ÄÚÈİ//
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

        //ÏÔÊ¾ÆÁµÚ4ĞĞÏÔÊ¾ÄÚÈİ//
        OLED_ShowString(0, 39, "D");

        if(Send_Data.Send_Str.R_status[1] == 0x00)
            OLED_ShowString(15, 39, "Stop");

        if(Send_Data.Send_Str.R_status[1] == 0x01)
            OLED_ShowString(15, 39, "Run ");

        OLED_ShowString(60, 39, "X");

        if(Send_Data.Send_Str.R_status[2] == 0x00 && Send_Data.Send_Str.R_status[3] == 0x00) 	OLED_ShowString(75, 39, "Middle");

        if(Send_Data.Send_Str.R_status[2] == 0x01)  OLED_ShowString(75, 39, "High  ");

        if(Send_Data.Send_Str.R_status[3] == 0x01)  OLED_ShowString(75, 39, "Low   ");

        //ÏÔÊ¾ÆÁµÚ5ĞĞÏÔÊ¾ÄÚÈİ
        OLED_ShowString(0, 52, "Car");

        if(Send_Data.Send_Str.Speed == 2)  OLED_ShowString(30, 52, "Middle");

        if(Send_Data.Send_Str.Speed == 1)  OLED_ShowString(30, 52, "High  ");

        if(Send_Data.Send_Str.Speed == 4)  OLED_ShowString(30, 52, "Low   ");


        if(SBUS_ON_Flag == 1)     OLED_ShowString(90, 52, "R-C  ");

        else                      OLED_ShowString(90, 52, "ROS  ");

        //Ë¢ĞÂÆÁÄ»
        OLED_Refresh_Gram();
    }
}

