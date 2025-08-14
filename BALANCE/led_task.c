#include "led_task.h"

/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
�������ܣ�LED����˸����
��ڲ�������
����  ֵ����
**************************************************************************/
void led_task(void *pvParameters)
{
    while(1)
    {
        static u8 led_state=0;//LED��������
		
		//����Զ��س�״̬,��LED����ɫ�����޸�
		static u8 check_charger_mode;
		static u8 check_Charging;
		
		//����س�״̬,LED��ɫΪ��ɫ
		if( 0 == check_charger_mode && 1 == charger.AllowRecharge ) LED_SetColor(LED_Yellow);
		
		//�˳��س�״̬,LED��ɫ����Ĭ����ɫ
		if( 1 == check_charger_mode && 0 == charger.AllowRecharge ) LED_SetColor(LED_Red);
		check_charger_mode = charger.AllowRecharge;
		
		if( 1==charger.AllowRecharge ) 
		{	//�Զ��س�״̬�³��ɹ�ʱ��LED����Ϊ��ɫ
			if( 0==check_Charging && 1 == charger.Charging ) LED_SetColor(LED_Purple);
			if( 1==check_Charging && 0 == charger.Charging ) LED_SetColor(LED_Yellow);
			check_Charging = charger.Charging; 
		}

        //LED��˸����
        led_state = !led_state;
        LED_SetState(led_state);
		
        //The LED flicker task is very simple, requires low frequency accuracy, and uses the relative delay function
        //LED��˸����ǳ��򵥣���Ƶ�ʾ���Ҫ��ͣ�ʹ�������ʱ����
        vTaskDelay(SysVal.LED_delay);
    }
}


