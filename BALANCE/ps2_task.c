#include "ps2_task.h"

/**************************************************************************
Function: Ps2 handle task
Input   : none
Output  : none
�������ܣ�PS2�ֱ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void pstwo_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(PS2_TASK_RATE));
		
		if( SysVal.HardWare_Ver == V1_0 ) // V1.0Ӳ���汾 ʹ�ó���ps2�ֱ�
		{
			//Read the ps2 data
			//��ȡPS2������
			PS2_Read();		
		}
		else if( SysVal.HardWare_Ver == V1_1 ) //V1.1Ӳ���汾 ʹ��usb�ֱ�
		{
			//V1.1Ӳ��ʹ��USB�ֱ�,��ȡ�����ڴ˺���
		}
		
		//��Ϸ�ֱ�ģʽ����
		if( GamePadInterface->StartFlag == 1 && Get_Control_Mode(_PS2_Control)==0 && SysVal.Time_count>=CONTROL_DELAY && GamePadInterface->LY > 150 )
			Set_Control_Mode(_PS2_Control);
		
    }
}

