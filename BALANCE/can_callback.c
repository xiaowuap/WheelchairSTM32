#include "can_callback.h"

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;   
	
	u8 temp_rxbuf[8];
	
	//��ȡCAN1 FIFO0���������
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	//�����ݶ���������ʹ��
	memcpy(temp_rxbuf,RxMessage.Data,8);
	
	//��׼֡ID���ݴ���
	if( RxMessage.IDE==CAN_Id_Standard )
	{
		switch( RxMessage.StdId ) //֡ID��
		{
			case 0x181: //֡ID 0x181ΪС������ָ��
			{
				//����CAN����ģʽ
				if( Get_Control_Mode(_CAN_Control)==0 ) Set_Control_Mode(_CAN_Control);
				robot_control.command_lostcount = 0;//���ʱˢ��
				//����С��Ŀ���ٶ�
				robot_control.Vx = ((float)((short)((temp_rxbuf[0]<<8)|(temp_rxbuf[1]))))/1000.0f;
				robot_control.Vy = ((float)((short)((temp_rxbuf[2]<<8)|(temp_rxbuf[3]))))/1000.0f;
				robot_control.Vz = ((float)((short)((temp_rxbuf[4]<<8)|(temp_rxbuf[5]))))/1000.0f;
				break;
			}
			
			case 0x182: //�Զ��س��豸������
			{		
				charger.OutLine_Check = 0;//�س�״̬����ˢ��
				#if defined AKM_CAR
					Handle_AKM_AutoRecharge(temp_rxbuf); //�������Զ��س��߼�������
				#else
					Handle_Normal_AutoRecharge(temp_rxbuf); //����С���Զ��س䴦���߼�����
				#endif
			}
			
			default:
				break;
		}
	}
	
	//��չ֡ID���ݴ���
	else if( RxMessage.IDE==CAN_Id_Extended )
	{
		switch( RxMessage.ExtId ) //֡ID��
		{
			case 0x12345678://�Զ��س�������У��,���ڼ��С���Ƿ�����Զ��س��豸
			{
				u8 check=0;
				for(u8 i=0;i<8;i++) 
				{
					check += temp_rxbuf[i];
				}
				if( check==8 ) SysVal.HardWare_charger = 1;
				break;
			}
			
			case 0x001:
			{
				break;
			}
				
			default:
				break;
		}
	}
	
}
