#include "auto_recharge.h"
#include "can.h"
#include "robot_init.h"

CHARGER_t charger;


//�Զ��س����������λ
void auto_recharge_reset(void)
{
	//�Զ��س�ģʽ
	charger.AllowRecharge = 0;
	
	//�Ƿ��ڳ��
	charger.Charging = 0;
	
	//������
	charger.ChargingCurrent = 0;
	
	//�����ӹܱ�־λ
	charger.NavWalk = 0;
	
	//���߼��
	charger.OutLine_Check = 0;
	
	//Ĭ�ϵĶԽ��ٶ�����
	charger.Dock_MoveX = -0.1f;
	charger.Dock_MoveY = 0;
	charger.Dock_MoveZ = 0.2f;
	
	//4·�����ź�
	charger.L_A = 0;
	charger.L_B = 0;
	charger.R_A = 0;
	charger.R_B = 0;
	charger.RED_STATE = 0;
	
	//����ˢ��ʱ��
	#if defined AKM_CAR
		//������С������
		charger.RedRefalsh_Time = 50;
	#else
		//�ǰ�����С������0,����ʹ�ó��װ���ϴ��ڵ�Ĭ��ʱ��
		charger.RedRefalsh_Time = 0;
	#endif
	
	//������ơ������ӹܿ����ٶȸ�λ
	charger.Red_MoveX = 0;
	charger.Red_MoveY = 0;
	charger.Red_MoveZ = 0;
	charger.Up_MoveX = 0;
	charger.Up_MoveY = 0;
	charger.Up_MoveZ = 0;
	
	//�����Ƿ���ڳ��װ��
	Find_Charging_HardWare();
}


//���ʵ�ǰ�Ƿ���ڳ��װ��Ӳ��
//���� ��չ֡idΪ0x12345678,����Ϊ{1,1,1,1,1,1,1,1} �����װ��,���װ��������һ����id������,˵�����װ�������߲��ɹ�����.
void Find_Charging_HardWare(void)
{
    u8 tmpdata[8]= {1,1,1,1,1,1,1,1};
    CAN1_Send_EXTid_Num(0x12345678,tmpdata);
}

//���ó��װ���Ϸ��ĶԽ��ٶȵĴ�С,�Լ����ú����ˢ��ʱ��
void CAN_Send_AutoRecharge(void)
{
	u8 CAN_SENT[8];
	
	//Ԥ��λ
	CAN_SENT[0]=0;
	
	//Set the speed of the infrared interconnection, unit m/s
	//���ú���Խӵ��ٶȴ�С����λmm/s
	CAN_SENT[1]=((short)(charger.Dock_MoveX*1000))>>8;
	CAN_SENT[2]=((short)(charger.Dock_MoveX*1000));
	CAN_SENT[3]=((short)(charger.Dock_MoveY*1000))>>8;
	CAN_SENT[4]=((short)(charger.Dock_MoveY*1000));
	CAN_SENT[5]=((short)(charger.Dock_MoveZ*1000))>>8;
	CAN_SENT[6]=((short)(charger.Dock_MoveZ*1000));
	CAN_SENT[7] = charger.RedRefalsh_Time;	
	CAN1_Send_Num(0x105,CAN_SENT);

}


//�ǰ�����С�����Զ��س䴦���߼�
#if !defined AKM_CAR
void Handle_Normal_AutoRecharge(uint8_t* temp_rxbuf)
{
	//Calculate the three-axis target velocity, unit: mm/s
	//��������Ŀ���ٶȣ���λ��mm/s
	charger.Red_MoveX=((float)((short)((temp_rxbuf[0]<<8)+(temp_rxbuf[1]))))/1000; 
	charger.Red_MoveY = 0;
	//charger.Red_MoveY=((float)((short)((temp_rxbuf[2]<<8)+(temp_rxbuf[3]))))/1000;
	charger.Red_MoveZ=((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
	
	charger.Charging=temp_rxbuf[6]&1;		//���״̬��־λ
	charger.L_A = (temp_rxbuf[6]>>5)&0x01;
	charger.L_B = (temp_rxbuf[6]>>4)&0x01;
	charger.R_B = (temp_rxbuf[6]>>3)&0x01;
	charger.R_A = (temp_rxbuf[6]>>2)&0x01;
	charger.RED_STATE = charger.L_A + charger.L_B + charger.R_B + charger.R_A;
	
	//�����ں����źŻ�С���ڳ��,���Ŀ���ٶ�
	if( 0 == charger.RED_STATE || 1 == charger.Charging  ) charger.Red_MoveX = 0 , charger.Red_MoveY = 0 , charger.Red_MoveZ = 0;
	
	//����������
	if(temp_rxbuf[7]>128) charger.ChargingCurrent=-(256-temp_rxbuf[7])*30;
	else charger.ChargingCurrent=(temp_rxbuf[7]*30);
}
#endif




#if defined AKM_CAR
//����debug���ڲ�����,��debugʱ����Ϊȫ�����
u8 liar_adjust=0;           //���߳��ģʽ
u8 last_state=0;            //��������źŵ���һ��״̬
u16 adjust_timeout=0;       //���ڳ�ʱ����,����ͳ�Ƶ���ʱ���Ƿ����
u16 double_adjust_timeout=0;//˫�ߵ��ڳ�ʱ����,����ͳ�Ƶ���ʱ���Ƿ����
u8 red_now_state;           //�����źŵ�ǰ��״̬
u8 touch_state;             //���׮����װ����Ƭ�ĽӴ�״̬

void Handle_AKM_AutoRecharge(uint8_t* temp_rxbuf)
{
	static u8 tmp_state=0;
	static u8 last_touchstate=0;
	
	//�Ӵ���Ƭ���״̬����
	static u8 state_lock = 0;
	
	//�Ӵ���Ƭ���뿪ʱ����̬��־
	static u8 change_state = 0;
	
	//�Ӵ���Ƭ���뿪ʱ����ʱ�ں�
	static u16 time_core = 0;
	
	//�Ӵ���Ƭʱ���˲���������
	static u16 filter_cur = 0,filter_vol = 0;
	
	//С�������̬�ĵ�������
	static u8 adjust_times = 0;
	static u8 adjust_vol = 0 , adjust_cur = 0;
	
	//���߳���־λ
	static u8 liar_charge = 0;
	
	//���߶��Ӵ����ˣ���ǵ���1��
	if(adjust_vol&&adjust_cur) adjust_vol = 0,adjust_cur = 0,adjust_times++;
	
	//����״̬����
	charger.L_A = (temp_rxbuf[6]>>5)&0x01;
	charger.L_B = (temp_rxbuf[6]>>4)&0x01;
	charger.R_B = (temp_rxbuf[6]>>3)&0x01;
	charger.R_A = (temp_rxbuf[6]>>2)&0x01;
	
	//���ڼ����������ѹ���Ӵ����
	touch_state = temp_rxbuf[2];
	if( last_touchstate!=0xAA&&touch_state==0xAA ) adjust_timeout=0,liar_adjust++;//����״̬������¼
	if( (last_touchstate!=0xAA&&touch_state==0xAA) || (last_state!=0xBB&&touch_state==0xBB) ) double_adjust_timeout=0;//����һ�߽Ӵ������˫���ӳ�ʱ��
	last_touchstate = touch_state;
	
	//���߶��޽Ӵ�����5�룬��յ��������궨
	if(adjust_times>0) double_adjust_timeout++;
	if( double_adjust_timeout>250 ) adjust_times=0,double_adjust_timeout=0;
	
	//�����޽Ӵ�5�룬��ճ��ߵ��������궨
	if( liar_adjust>0 ) adjust_timeout++;
	if( adjust_timeout>250 ) liar_adjust=0,adjust_timeout=0;//�Ӵ��������5�룬�ж�Ϊ�쳣�������յ���������¼��
	
	
	//��⵽�Ѿ�������ڣ�����Ҫ��λ
	if(liar_charge==1&&touch_state!=0xAA&&touch_state!=0xCF) 
	{
		if( touch_state==0xAB && robot.voltage >=25.0f )//��������
		{
			charger.AllowRecharge=0;
		}
		adjust_times = 0,liar_charge=0;
	}	
	
	//���߳�磬�Զ����־λtouch_state
	if(liar_charge) touch_state = 0xFC;
	
	/* ��Բ�ͬ�������ò�ͬ���� */
	u16 leave_times = 75; //�Ӵ���Ƭ���뿪��ʱ�� 75*20ms = 1.5s
	u8 yuzhi = 40;        //�Ӵ���Ƭ��������ʱ�� 40*20ms = 800ms
	if(charger.AllowRecharge==0) 
	{

		adjust_times = 0,liar_charge=0,liar_adjust=0;//�ر��Զ��س�ʱ���궨�ѵ����Ĵ���Ϊ0
		if( robot.type==2 || robot.type==3 )  Set_Robot_PI_Param(300,300); 
		if( robot.type==4 ) Set_Robot_PI_Param(400,100); 
		if( robot.type==5 ) Set_Robot_PI_Param(50,200); 
	}
	else
	{	
		if( robot.type==0 || robot.type==1 ) leave_times=80, charger.RedRefalsh_Time=100; //ԭ80��50
		if( robot.type==2 || robot.type==3 ) Set_Robot_PI_Param(600,600) , charger.RedRefalsh_Time=100; 
		if( robot.type==4 || robot.type==5 ) 
		{
			//Red_Docker_X=-0.1f, Red_Docker_Y=0, Red_Docker_Z=0.2f; Ĭ��ֵ
			charger.Dock_MoveX = -0.08f;
			charger.Dock_MoveZ = 0.15f;
			Set_Robot_PI_Param(600,600);
			leave_times=113;//����������ҵ����Ӧ��������Ҫ�����ĵ���ʱ��
			charger.RedRefalsh_Time=150;//����״̬ˢ��ʱ��(�����ӦԽ���������ʱ��Խ������������ٱȡ�С�����ض��й�ϵ)
		}
	}

	//�������ںϳ�һ����ֵ��ʾ״̬�������״̬���
	red_now_state = charger.L_A << 3 | charger.L_B << 2 | charger.R_B << 1 | charger.R_A << 0 ;
	
	if( charger.AllowRecharge )//�Զ��س俪����ż�¼״̬
	{
		//����״̬�����ı�ʱ��������һ������״̬
		if(red_now_state!=tmp_state) last_state = tmp_state;	
		tmp_state = red_now_state;
	}
	
	////////////////////////////  �Խ��߼����� ��ʼ ////////////////////////////	
	charger.RED_STATE = charger.L_A + charger.L_B + charger.R_B + charger.R_A;
	
		 if(charger.L_A==0&&charger.L_B==0&&charger.R_B==0&&charger.R_A==1)  front_right; //1
		 
	else if(charger.L_A==0&&charger.L_B==0&&charger.R_B==1&&charger.R_A==0)  back_left;  //2
	
	else if(charger.L_A==0&&charger.L_B==0&&charger.R_B==1&&charger.R_A==1)  front_right;  //3

	else if(charger.L_A==0&&charger.L_B==1&&charger.R_B==0&&charger.R_A==0)  //4
	{
		front_left; 
	}	
	
	// charger.L_A==0&&charger.L_B==1&&charger.R_B==0&&charger.R_A==1 ������ 5 
	
	else if(charger.L_A==0&&charger.L_B==1&&charger.R_B==1&&charger.R_A==0) //6
	{
		front_left;   
		if(last_state==2) back;
		
	}	
	
	else if(charger.L_A==0&&charger.L_B==1&&charger.R_B==1&&charger.R_A==1)  back; //7
	
	else if(charger.L_A==1&&charger.L_B==0&&charger.R_B==0&&charger.R_A==0) 
	{
		back_right; //8
		if( last_state==9 ) back;
	}

	else if(charger.L_A==1&&charger.L_B==0&&charger.R_B==0&&charger.R_A==1)   //9
	{
		front_right;
		if( last_state==8||last_state==1 ) back;
	}
	
	else if (charger.L_A==1&&charger.L_B==0&&charger.R_B==1&&charger.R_A==0)  back; //10
	
	else if(charger.L_A==1&&charger.L_B==0&&charger.R_B==1&&charger.R_A==1) // 11
	{
		front_right;
		if(last_state==9) back;
	}
	
	else if(charger.L_A==1&&charger.L_B==1&&charger.R_B==0&&charger.R_A==0)  back_right;  //12	
	
	else if(charger.L_A==1&&charger.L_B==1&&charger.R_B==0&&charger.R_A==1)
	{
		back; //13
	}			
	
	else if(charger.L_A==1&&charger.L_B==1&&charger.R_B==1&&charger.R_A==0)  //14
	{
		front_left; 
		if(last_state==6||last_state==3) back;
	}			

	else if(charger.L_A==1&&charger.L_B==1&&charger.R_B==1&&charger.R_A==1)  back;		  //15
	
	//���������ȫ0����5
	else
	{
		charger.RED_STATE = 0;//����ʶ��״̬����Ϊ0
		stop;
	}

	
	if(touch_state==0xAA||state_lock==1)//���װ���Ӵ����˳����
	{
		stop;		

		filter_cur++;   
		if(filter_cur>yuzhi) //�ӳ�����
		{
			adjust_cur = 1; //��ǽӴ�
			
			filter_cur = yuzhi+1;//�˲�������������ֹѭ�����
			
			state_lock = 1;//�����Ӵ�״̬����ʾ�ڸ���״̬ǰ����һֱ���뱾�ж�
			
			cur_front_right;//������̬
			
			//�����������������ƣ��ֽӴ����˳���������ٵ����ˣ�ʹ�õ��߳��
			u8 allow_adjust = 2; //�ԽӺ��������λ�˵Ĵ���
			if(adjust_times==allow_adjust||( liar_adjust>=3&&adjust_times>0 ))
			{
				stop;		
				liar_charge = 1;//�������ˣ��������߳��
				liar_adjust=0;
			}
		}
		
	}
	else if(touch_state==0xBB||state_lock==2)//�Ӵ����˲�ѹ��
	{
		stop;
		
		filter_vol++;   
		if(filter_vol>yuzhi) //�ӳ�����
		{  
			adjust_vol = 1; //��ǽӴ�1��
			
			filter_vol = yuzhi+1;
			state_lock = 2;
			cur_front_left;//������̬
		}
	}
	else if(touch_state==0xCF||touch_state==0xFC)
	{
		liar_charge=1;//ͬʱ�Ӵ���ѹ���ͳ����ʱ�����ŵ��߳�磬����ȶ���
		liar_adjust=0;
		stop;
	}
	
	//��Ƭ�Ӵ�״̬����
	if(state_lock!=0)
	{
		time_core++;
		if(time_core> leave_times )
		{
			if(state_lock==2) change_state = 2 ;
			if(state_lock==1) change_state = 1 ;
			state_lock = 0,time_core=0,filter_vol=0,filter_cur=0;
		}				
	}
	else
		time_core =  0;
	
	//��Ƭ״̬���º����ε�ȫ1��������к������
	//�Ӵ�����Ƭ���뿪����ֻ����ȫ������˵����ݣ���������һ��ǰ��������ת��
	if(change_state!=0)
	{
		static u8 shielding=0;
		static u8 leaveCount=0;	

		if(++leaveCount==255) leaveCount=0, change_state = 0,shielding=0;//��������ĵ���ʱ��
		
		if(red_now_state==9) {front_right;shielding++;}//����״̬����̫����
		else if( red_now_state==6 ) {front_left;shielding++;}//����״̬����̫����
		else if( red_now_state!=15 ) {front;shielding=0;}
		
		//����ñ���һֱ������˵������ʱ��̫���ˣ���յ����������¿�ʼ
		if(shielding>250) adjust_times=0,liar_adjust=0;
		
		if(last_state==15 || last_state==9 || last_state==6 ) leaveCount=0,change_state = 0,shielding=0;
	}
	
	//Z�ٶ�ת��Ϊ�Ƕ�
	charger.Red_MoveZ = Akm_Vz_to_Angle(charger.Red_MoveX,charger.Red_MoveZ);

	////////////////////////////  �Խ��߼����� ���� ////////////////////////////
	
	if(liar_charge==1) charger.Charging = 1;//���������ڳ��
	else charger.Charging = 0;
	
	if(charger.Charging==1) charger.Red_MoveX = 0, charger.Red_MoveY = 0 , charger.Red_MoveZ = 0;
	
	//����������
	if(temp_rxbuf[7]>128) charger.ChargingCurrent=-(256-temp_rxbuf[7])*30;
	else  charger.ChargingCurrent=(temp_rxbuf[7]*30);
}
#endif


