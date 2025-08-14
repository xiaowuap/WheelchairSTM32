#include "robot_select_init.h"

static void Robot_Init(uint8_t type,float wheelspacing,float axlespacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,float akm_min_turn,float omin_turnR,int vkp,int vki);

//����һ�������ˣ��ڲ����������˵ĸ��ֲ���
static ROBOT robot;

/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
�������ܣ����ݵ�λ���л���Ҫ���Ƶ�С������
��ڲ�������
����  ֵ����
**************************************************************************/
void Robot_Select(void)
{
	uint16_t TypeNum = 4096/CAR_NUMBER;//���ݳ�������,��ȡADC�ɼ��ķ�Ƶϵ��
	
	TypeNum = Get_ADC1_Average(CarMode_Ch,10)/TypeNum;//���ݰ��ӵ�λ��ѡ��ȷ����ͬ�ĳ���
	
	#if defined AKM_CAR
	
	switch( TypeNum )
	{
		//���䳣�桢����
		case 0: Robot_Init(TypeNum,SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,0,VEL_KP,VEL_KI);break;
		case 1: Robot_Init(TypeNum,SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_51,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,0,VEL_KP,VEL_KI);break;
		
		//�����ʽ���桢����
		case 2: Robot_Init(TOP_AKM_BS_wheelspacing,TOP_AKM_BS_axlespacing,MD60N_18,GMR_500,TOP_AKM_BS_WheelDiameter,VEL_KP,VEL_KI);break;
		case 3: Robot_Init(TOP_AKM_BS_wheelspacing,TOP_AKM_BS_axlespacing,MD60N_47,GMR_500,TOP_AKM_BS_WheelDiameter,VEL_KP,VEL_KI);break;
		
		//����������桢����
		case 4: Robot_Init(TOP_AKM_DL_wheelspacing,TOP_AKM_DL_axlespacing,MD60N_18,GMR_500,TOP_AKM_DL_WheelDiameter,400,100);break;
		case 5: Robot_Init(TOP_AKM_DL_wheelspacing,TOP_AKM_DL_axlespacing,MD60N_47,GMR_500,TOP_AKM_DL_WheelDiameter,50,200);break;
		
		//Ԥ���Ǳ궨�Ƴ� ����
		case 6: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,VEL_KP,VEL_KI);break;
		
		//7��8��װ����ʹ�ó���
		case 7: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,VEL_KP,VEL_KI);break;
		case 8: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,VEL_KP,VEL_KI);break;
		
		//���ٰ�����(���䳵��,5.18���ٱȵ��)
		case 9: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_5_18,GMR_500,SENIOR_AKM_WheelDiameter,200,50);break;
		
		default:break;
	}
	
	
	#elif defined DIFF_CAR
	
	#elif defined MEC_CAR
	
	#elif defined _4WD_CAR
	
	#elif defined OMNI_CAR
	
	#endif
    
}


/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
�������ܣ���ʼ��С������
��ڲ������־� ��� ��ת�뾶 ������ٱ� ������������� ��ֱ̥��
����  ֵ����
**************************************************************************/

void robot_param_setKP(int setKP)
{
	robot.V_KP = setKP;
}

void robot_param_setKI(int setKI)
{
	robot.V_KI = setKI;
}


//��ڲ��������͡��־ࡢ��ࡢ������ٱȡ�������������ȡ�����ֱ������������Сת��뾶��ȫ������ת�뾶��Ĭ��kp��Ĭ��ki
static void Robot_Init(uint8_t type,float wheelspacing,float axlespacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,float akm_min_turn,float omin_turnR,int vkp,int vki)
{
	robot.HardwareParam.type = type;                   //����
    robot.HardwareParam.WheelSpacing = wheelspacing;   //�־�
	robot.HardwareParam.AxleSpacing = axlespacing;     //���
	robot.HardwareParam.GearRatio = gearratio;         //������ٱ�
	robot.HardwareParam.EncoderAccuracy = Accuracy;    //�������������
	robot.HardwareParam.Wheel_Circ = tyre_diameter*PI; //�����ܳ�(PI*D)
	robot.HardwareParam.MIN_turn_radius = akm_min_turn;//��Сת��뾶
	
	
	robot_param_setKP(vkp);
	robot_param_setKI(vki);
}

//�־�
float get_wheelspacing(void)
{
	return robot.HardwareParam.WheelSpacing;
}

//���
float get_AxleSpacing(void)
{
	return robot.HardwareParam.AxleSpacing;
}

//���ٱ�
float get_GearRatio(void)
{
	return robot.HardwareParam.GearRatio;
}

//����������
uint16_t get_EncoderAccuracy(void)
{
	return robot.HardwareParam.EncoderAccuracy;
}

//�����ܳ�
float get_Wheel_Circ(void)
{
	return robot.HardwareParam.Wheel_Circ;
}

int get_robot_KP(void)
{
	return robot.V_KP;
}

int get_robot_KI(void)
{
	return robot.V_KI;
}
