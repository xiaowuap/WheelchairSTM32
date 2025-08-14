#include "robot_init.h"

static void set_RobotType(uint8_t type);
static void Robot_Init(float wheelspacing,float axlespacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,float akm_min_turn,int vkp,int vki);


//С��Ӳ���ṹ�͵����ر���
ROBOT_t robot;

/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
�������ܣ����ݵ�λ���л���Ҫ���Ƶ�С������
��ڲ�������
����  ֵ����
**************************************************************************/
/*
���������ʹ���˵��:
0  :���䰢���� ����        SENIOR_AKM
1  :���䰢���� ����        SENIOR_AKM
2  :���䰢������ʽ���� ���� TOP_AKM_BS
3  :���䰢������ʽ���� ���� TOP_AKM_BS
4  :���䰢������������ ���� TOP_AKM_DL
5  :���䰢������������ ���� TOP_AKM_DL
6  :�Ǳ�׼�Ķ��Ƴ���       
7  :���ڰ�װʱ���Եĳ���
8  :���ڰ�װʱ���Եĳ���
9  :���䰢���� ���ٰ�       SENIOR_AKM
*/
void Robot_Select(void)
{
	uint16_t TypeNum = 4096/CAR_NUMBER;//���ݳ�������,��ȡADC�ɼ��ķ�Ƶϵ��
	
	TypeNum = Get_ADC1_Average(CarMode_Ch,10)/TypeNum;
	
	set_RobotType(TypeNum);//���ݰ��ӵ�λ��ѡ��ȷ����ͬ�ĳ���
	
	switch( TypeNum )
	{
		//���䳣�桢����
		case 0: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,VEL_KP,VEL_KI); break;
		case 1: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_51,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		
		//�����ʽ���桢����
		case 2: Robot_Init(TOP_AKM_BS_wheelspacing,TOP_AKM_BS_axlespacing,MD60N_18,GMR_500,TOP_AKM_BS_WheelDiameter,TOP_AKM_BS_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		case 3: Robot_Init(TOP_AKM_BS_wheelspacing,TOP_AKM_BS_axlespacing,MD60N_47,GMR_500,TOP_AKM_BS_WheelDiameter,TOP_AKM_BS_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		
		//����������桢����
		case 4: Robot_Init(TOP_AKM_DL_wheelspacing,TOP_AKM_DL_axlespacing,MD60N_18,GMR_500,TOP_AKM_DL_WheelDiameter,TOP_AKM_DL_MIN_TURN_RADIUS,400,100);break;
		case 5: Robot_Init(TOP_AKM_DL_wheelspacing,TOP_AKM_DL_axlespacing,MD60N_47,GMR_500,TOP_AKM_DL_WheelDiameter,TOP_AKM_DL_MIN_TURN_RADIUS,50,200);break;
		
		//Ԥ���Ǳ궨�Ƴ� ����
		case 6: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		
		//7��8��װ����ʹ�ó���
		case 7: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		case 8: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_27,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,VEL_KP,VEL_KI);break;
		
		//���ٰ�����(���䳵��,5.18���ٱȵ��)
		case 9: Robot_Init(SENIOR_AKM_wheelspacing,SENIOR_AKM_axlespacing,MD36N_5_18,GMR_500,SENIOR_AKM_WheelDiameter,SENIOR_AKM_MIN_TURN_RADIUS,200,50);break;
		
		default:break;
	}
	
}


/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
�������ܣ���ʼ��С������
��ڲ������־� ��� ��ת�뾶 ������ٱ� ������������� ��ֱ̥��
����  ֵ����
**************************************************************************/
static void set_RobotType(uint8_t type)
{
	robot.type = type;
}

//��ڲ��������͡��־ࡢ��ࡢ������ٱȡ�������������ȡ�����ֱ������������Сת��뾶��ȫ������ת�뾶��Ĭ��kp��Ĭ��ki
static void Robot_Init(float wheelspacing,float axlespacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,float akm_min_turn,int vkp,int vki)
{
    robot.HardwareParam.WheelSpacing = wheelspacing;   //�־�
	robot.HardwareParam.AxleSpacing = axlespacing;     //���
	robot.HardwareParam.GearRatio = gearratio;         //������ٱ�
	robot.HardwareParam.EncoderAccuracy = Accuracy;    //�������������
	robot.HardwareParam.Wheel_Circ = tyre_diameter*PI; //�����ܳ�(PI*D)
	robot.HardwareParam.MIN_turn_radius = akm_min_turn;//��Сת��뾶
	//����������*������ٱ�*��Ƶ�� = �����תһȦ�ı���������
	robot.HardwareParam.Encoder_precision = EncoderMultiples * robot.HardwareParam.EncoderAccuracy * robot.HardwareParam.GearRatio;
	robot.V_KP = vkp;                                  //Ĭ��kp����
	robot.V_KI = vki;                                  //Ĭ��ki����

}

