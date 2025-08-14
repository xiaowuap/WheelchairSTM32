#include "robot_init.h"

static void set_RobotType(uint8_t type);

static void Robot_Init(float wheelspacing,float axlespacing,float gearratio,int Accuracy,\
	                    float tyre_diameter,int vkp,int vki);

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
void Robot_Select(void)
{
	uint16_t TypeNum = 4096/CAR_NUMBER;//���ݳ�������,��ȡADC�ɼ��ķ�Ƶϵ��
	
	TypeNum = Get_ADC1_Average(CarMode_Ch,10)/TypeNum;
	
	set_RobotType(TypeNum);//���ݰ��ӵ�λ��ѡ��ȷ����ͬ�ĳ���
	
	switch( TypeNum )
	{
		//Ԥ��
		case 0: Robot_Init( SENIOR_MEC_BS_wheelspacing , SENIOR_MEC_BS_axlespacing , MD36N_27 , GMR_500 , Mecanum_100 , VEL_KP , VEL_KI ); break;
		
		//�����ʽ����
		case 1: Robot_Init( SENIOR_MEC_BS_wheelspacing , SENIOR_MEC_BS_axlespacing , MD36N_27 , GMR_500 , Mecanum_100 , VEL_KP , VEL_KI ); break;
		
		//�����������
		case 2: Robot_Init( SENIOR_MEC_DL_wheelspacing , SENIOR_MEC_DL_axlespacing , MD36N_27 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		
		//�����ʽ���桢����
		case 3: Robot_Init( TOP_MEC_BS_wheelspacing , TOP_MEC_BS_axlespacing , MD60N_18 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		case 4: Robot_Init( TOP_MEC_BS_wheelspacing , TOP_MEC_BS_axlespacing , MD60N_47 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		
		//����������桢����
		case 5: Robot_Init( TOP_MEC_DL_wheelspacing , TOP_MEC_DL_axlespacing , MD60N_18 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		case 6: Robot_Init( TOP_MEC_DL_wheelspacing , TOP_MEC_DL_axlespacing , MD60N_47 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		
		//�콢��ʽ���桢����
		case 7: Robot_Init( FLAGSHIP_MEC_BS_wheelspacing , FLAGSHIP_MEC_BS_axlespacing , MD60N_18 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		case 8: Robot_Init( FLAGSHIP_MEC_BS_wheelspacing , FLAGSHIP_MEC_BS_axlespacing , MD60N_47 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		
		//�콢�������桢����
		case 9 : Robot_Init( FLAGSHIP_MEC_DL_wheelspacing , FLAGSHIP_MEC_DL_axlespacing , MD60N_18 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		case 10: Robot_Init( FLAGSHIP_MEC_DL_wheelspacing , FLAGSHIP_MEC_DL_axlespacing , MD60N_47 , GMR_500 , Mecanum_152 , VEL_KP , VEL_KI ); break;
		
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
	                    float tyre_diameter,int vkp,int vki)
{
    robot.HardwareParam.WheelSpacing = wheelspacing;   //�־�
	robot.HardwareParam.AxleSpacing = axlespacing;     //���
	robot.HardwareParam.GearRatio = gearratio;         //������ٱ�
	robot.HardwareParam.EncoderAccuracy = Accuracy;    //�������������
	robot.HardwareParam.Wheel_Circ = tyre_diameter*PI; //�����ܳ�(PI*D)
	//����������*������ٱ�*��Ƶ�� = �����תһȦ�ı���������
	robot.HardwareParam.Encoder_precision = EncoderMultiples * robot.HardwareParam.EncoderAccuracy * robot.HardwareParam.GearRatio;
	robot.V_KP = vkp;                                  //Ĭ��kp����
	robot.V_KI = vki;                                  //Ĭ��ki����
}

