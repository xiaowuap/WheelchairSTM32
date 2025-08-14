#include "robot_init.h"

static void set_RobotType(uint8_t type);
static void Robot_Init(float wheelspacing,float gearratio,int Accuracy,\
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
		//�°�ȫ����

		//����� �����͡�������
		case 0: Robot_Init(Omni_Turn_Radiaus_180 , MD36N_27 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI); break;
		case 1: Robot_Init(Omni_Turn_Radiaus_180 , MD36N_51 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI); break;
		
		//����� ������
		case 2: Robot_Init(Omni_Turn_Radiaus_290 , MD60N_18 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI); break;

		
		//�ɰ�
//		//���ٰ� ����������ȫ����
//		case 0: Robot_Init(Omni_Turn_Radiaus_164 , MD36N_5_18 , GMR_500 , FullDirecion_75 ,  VEL_KP , VEL_KI); break;
//		case 1: Robot_Init(Omni_Turn_Radiaus_180 , MD36N_27 ,   GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI); break;
//		
//		//����ȫ���������γ��桢����
//		case 2: Robot_Init(Omni_Turn_Radiaus_180 , MD36N_27 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI); break;
//		case 3: Robot_Init(Omni_Turn_Radiaus_180 , MD36N_51 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI); break;
//		
//		//����ȫ���ֳ���3�ֲ�ֱͬ��
//		case 4: Robot_Init(Omni_Turn_Radiaus_290 , MD60N_18 , GMR_500 , FullDirecion_127 , VEL_KP , VEL_KI); break;
//		case 5: Robot_Init(Omni_Turn_Radiaus_290 , MD60N_18 , GMR_500 , FullDirecion_152 , VEL_KP , VEL_KI); break;
//		case 6: Robot_Init(Omni_Turn_Radiaus_290 , MD60N_18 , GMR_500 , FullDirecion_203 , VEL_KP , VEL_KI); break;
		
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

//��ڲ��������͡��־ࡢ������ٱȡ�������������ȡ�����ֱ����Ĭ��kp��Ĭ��ki
static void Robot_Init(float omni_turn,float gearratio,int Accuracy,\
	                    float tyre_diameter,int vkp,int vki)
{
    robot.HardwareParam.WheelSpacing = 0;              //�־�
	robot.HardwareParam.AxleSpacing = 0;               //���
	robot.HardwareParam.GearRatio = gearratio;         //������ٱ�
	robot.HardwareParam.EncoderAccuracy = Accuracy;    //�������������
	robot.HardwareParam.Wheel_Circ = tyre_diameter*PI; //�����ܳ�(PI*D)
	robot.HardwareParam.TurnRadiaus = omni_turn;       //ȫ������ת�뾶
	
	//ȫ�����˶�ѧ������صĲ���
	robot.HardwareParam.X_PARAMETER = sqrt(3)/2.0f;
	robot.HardwareParam.Y_PARAMETER = 0.5f;
	robot.HardwareParam.L_PARAMETER = 1.0f;
	
	//����������*������ٱ�*��Ƶ�� = �����תһȦ�ı���������
	robot.HardwareParam.Encoder_precision = EncoderMultiples * robot.HardwareParam.EncoderAccuracy * robot.HardwareParam.GearRatio;
	robot.V_KP = vkp;                                  //Ĭ��kp����
	robot.V_KI = vki;                                  //Ĭ��ki����

}

