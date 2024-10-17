#include "robot_select_init.h"

//Initialize the robot parameter structure
//��ʼ�������˲����ṹ��
Robot_Parament_InitTypeDef  Robot_Parament; 
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
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
  //ADCֵ�ֶα�����ȡ����С���ͺ�����
	Divisor_Mode=4096/CAR_NUMBER+5;
	Car_Mode=(int) ((Get_adc_Average(CAR_MODE_ADC,10))/Divisor_Mode); //Collect the pin information of potentiometer //�ɼ���λ��������Ϣ	
  if(Car_Mode>10)Car_Mode=10;
	#if Akm_Car
	{
		if (Car_Mode==0)  Robot_Init(SENIOR_AKM_wheelspacing, SENIOR_AKM_axlespacing, MD36N_27, Photoelectric_500, SENIOR_AKM_Tyre_Diameter); //SENIOR_AKM_27 - ���䰢����С�������� 
		if (Car_Mode==1)  Robot_Init(SENIOR_AKM_wheelspacing, SENIOR_AKM_axlespacing, MD36N_51, Photoelectric_500, SENIOR_AKM_Tyre_Diameter); //SENIOR_AKM_51 - ���䰢����С��������
		
		if (Car_Mode==2)  Robot_Init(TOP_AKM_BS_wheelspacing, TOP_AKM_BS_axlespacing, MD60N_18, Photoelectric_500, TOP_AKM_BS_WHEEL_Diameter); //TOP_AKM_BS_18 - ���䰢����С����ʽ���ҳ�����  //BS: Pendulum suspension
		if (Car_Mode==3)  Robot_Init(TOP_AKM_BS_wheelspacing, TOP_AKM_BS_axlespacing, MD60N_47, Photoelectric_500, TOP_AKM_BS_WHEEL_Diameter); //TOP_AKM_BS_18 - ���䰢����С����ʽ���������� 
		
		if (Car_Mode==4)  Robot_Init(TOP_AKM_DL_wheelspacing, TOP_AKM_DL_axlespacing, MD60N_18, Photoelectric_500, TOP_AKM_DL_Tyre_Diameter),  //TOP_AKM_DL_18 - ���䰢����С���������ҳ�����(������) //DL: Independent suspension
			                Velocity_KP=400,Velocity_KI=100; //PID parameter optimization //PID�����Ż�
		if (Car_Mode==5)  Robot_Init(TOP_AKM_DL_wheelspacing, TOP_AKM_DL_axlespacing, MD60N_47, Photoelectric_500, TOP_AKM_DL_Tyre_Diameter),  //TOP_AKM_DL_47 - ���䰢����С����������������(������) 
		                  Velocity_KP=50,Velocity_KI=200; //PID parameter optimization //PID�����Ż�
		//if (Car_Mode==6)  Robot_Init(LONG_AKM_wheelspacing, LONG_AKM_axlespacing,   MD36N_51, Photoelectric_500, SENIOR_AKM_Tyre_Diameter);  //Customized special - ����ר��
		if (Car_Mode==7)  Robot_Init(TOP_AKM_DL_wheelspacing, TOP_AKM_DL_axlespacing,   MD60N_18, Photoelectric_500, TOP_AKM_DL_Tyre_Diameter);  //Customized special - ����ר��
		if (Car_Mode==8)  Robot_Init(TOP_AKM_DL_wheelspacing, TOP_AKM_DL_axlespacing,   MD60N_18, Photoelectric_500, TOP_AKM_DL_Tyre_Diameter);  //Customized special - ����ר��
  }
	#elif Diff_Car
	{
		if (Car_Mode==0)  Robot_Init(TOP_DIFF_wheelspacing,           0, MD36N_27, Photoelectric_500, TOP_DIFF_Tyre_Diameter);//TOP_DIFF_27 - ������ٳ�����  
		if (Car_Mode==1)  Robot_Init(TOP_DIFF_wheelspacing,           0, MD36N_51, Photoelectric_500, TOP_DIFF_Tyre_Diameter);//TOP_DIFF_51 - �������������
		
		if (Car_Mode==2)  Robot_Init(FOUR_WHEEL_DIFF_BS_wheelspacing, 0, MD60N_18, Photoelectric_500, FOUR_WHEEL_DIFF_Tyre_Diameter);//FOUR_WHEEL_DIFF_BS_18 - ����������ʽ���ҳ����� //BS: Pendulum suspension
		if (Car_Mode==3)  Robot_Init(FOUR_WHEEL_DIFF_BS_wheelspacing, 0, MD60N_47, Photoelectric_500, FOUR_WHEEL_DIFF_Tyre_Diameter);//FOUR_WHEEL_DIFF_BS_47 - ����������ʽ����������
		
		if (Car_Mode==4)  Robot_Init(FOUR_WHEEL_DIFF_DL_wheelspacing, 0, MD60N_18, Photoelectric_500, FOUR_WHEEL_DIFF_Tyre_Diameter);//FOUR_WHEEL_DIFF_BS_18 - ���������������ҳ����� //DL: Independent suspension
		if (Car_Mode==5)  Robot_Init(FOUR_WHEEL_DIFF_DL_wheelspacing, 0, MD60N_47, Photoelectric_500, FOUR_WHEEL_DIFF_Tyre_Diameter);//FOUR_WHEEL_DIFF_BS_47 - ����������������������
		if (Car_Mode==6)  Robot_Init(Wheelchair_DIFF_wheelspacing,    0, MDN1,     Photoelectric_600,    Wheelchair_Tyre_Diameter),
		Velocity_KP=100,Velocity_KI=50;
  }
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
void Robot_Init(float wheelspacing,float axlespacing,int gearratio,int Accuracy,float tyre_diameter) 
{
  Robot_Parament.WheelSpacing=wheelspacing;   //Wheelspacing �־�  
  Robot_Parament.AxleSpacing=axlespacing;     //Axlespacing ���
  Robot_Parament.GearRatio=gearratio;         //motor_gear_ratio //������ٱ�
  Robot_Parament.EncoderAccuracy=Accuracy;    //Number_of_encoder_lines //����������(����������)
  Robot_Parament.WheelDiameter=tyre_diameter; //Diameter of driving wheel //�������־�
	
	//Encoder value corresponding to 1 turn of motor (wheel)
	//���(����)ת1Ȧ��Ӧ�ı�������ֵ
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference //�������ܳ�	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI; 
	//Wheelspacing �־� 
    Wheel_spacing=Robot_Parament.WheelSpacing;  
    //Wheel_axlespacing (Wheel_axlespacing is not required for motion analysis of differential trolleys) //���(����С�����˶���������Ҫ���)	
	Axle_spacing=Robot_Parament.AxleSpacing;   
}
