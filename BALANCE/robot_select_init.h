#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//�����˲����ṹ��
typedef struct  
{
  float WheelSpacing;      //Wheelspacing, Mec_Car is half wheelspacing //�־� ���ֳ�Ϊ���־�
  float AxleSpacing;       //Axlespacing, Mec_Car is half axlespacing //���? ���ֳ�Ϊ�����?	
  int   GearRatio;         //Motor_gear_ratio //������ٱ�?
  int   EncoderAccuracy;   //Number_of_encoder_lines //����������(����������)
  float WheelDiameter;     //Diameter of driving wheel //������ֱ��	
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional trolley //ȫ����С����ת�뾶
}Robot_Parament_InitTypeDef;

//The minimum turning radius of different Ackermann models is determined by the mechanical structure:
//the maximum Angle of the wheelbase, wheelbase and front wheel
//��ͬ���������͵���Сת��뾶���ɻ�е�ṹ�������־�?��ࡢǰ�����ת��
#define   SENIOR_AKM_MIN_TURN_RADIUS       0.750f 
#define   TOP_AKM_BS_MIN_TURN_RADIUS       1.400f 
#define   TOP_AKM_DL_MIN_TURN_RADIUS       1.200f 

//Axle_spacing //���?
#define   SENIOR_AKM_axlespacing           0.322f 
#define   TOP_AKM_BS_axlespacing           0.590f 
#define   TOP_AKM_DL_axlespacing           0.530f 

//Wheel_spacing //�־�
#define   SENIOR_AKM_wheelspacing          0.322f  
#define   TOP_AKM_BS_wheelspacing          0.508f 
#define   TOP_AKM_DL_wheelspacing          0.585f 
#define   TOP_DIFF_wheelspacing            0.329f
#define   FOUR_WHEEL_DIFF_BS_wheelspacing  0.573f
#define   FOUR_WHEEL_DIFF_DL_wheelspacing  0.573f
#define   Wheelchair_DIFF_wheelspacing     0.600f

//Motor_gear_ratio
//������ٱ�?
#define   MD36N_5_18  5.18
#define   MD36N_27    27
#define   MD36N_51    51
#define   MD36N_71    71
#define   MD60N_18    18
#define   MD60N_47    47
#define   MDN27_3     27.3
#define   MDN1        1

//Number_of_encoder_lines
//����������
#define		Photoelectric_500 500
#define	  Hall_13 13
#define	  Hall_8  8

//Diameter of trolley tire
//С����ֱ̥��
#define   SENIOR_AKM_Tyre_Diameter      0.125
#define   TOP_AKM_BS_WHEEL_Diameter     0.180
#define   TOP_AKM_DL_Tyre_Diameter      0.254
#define   TOP_DIFF_Tyre_Diameter        0.125
#define   FOUR_WHEEL_DIFF_Tyre_Diameter 0.215
#define   Wheelchair_Tyre_Diameter      0.300

//Mecanum wheel tire diameter series
//������ֱ̥��
#define		Mecanum_60  60
#define		Mecanum_75  75
#define		Mecanum_100 100
#define		Mecanum_127 127
#define		Mecanum_152 152
	   
//Omni wheel tire diameter series
//�־�ȫ����ֱ��ϵ��
#define	  FullDirecion_75  75
#define	  FullDirecion_127 127
#define	  FullDirecion_152 152
#define	  FullDirecion_203 203
#define	  FullDirecion_217 217


//The encoder octave depends on the encoder initialization Settings
//��������Ƶ����ȡ���ڱ�������ʼ������
#define   EncoderMultiples 4
//Encoder data reading frequency
//���������ݶ�ȡƵ��
#define CONTROL_FREQUENCY 100

void Robot_Select(void);
void Robot_Init(float wheelspacing,float axlespacing,int gearratio,int Accuracy,float tyre_diameter) ;

#endif
