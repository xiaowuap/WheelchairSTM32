#ifndef __DATA_TASK_H
#define __DATA_TASK_H

#include "system.h"

#define DATA_TASK_PRIO		4             //Task priority //�������ȼ�
#define DATA_STK_SIZE 		512           //Task stack size //�����ջ��С
#define DATA_TASK_RATE      RATE_20_HZ   //����Ƶ��


//����24�ֽ�֡ͷ��֡β�����ݳ���
#define FRAME_HEADER      0X7B //Frame_header //֡ͷ
#define FRAME_TAIL        0X7D //Frame_tail   //֡β
#define SEND_DATA_SIZE    24

//�Զ��س�֡ͷ��֡β�����ݳ���
#define AutoCharge_HEADER      0X7C //Frame_header //֡ͷ
#define AutoCharge_TAIL        0X7F //Frame_tail   //֡β
#define AutoCharge_DATA_SIZE    8

//IMU��������
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2���ֽ�
	short Y_data; //2 bytes //2���ֽ�
	short Z_data; //2 bytes //2���ֽ�
}Mpu6050_Data;

//�����������ٶ�
typedef struct __Robot_Data_ 
{
	short X_speed; //2 bytes //2���ֽ�
	short Y_speed; //2 bytes //2���ֽ�
	short Z_speed; //2 bytes //2���ֽ�
}Robot_Vel;

//�������ݽṹ�嶨��
typedef struct _SEND_DATA_  
{
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1���ֽ�
		Robot_Vel    Vel;            //6�ֽ�
		Mpu6050_Data Accelerometer;  //6 bytes //6���ֽ�
		Mpu6050_Data Gyroscope;      //6 bytes //6���ֽ�	
		short Power_Voltage;        //2 bytes //2���ֽ�
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}Sensor_Str;
	
	unsigned char buffer[SEND_DATA_SIZE];//ʵ�����ڷ������ݵĻ�����
}SEND_DATA;

//�������ݽṹ�嶨��
typedef struct _SEND_AutoCharge_DATA_  
{
	unsigned char buffer[AutoCharge_DATA_SIZE];
	struct _AutoCharge_Str_
	{
		unsigned char Frame_Header; //1 bytes //1���ֽ�
		short Charging_Current;	    //2 bytes //2���ֽ�
		unsigned char RED;          //1 bytes //1���ֽ�
		unsigned char Charging;     //1 bytes //1���ֽ�
		unsigned char yuliu;		//1 bytes //1���ֽ�
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}AutoCharge_Str;
}SEND_AutoCharge_DATA;

//����ʹ�ñ���
uint8_t Check_BCC(const uint8_t *data, uint16_t length);
void data_task(void *pvParameters);

extern TaskHandle_t data_TaskHandle ;

//�ڲ�ʹ�ú���
static float* Kinematics_akm_diff(float motorA,float motorB);
static float* Kinematics_omni(float motorA,float motorB,float motorC);
static float* Kinematics_mec_4wd(float motorA,float motorB,float motorC,float motorD);
static void data_transition(void);
static void Usart1_SendTask(void);
static void Usart3_SendTask(void);
static void CAN1_SendTask(void);

#endif
