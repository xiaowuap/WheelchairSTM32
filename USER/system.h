#ifndef __SYSTEM_H
#define __SYSTEM_H

//���ó�����ʼƫ�Ƶ�ַ. ������ʼ��ַ = 0x8000000 + VECT_TAB_OFFSET
#define VECT_TAB_OFFSET  0x00000

//оƬͷ�ļ�
#include "stm32f4xx.h"

//C library function related header file
//C�⺯�������ͷ�ļ�
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"

//FreeRTOS related header files
// FreeRTOS���ͷ�ļ�
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

//Board level support package related header files
//�弶֧�ְ�(Ӳ��)���ͷ�ļ�
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "adc.h"
#include "can.h"
#include "encoder.h"
#include "key.h"
#include "LED.h"
#include "motor.h"
#include "oled.h"
#include "pstwo.h"
#include "stmflash.h"
#include "auto_recharge.h"
#include "buzzer.h"
#include "enable_key.h"
#include "uartx.h"
#include "I2C.h"
#include "MPU6050.h"
#include "ICM20948.h"
#include "remote.h"
#include "usb_host.h"

//�����˱���������ͷ�ļ�
#include "robot_init.h"

//Main logic code related header files
//���߼����ͷ�ļ�
#include "balance_task.h"
#include "imu_task.h"
#include "show_task.h"
#include "led_task.h"
#include "ps2_task.h"
#include "data_task.h"


//Check the multiple vehicle model definitions and ensure that only one model is allowed to exist at a time.
//�Զ��س��Ͷ�����,������������ͬʱ����.
#if defined AKM_CAR + defined DIFF_CAR + defined MEC_CAR + defined _4WD_CAR + defined OMNI_CAR > 1
	#error "ERROR: multiple vehicle model definitions."
#endif

//����ӿ�
void systemInit(void);

/***Macros define***/ /***�궨��***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//����(1000/100hz=10)�����������С�������˶�

//TODO:�ָ���1000
#define CONTROL_DELAY		1000

//RTOS����Ƶ��
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000


#endif 
