#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"

/*--------moto Interface Fun --------*/
//���⿪�ŵĽӿ�
#define SERVO_INIT     1500                             //Servo zero point //������
#define FULL_DUTYCYCLE 16800                            //���ӵ������ռ�ձ���ֵ,���ʼ���������.
void MiniBalance_PWM_Init(u16 arr,u16 psc);            //ֱ�������ʼ��
void V1_0_MiniBalance_PWM_Init(u16 arr,u16 psc);      //ֱ�������ʼ��
void Servo_Senior_Init(u16 arr,u16 psc,int servomid); //���䰢�������Ͷ����ʼ��
void Servo_Top_Init(u16 arr,u16 psc,int servomid);    //���䰢�������Ͷ����ʼ��
/*--------------------------------------------*/


//�û�����User config,�����漰�����޸�ʱ,���������Ÿ���,���޸ĸ��ļ��µ����Ų�������
/*--------Motor_A User config--------*/
//PWM��������
#define ENABLE_PWMA_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE)  //PWMAʱ��ʹ��
#define ENABLE_PWMA_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE) //PWMA��Ӧ�����Ŷ˿�ʹ��
	
#define PWMA_PORT      GPIOC            //PWMA���Ŷ˿�
#define PWMA_PIN       GPIO_Pin_9       //PWMA����
#define PWMA_TIM       TIM8             //PWMAʹ�õĶ�ʱ��
#define PWMA_Channel   4                //ʹ���˶�ʱ������һ��ͨ��CCRx(1~4)

//���������������
#define ENABLE_AIN1_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)
#define ENABLE_AIN2_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)

#define AIN1 	            PBout(13)//AIN1
#define AIN1_PORT           GPIOB
#define AIN1_PIN            GPIO_Pin_13

#define AIN2 	            PBout(12)//AIN2
#define AIN2_PORT           GPIOB
#define AIN2_PIN            GPIO_Pin_12
/*----------------------------------*/

/*--------Motor_B User config--------*/
//PWM��������
#define ENABLE_PWMB_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE)  //PWMBʱ��ʹ��
#define ENABLE_PWMB_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE) //PWMB��Ӧ�����Ŷ˿�ʹ��
	
#define PWMB_PORT      GPIOC            //PWMB���Ŷ˿�
#define PWMB_PIN       GPIO_Pin_8       //PWMB����
#define PWMB_TIM       TIM8             //PWMBʹ�õĶ�ʱ��
#define PWMB_Channel   3                //ʹ���˶�ʱ������һ��ͨ��CCRx(1~4)

//���������������
#define ENABLE_BIN1_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE)
#define ENABLE_BIN2_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE)

#define BIN1 	            PCout(0)//BIN1
#define BIN1_PORT           GPIOC
#define BIN1_PIN            GPIO_Pin_0

#define BIN2 	            PEout(15)//BIN2
#define BIN2_PORT           GPIOE
#define BIN2_PIN            GPIO_Pin_15
/*----------------------------------*/

/*--------Motor_C User config--------*/
//PWM��������
#define ENABLE_PWMC_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE) //PWMCʱ��ʹ��
#define ENABLE_PWMC_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE) //PWMC��Ӧ�����Ŷ˿�ʹ��
	
#define PWMC_PORT      GPIOC            //PWMC���Ŷ˿�
#define PWMC_PIN       GPIO_Pin_7       //PWMC����
#define PWMC_TIM       TIM8             //PWMCʹ�õĶ�ʱ��
#define PWMC_Channel   2                //ʹ���˶�ʱ������һ��ͨ��CCRx(1~4)

//���������������
#define ENABLE_CIN1_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE)
#define ENABLE_CIN2_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE)

#define CIN1 	        PDout(10)//CIN1
#define CIN1_PORT       GPIOD
#define CIN1_PIN        GPIO_Pin_10

#define CIN2 	        PDout(12)//CIN2
#define CIN2_PORT       GPIOD
#define CIN2_PIN        GPIO_Pin_12
/*----------------------------------*/

/*--------Motor_D User config--------*/
//PWM��������
#define ENABLE_PWMD_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE) //PWMDʱ��ʹ��
#define ENABLE_PWMD_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE) //PWMD��Ӧ�����Ŷ˿�ʹ��
	
#define PWMD_PORT      GPIOC            //PWMD���Ŷ˿�
#define PWMD_PIN       GPIO_Pin_6       //PWMD����
#define PWMD_TIM       TIM8             //PWMDʹ�õĶ�ʱ��
#define PWMD_Channel   1                //ʹ���˶�ʱ������һ��ͨ��CCRx(1~4)

//���������������
#define ENABLE_DIN1_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE)
#define ENABLE_DIN2_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE)

#define DIN1 	        PCout(12)//DIN1
#define DIN1_PORT       GPIOC
#define DIN1_PIN        GPIO_Pin_12

#define DIN2 	        PAout(8) //DIN2
#define DIN2_PORT       GPIOA
#define DIN2_PIN        GPIO_Pin_8
/*----------------------------------*/

/*--------Senior akm Servo User config--------*/
//���䰢�������Ͷ������
//PWM��������
#define ENABLE_SERVO_SENIOR_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE)  //�����ʱ��ʱ��ʹ��
#define ENABLE_SERVO_SENIOR_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE) //�����Ӧ�����Ŷ˿�ʹ��
	
#define SERVO_SENIOR_PORT      GPIOA            //������Ŷ˿�
#define SERVO_SENIOR_PIN       GPIO_Pin_2       //�������
#define SERVO_SENIOR_TIM       TIM9             //���ʹ�õĶ�ʱ��
#define SERVO_SENIOR_Channel   1                //ʹ���˶�ʱ������һ��ͨ��CCRx(1~4)
/*--------------------------------------------*/

/*--------Top akm Servo User config--------*/
//���䰢�������Ͷ������
#define ENABLE_SERVO_TOP_TIM_CLOCK   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE) //���ʱ��ʹ��
#define ENABLE_SERVO_TOP_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE) //�����Ӧ�����Ŷ˿�ʹ��
	
#define SERVO_TOP_PORT      GPIOB            //������Ŷ˿�
#define SERVO_TOP_PIN       GPIO_Pin_7       //�������
#define SERVO_TOP_TIM       TIM4             //���ʹ�õĶ�ʱ��
#define SERVO_TOP_Channel   2                //ʹ���˶�ʱ������һ��ͨ��CCRx(1~4)
/*--------------------------------------------*/

/*-------- V1.0 Motor_B User config--------*/
//V1.00�汾Ӳ���ĵ��������������
#define V1_0_ENABLE_BIN1_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE)
#define V1_0_ENABLE_BIN2_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE)

#define V1_0_BIN1 	              PCout(0)//BIN1
#define V1_0_BIN1_PORT            GPIOC
#define V1_0_BIN1_PIN             GPIO_Pin_0

#define V1_0_BIN2 	              PBout(14)//BIN2
#define V1_0_BIN2_PORT            GPIOB
#define V1_0_BIN2_PIN             GPIO_Pin_14
/*----------------------------------*/



//�����û��������ɵ�����,����Ҫ�û��޸�
//The configuration generated based on user configuration does not require user modification
#if PWMA_Channel==1
	#define PWMA           PWMA_TIM->CCR1
#elif PWMA_Channel == 2
	#define PWMA           PWMA_TIM->CCR2
#elif PWMA_Channel == 3
	#define PWMA           PWMA_TIM->CCR3
#elif PWMA_Channel == 4
	#define PWMA           PWMA_TIM->CCR4
#else
	#error "PWM_A Channel configuration error"
#endif

#if PWMB_Channel==1
	#define PWMB           PWMB_TIM->CCR1
#elif PWMB_Channel == 2
	#define PWMB           PWMB_TIM->CCR2
#elif PWMB_Channel == 3
	#define PWMB           PWMB_TIM->CCR3
#elif PWMB_Channel == 4
	#define PWMB           PWMB_TIM->CCR4
#else
	#error "PWM_B Channel configuration error"
#endif

#if PWMC_Channel==1
	#define PWMC           PWMC_TIM->CCR1
#elif PWMC_Channel == 2
	#define PWMC           PWMC_TIM->CCR2
#elif PWMC_Channel == 3
	#define PWMC           PWMC_TIM->CCR3
#elif PWMC_Channel == 4
	#define PWMC           PWMC_TIM->CCR4
#else
	#error "PWM_C Channel configuration error"
#endif

#if PWMD_Channel==1
	#define PWMD           PWMD_TIM->CCR1
#elif PWMD_Channel == 2
	#define PWMD           PWMD_TIM->CCR2
#elif PWMD_Channel == 3
	#define PWMD           PWMD_TIM->CCR3
#elif PWMD_Channel == 4
	#define PWMD           PWMD_TIM->CCR4
#else
	#error "PWM_D Channel configuration error"
#endif

#if SERVO_SENIOR_Channel==1
	#define SERVO_SENIOR           SERVO_SENIOR_TIM->CCR1
#elif SERVO_SENIOR_Channel == 2
	#define SERVO_SENIOR           SERVO_SENIOR_TIM->CCR2
#elif SERVO_SENIOR_Channel == 3
	#define SERVO_SENIOR           SERVO_SENIOR_TIM->CCR3
#elif SERVO_SENIOR_Channel == 4
	#define SERVO_SENIOR           SERVO_SENIOR_TIM->CCR4
#else
	#error "SERVO_SENIOR_Channel Channel configuration error"
#endif

#if SERVO_TOP_Channel==1
	#define SERVO_TOP           SERVO_TOP_TIM->CCR1
#elif SERVO_TOP_Channel == 2
	#define SERVO_TOP           SERVO_TOP_TIM->CCR2
#elif SERVO_TOP_Channel == 3
	#define SERVO_TOP           SERVO_TOP_TIM->CCR3
#elif SERVO_TOP_Channel == 4
	#define SERVO_TOP           SERVO_TOP_TIM->CCR4
#else
	#error "SERVO_TOP_Channel Channel configuration error"
#endif


/*---------------------------------------------------------------*/

#endif
