#ifndef __REMOTE_H
#define __REMOTE_H
#include "sys.h"

//�����жϵ���ֵ.����ģ�Ļ���ͳ��ֵ��������ֵ������Ϊ��ǰ�Ǹ����źŻ������޺�ģ�ź�,��λ��ģң�ص���ֵ,��ֹС������ʧ����Ϊ.
//��ֵԽС���Խ����,��ֵԽ��Խ����.
#define Filter_Threshold 10

//ң����صı���
typedef struct  
{
	int ch1; //��ģͨ��1~4��ȡ������ֵ,��������
	int ch2; 
	int ch3;
	int ch4;
	uint16_t check_count; //��ģң��������ͳ��,���ڼ���Ƿ�����ʵ�ĺ�ģ�ź�,���˸���
}REMOTER_t;

//����ӿ�
void Remoter_Init(void);
extern REMOTER_t remoter;
void Remoter_Param_Init(REMOTER_t *p);


/*--------Remote User config--------*/
//��ģң������
#define ENABLE_REMOTE_TIM_CLOCK   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE)  //PWMAʱ��ʹ��

#define ENABLE_REMOTE_CH1_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE) //PWMA��Ӧ�����Ŷ˿�ʹ��
#define ENABLE_REMOTE_CH2_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE) //PWMA��Ӧ�����Ŷ˿�ʹ��
#define ENABLE_REMOTE_CH3_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE) //PWMA��Ӧ�����Ŷ˿�ʹ��
#define ENABLE_REMOTE_CH4_PORT_CLOCK  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE) //PWMA��Ӧ�����Ŷ˿�ʹ��
	
#define REMOTE_TIM            TIM1	
#define REMOTE_TIM_IRQn       TIM1_CC_IRQn        //��ģ���ݲɼ���Ӧ�Ķ�ʱ���ж�
#define REMOTE_TIM_IRQHandler TIM1_CC_IRQHandler //��ģ���ݲɼ���Ӧ���жϷ�����

#define REMOTE_CH1_PORT      GPIOE            
#define REMOTE_CH1_PIN       GPIO_Pin_9       

#define REMOTE_CH2_PORT      GPIOE            
#define REMOTE_CH2_PIN       GPIO_Pin_11

#define REMOTE_CH3_PORT      GPIOE           
#define REMOTE_CH3_PIN       GPIO_Pin_13 

#define REMOTE_CH4_PORT      GPIOE            
#define REMOTE_CH4_PIN       GPIO_Pin_14 

/*----------------------------------*/


//��ģң�ؽ��� ͨ����Ӧ�Ķ�ʱ��ͨ��
#define Set_CH1_Rising  TIM_OC1PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Rising)  //����������
#define Set_CH2_Rising  TIM_OC2PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Rising)
#define Set_CH3_Rising  TIM_OC3PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Rising)
#define Set_CH4_Rising  TIM_OC4PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Rising)

#define Set_CH1_Falling TIM_OC1PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Falling)  //�½�������
#define Set_CH2_Falling TIM_OC2PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Falling)
#define Set_CH3_Falling TIM_OC3PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Falling)
#define Set_CH4_Falling TIM_OC4PolarityConfig(REMOTE_TIM, TIM_ICPolarity_Falling)

#define Get_CH1_CNT TIM_GetCapture1(REMOTE_TIM) //��ȡ��������ֵ
#define Get_CH2_CNT TIM_GetCapture2(REMOTE_TIM)
#define Get_CH3_CNT TIM_GetCapture3(REMOTE_TIM)
#define Get_CH4_CNT TIM_GetCapture4(REMOTE_TIM)

#define Get_CH1_State TIM_GetITStatus(REMOTE_TIM, TIM_IT_CC1) //��ȡ�жϵ�״̬
#define Get_CH2_State TIM_GetITStatus(REMOTE_TIM, TIM_IT_CC2)
#define Get_CH3_State TIM_GetITStatus(REMOTE_TIM, TIM_IT_CC3)
#define Get_CH4_State TIM_GetITStatus(REMOTE_TIM, TIM_IT_CC4)

#define Clear_CH1_State TIM_ClearITPendingBit(REMOTE_TIM, TIM_IT_CC1) //����жϱ�־λ
#define Clear_CH2_State TIM_ClearITPendingBit(REMOTE_TIM, TIM_IT_CC2)
#define Clear_CH3_State TIM_ClearITPendingBit(REMOTE_TIM, TIM_IT_CC3)
#define Clear_CH4_State TIM_ClearITPendingBit(REMOTE_TIM, TIM_IT_CC4)


void Remoter_Init(void);


#endif
