#include "key.h"

/**************************************************************************
Function: Key initialization
Input   : none
Output  : none
�������ܣ�������ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	ENABLE_USER_KEY_PIN_CLOCK;
	GPIO_InitStructure.GPIO_Pin = USER_KEY_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(USER_KEY_PORT, &GPIO_InitStructure);
} 


/**************************************************************************
�������ܣ�����ɨ�躯��
��ڲ�����ִ�иú���������Ƶ�ʡ��ӳ��˲���ʱ��
����  ֵ��long_click��double_click��single_click��key_stateless��������˫������������״̬��
��    �ߣ�WHEELTEC
**************************************************************************/
u8 KEY_Scan(u16 Frequency,u16 filter_times)
{
    static u16 time_core;//��ʱ����
    static u16 long_press_time;//����ʶ��
    static u8 press_flag=0;//�������±��
    static u8 check_once=0;//�Ƿ��Ѿ�ʶ��1�α��
    static u16 delay_mini_1;
    static u16 delay_mini_2;
	
    float Count_time = (((float)(1.0f/(float)Frequency))*1000.0f);//�����1��Ҫ���ٸ�����

    if(check_once)//�����ʶ����������б���
    {
        press_flag=0;//�����1��ʶ�𣬱������
        time_core=0;//�����1��ʶ��ʱ������
        long_press_time=0;//�����1��ʶ��ʱ������
        delay_mini_1=0;
        delay_mini_2=0;
    }
    if(check_once&&KEY==1) check_once=0; //���ɨ��󰴼�������������һ��ɨ��

    if(KEY==0&&check_once==0)//����ɨ��
    {
        press_flag=1;//��Ǳ�����1��
		
        if(++delay_mini_1>filter_times)
        {
            delay_mini_1=0;
            long_press_time++;		
        }
    }

    if(long_press_time>(u16)(500.0f/Count_time))// ����1��
    {	
        check_once=1;//����ѱ�ʶ��
        return long_click; //����
    }

    //����������1���ֵ���󣬿����ں���ʱ
    if(press_flag&&KEY==1)
    {
        if(++delay_mini_2>filter_times)
        {
            delay_mini_2=0;
            time_core++; 
        }
    }		
	
    if(press_flag&&(time_core>(u16)(50.0f/Count_time)&&time_core<(u16)(300.0f/Count_time)))//50~700ms�ڱ��ٴΰ���
    {
        if(KEY==0) //����ٴΰ���
        {
            check_once=1;//����ѱ�ʶ��
            return double_click; //���Ϊ˫��
        }
    }
    else if(press_flag&&time_core>(u16)(300.0f/Count_time))
    {
        check_once=1;//����ѱ�ʶ��
        return single_click; //800ms��û�����£����ǵ���
    }

    return key_stateless;
}

