#include "buzzer.h"

/**************************************************************************
Function: Buzzer interface initialized
Input   : none
Output  : none
�������ܣ��������ӿڳ�ʼ��
��ڲ������� 
����  ֵ����
**************************************************************************/
void Buzzer_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	ENABLE_Buzzer_PIN_CLOCK;
	GPIO_InitStructure.GPIO_Pin =  Buzzer_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(Buzzer_PORT, &GPIO_InitStructure);
	
	Buzzer = BEEP_OFF;
}

/**************************************************************************
Function function: buzzer response function
Entry parameters: number of beeps, beep time interval, and frequency of executing the function
Return value: None
Author: WHEELTEC
�������ܣ���������Ӧ����
��ڲ�������������,����ʱ����,ִ�иú�����Ƶ��
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
static uint8_t Buzzer_Response(uint8_t numt,uint8_t times,uint8_t rate)
{
	uint16_t real_times = times*10;       //�û�Ҫ��ķ������ʱ��,��λms
	uint8_t taskover = 0;                 //����ͳ�������Ƿ�ִ�����
	uint16_t time_ratio = (uint16_t)( (1.0f/(float)rate) * 1000 );//ʱ����С��λ,������Ƶ�����.��λms
	static uint16_t buzzer_timecore = 0; //����ʱ��,������������ʼ�ۼ�ʱ��
	static uint8_t done_count = 0;       //��ɷ����Ĵ���
	
	if( numt==0 && times == 0 ) //������ȫ0����
	{
		taskover = 1;
		buzzer_timecore = 0;
		done_count = 0;
		Buzzer = BEEP_OFF;
		return taskover;
	}
	
	//����û�Ҫ���ʱ�����Ƿ񳬳���ʱ��С�ֱ���
	if( real_times < time_ratio ) 
	{
		//�޷������ʱ����,����Ӧ����
		taskover = 1;
		buzzer_timecore = 0;
		done_count = 0;
		Buzzer = BEEP_OFF;
		return taskover;
	}

	//������ʱ
	buzzer_timecore += (1.0f/(float)rate) * 1000 ; // ������ʱ��,��λ ms
	
	//ִ������
	if( buzzer_timecore<real_times )
	{
		Buzzer = BEEP_ON;
	}
	else if( buzzer_timecore>=real_times && buzzer_timecore<real_times*2 )
	{
		Buzzer = BEEP_OFF;
	}
	else if( buzzer_timecore>=real_times*2 )
	{
		buzzer_timecore = 0;//���¼�ʱ
		done_count ++ ;     //��¼���һ�η���
		if(done_count==numt)
		{
			Buzzer = BEEP_OFF;    //ȷ�����������ᷢ��
			done_count = 0;
			taskover = 1;
			return taskover;//�������
		}
	}
	return taskover;
}


/* ------------------ ���������������ر���,����ֱ���޸�. ------------------- */
#define BUZZER_QUEUE_LEN 10 //������������еĳ���

//���������������.�� BUZZER_QUEUE_LEN �� ( ��������(u8) , ����ʱ��(u8)) ������
static uint8_t buzzer_task_buf[BUZZER_QUEUE_LEN][2];
static uint8_t buzzer_NowLen = 0;//��������ǰ������г���
static uint8_t buzzer_task_tail = 0;//����β��
static uint8_t buzzer_task_head = 0;//����ͷ
/* ----------------------------------------------------------------------- */

/**************************************************************************
Functionality: Add a task to the buzzer execution queue.
Input Parameters: Number of buzzes, time interval between each buzz (in units of *10ms).
Return Value: 0: Task addition failed, 1: Task addition succeeded.
Author: WHEELTEC
�������ܣ��������񵽷�����ִ��������
��ڲ���������������ÿ�η�����ʱ��������λ*10ms��
����  ֵ��0:�������ʧ�� 1:������ӳɹ�
��    �ߣ�WHEELTEC
**************************************************************************/
uint8_t Buzzer_AddTask(uint8_t num_time,uint8_t times )
{
	//�����������Ƿ�����
	if( buzzer_NowLen >= BUZZER_QUEUE_LEN )
	{
		return 0;
	}
	else
	{
		//�������񵽻��ζ���β��
		buzzer_task_buf[buzzer_task_tail][0] = num_time;
		buzzer_task_buf[buzzer_task_tail][1] = times;
		
		//����β����������
		buzzer_task_tail = (buzzer_task_tail + 1)%BUZZER_QUEUE_LEN;
		
		//ͳ�ƶ��������ݳ���
		buzzer_NowLen++;
	}
	return 1;
}



/**************************************************************************
Functionality: Buzzer task execution, retrieving and executing tasks added by the
               user through the Buzzer_AddTask() function one by one from the task queue,
               with a fixed time interval of 1 second between each task.
Return Value: None.
Author: WHEELTEC
�������ܣ����������񣬽��û�ͨ������ Buzzer_AddTask() ��ӵ������ȡ�������������ִ��.����ÿ������֮��ʱ�����̶�Ϊ1��
��ڲ�����ִ�з��������Ƶ��
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
void Buzzer_task(uint8_t rate)
{
	static uint8_t task_over = 1;
	static uint8_t task_delay = 0;//����������֮���ʱ����
	static uint8_t start_delay = 0;//��������ɱ�־λ. 0��ִ������,1�ȴ���
	
	//����������ݴ�Ŵ�
	static uint8_t task[2]={0,0};
	
	if( start_delay==0 ) //��������ʱ���
	{
		//���зǿ��ҵ�ǰû�з���������ִ��,��ȡ����������е�����˳��ִ��
		if( buzzer_NowLen!=0 && task_over==1 ) 
		{
			//ȡ��һ������
			task[0] = buzzer_task_buf[buzzer_task_head][0];
			task[1] = buzzer_task_buf[buzzer_task_head][1];
			
			//���ζ���ͷ����
			buzzer_task_head = (buzzer_task_head + 1)%BUZZER_QUEUE_LEN;
			
			//ͳ�ƶ��������ݳ���
			buzzer_NowLen --;
		}
	}

	//����ȡ����,��ʼִ��
	task_over = Buzzer_Response(task[0],task[1],rate);
	
	//���������������ֵ
	if( task_over==1 ) task[0]=0,task[1]=0,start_delay=1;
	
	//����ȴ�������������֮��,�ȴ�1����������
	if(start_delay)
	{
		task_delay++;
		if( task_delay>=rate*1 ) task_delay = 0,start_delay=0;
	}
	
}
