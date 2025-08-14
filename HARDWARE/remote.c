#include "remote.h"

REMOTER_t remoter;

/**************************************************************************
Function: Model aircraft remote control initialization function, timer 1 input capture initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
�������ܣ���ģң�س�ʼ����������ʱ��1���벶���ʼ��
��ڲ�����arr���Զ���װֵ��psc��ʱ��Ԥ��Ƶ��
�� �� ֵ����
**************************************************************************/
//����ͨ�����ã����������������Ϊ�����������
static void GPIO_AFPP_Init(GPIO_TypeDef* GPIOx,uint16_t PINx)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   =  PINx;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

//���Ÿ�������
static void GPIOx_PIN_AFConfig(GPIO_TypeDef* GPIOx,uint16_t PINx,TIM_TypeDef* TIMx)
{
	//ȷ�ϸ��õ����ź�
	uint8_t PinSource=0;
	for(PinSource=0;PinSource<16;PinSource++)
	{
		if( ( (PINx>>PinSource)&0x01 )==1 ) break;
	}
	
	//ȷ�ϸ��õĶ�ʱ��
	//��TIM6��7��ͨ����,��������ͨ��,��Ӧʹ�ø��ù���
	uint8_t GPIO_AF;
	      if( TIMx == TIM1 || TIMx == TIM2 )                                       GPIO_AF = 0x01;
	else if( TIMx == TIM3  || TIMx == TIM4  || TIMx == TIM5 )                     GPIO_AF = 0x02;
	else if( TIMx == TIM8  || TIMx == TIM9  || TIMx == TIM10 || TIMx == TIM11 )   GPIO_AF = 0x03;
	else if( TIMx == TIM12 || TIMx == TIM13 || TIMx == TIM14)                     GPIO_AF = 0x09;
	
	//��������
	GPIO_PinAFConfig(GPIOx,PinSource,GPIO_AF);
}

void Remoter_Param_Init(REMOTER_t *p)
{
	p->ch1 = 1500;
	p->ch2 = 1500;
	p->ch3 = 1000;
	p->ch4 = 1500;
	p->check_count = 0;
}

void Remoter_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
	
	//�����붨ʱ��ʱ�ӿ���
	ENABLE_REMOTE_TIM_CLOCK;
	ENABLE_REMOTE_CH1_PORT_CLOCK;
	ENABLE_REMOTE_CH2_PORT_CLOCK;
	ENABLE_REMOTE_CH3_PORT_CLOCK;
	ENABLE_REMOTE_CH4_PORT_CLOCK;
	
	//����Ӧ��������Ϊ�������
	GPIO_AFPP_Init(REMOTE_CH1_PORT,REMOTE_CH1_PIN);
	GPIO_AFPP_Init(REMOTE_CH2_PORT,REMOTE_CH2_PIN);
	GPIO_AFPP_Init(REMOTE_CH3_PORT,REMOTE_CH3_PIN);
	GPIO_AFPP_Init(REMOTE_CH4_PORT,REMOTE_CH4_PIN);
	
	//���Ÿ�������
	GPIOx_PIN_AFConfig(REMOTE_CH1_PORT,REMOTE_CH1_PIN,REMOTE_TIM);
	GPIOx_PIN_AFConfig(REMOTE_CH2_PORT,REMOTE_CH2_PIN,REMOTE_TIM);
	GPIOx_PIN_AFConfig(REMOTE_CH3_PORT,REMOTE_CH3_PIN,REMOTE_TIM);
	GPIOx_PIN_AFConfig(REMOTE_CH4_PORT,REMOTE_CH4_PIN,REMOTE_TIM);
	

    /*** Initialize timer 1 || ��ʼ����ʱ��1 ***/
    //Set the counter to automatically reload //�趨�������Զ���װֵ
    TIM_TimeBaseStructure.TIM_Period = 9999;     //��װ��ֵ,��������10000������װ��
    //Pre-divider //Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_Prescaler = 167;   //Ԥ��Ƶ168���߼���ʱ�� 168M/168 = 1M ==> 1us�ɼ���1
    //Set the clock split: TDTS = Tck_tim //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //TIM up count mode //TIM���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //Initializes the timebase unit for TIMX based on the parameter specified in TIM_TimeBaseInitStruct
    //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
    TIM_TimeBaseInit(REMOTE_TIM, &TIM_TimeBaseStructure);

    /*** ��ʼ��TIM1���벶�������ͨ��1 || Initialize TIM1 for the capture parameter, channel 1 ***/
    //Select input //ѡ�������
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    //Rising edge capture //�����ز���
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    //Configure input frequency division, regardless of frequency //���������Ƶ,����Ƶ
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    //IC1F=0000 Configure input filter //���������˲���
    TIM_ICInitStructure.TIM_ICFilter = 0x0F;
    TIM_ICInit(REMOTE_TIM, &TIM_ICInitStructure);

    /*** ��ʼ��TIM1���벶�������ͨ��2 || Initialize TIM1 for the capture parameter, channel 2 ***/
    //CC1S=01 Select input //ѡ�������
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    //Rising edge capture //�����ز���
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    //Configure input frequency division, regardless of frequency //���������Ƶ,����Ƶ
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 ���������˲���
    TIM_ICInit(REMOTE_TIM, &TIM_ICInitStructure);

    /*** ��ʼ��TIM1���벶�������ͨ��3 || Initialize TIM1 for the capture parameter, channel 3 ***/
    //Select input //ѡ�������
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
    //Rising edge capture //�����ز���
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    //Configure input frequency division, regardless of frequency //���������Ƶ,����Ƶ
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    //IC1F=0000 Configure input filter //���������˲��������˲�
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInit(REMOTE_TIM, &TIM_ICInitStructure);

    /*** ��ʼ��TIM1���벶�������ͨ��4 || Initialize TIM1 for the capture parameter, channel 4 ***/
    //Select input //ѡ�������
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
    //Rising edge capture //�����ز���
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    //Configure input frequency division, regardless of frequency //���������Ƶ,����Ƶ
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    //IC1F=0000 Configure input filter //���������˲��������˲�
    TIM_ICInitStructure.TIM_ICFilter = 0x00;
    TIM_ICInit(REMOTE_TIM, &TIM_ICInitStructure);

    /*** interrupt packet initialization || �жϷ����ʼ�� ***/
    //TIM1 interrupts //TIM1�ж�
    NVIC_InitStructure.NVIC_IRQChannel = REMOTE_TIM_IRQn ;
    //Preempt priority 0 //��ռ���ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    //Level 0 from priority //�����ȼ�0��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //IRQ channels are enabled //IRQͨ����ʹ��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initializes the peripheral NVIC register according to the parameters specified in NVIC_InitStruct
    //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
    NVIC_Init(&NVIC_InitStructure);

    //Allow CC1IE,CC2IE,CC3IE,CC4IE to catch interrupts, not allowed update_interrupts
    //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�
    TIM_ITConfig(REMOTE_TIM, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);

    //Enable timer //ʹ�ܶ�ʱ��
    TIM_Cmd(REMOTE_TIM, ENABLE);
}
