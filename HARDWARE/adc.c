#include "adc.h"
#include "motor.h"

//������ת�򻬹�����
AKM_SERVO_ADC Akm_Servo;

//ʹ��DMA�ɼ���������
#define akm_servo_bufsize 80 //ÿ��ADC���ݹ��ɼ��ĸ���
static uint16_t adc_OriBuf[akm_servo_bufsize][2]; //����λ�� �� ���������λ�� ��ԭʼ��ֵ(ADC��ֵ)

/**************************************************************************
Function: ADC initializes battery voltage detection
Input   : none
Output  : none
�������ܣ�ADC��ʼ����ص�ѹ���
��ڲ�������
����  ֵ����
**************************************************************************/
static void ADC1_Pin_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	ENABLE_BATTERY_PIN_CLOCK;
    GPIO_InitStructure.GPIO_Pin = Battery_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
    GPIO_Init(Battery_PORT, &GPIO_InitStructure);
	
	ENABLE_CarMode_PIN_CLOCK;
    GPIO_InitStructure.GPIO_Pin = CarMode_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
    GPIO_Init(CarMode_PORT, &GPIO_InitStructure);
}


static void ADC2_Pin_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	ENABLE_SLIDE_PIN_CLOCK;
    GPIO_InitStructure.GPIO_Pin = SLIDE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
    GPIO_Init(SLIDE_PORT, &GPIO_InitStructure);
	
	ENABLE_ServoAdj_PIN_CLOCK;
    GPIO_InitStructure.GPIO_Pin = ServoAdj_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
    GPIO_Init(ServoAdj_PORT, &GPIO_InitStructure);
}

static void ADCx_DMAConfig_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    ENABLE_DMAx_CLOCK;

    DMA_DeInit(DMAx_Streamx);
    while(DMA_GetCmdStatus(DMAx_Streamx)!=DISABLE); //�ȴ�DMA������
    DMA_InitStructure.DMA_Channel = DMA_ChannelX; //ѡ��DMAͨ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = DMA_REG_ADDR; //����ĵ�ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)adc_OriBuf; //�洢���ĵ�ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; //���� �� �洢�� ģʽ
    DMA_InitStructure.DMA_BufferSize = akm_servo_bufsize*2; //Ҫ������ֽ���
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //�洢����ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //���贫�����(16λ)
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //�洢���������
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //ѭ��ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //�����ȼ�

    //////////// FIFO����Disable����������һ�²��� ////////////
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
    //////////// *** ////////////
    DMA_Init(DMAx_Streamx, &DMA_InitStructure);//��ʼ��DMA

    DMA_Cmd(DMAx_Streamx, ENABLE);  //����DMA����
}

void ADC1_Init(void)
{
	ADC1_Pin_Init();//ʹ��ADC1��������
	
    ADC_CommonInitTypeDef  ADC_CommonInitStructure;
    ADC_InitTypeDef        ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ�� 
	
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);  //��λ����

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                     //����ģʽ
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; //���������׶�֮����ӳ�5��ʱ��
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;      //DMAʧ��
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;                  //Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz
    ADC_CommonInit(&ADC_CommonInitStructure);                                    //��ʼ��

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                       //12λģʽ
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;                                //��ɨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                          //�ر�����ת��
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;  //��ֹ������⣬ʹ���������
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                       //�Ҷ���
    ADC_InitStructure.ADC_NbrOfConversion = 1;                                   //1��ת���ڹ��������� Ҳ����ֻת����������1

    ADC_Init(ADC1, &ADC_InitStructure);//ADC1��ʼ��
    ADC_Cmd(ADC1, ENABLE);//����ADת����
}

void ADC2_Init(void)
{
	ADC2_Pin_Init();//ADC2ʹ������������
	
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef       ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); //ʹ��ADC2ʱ��

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);	   	 //ADC2��λ
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE);	 //��λ����

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                    //����ģʽ
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;     //DMAʧ��
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;                 //Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz
    ADC_CommonInit(&ADC_CommonInitStructure);                                   //��ʼ��

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;                      //12λģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                //��ͨ��ʱ,��Ҫʹ��ɨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                          //����ת��
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //��ֹ������⣬ʹ���������
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                      //�Ҷ���
    ADC_InitStructure.ADC_NbrOfConversion = 2;                                  //1��ת���ڹ��������� Ҳ����ֻת����������1
    ADC_Init(ADC2, &ADC_InitStructure);                                         //ADC��ʼ��

    //���ù���ͨ����DMA����
    ADC_RegularChannelConfig(ADC2,SLIDE_Ch,1,ADC_SampleTime_480Cycles);    //��һ�ɼ���������,�����        Akm_Servo.adc_OriBuf[x][0]
	ADC_RegularChannelConfig(ADC2,ServoAdj_Ch,2,ADC_SampleTime_480Cycles); //�ڶ��ɼ����Ͻǵ�λ������,����� Akm_Servo.adc_OriBuf[x][1]
	
	//������ת���꣬�Զ�����DMA����
    ADC_DMARequestAfterLastTransferCmd(ADC2,ENABLE);
	
	//ADC2����
    ADC_Cmd(ADC2, ENABLE);
	
	//DMA����
	ADCx_DMAConfig_Init();
	
	//ADC2 DMA��������ʹ��
    ADC_DMACmd(ADC2,ENABLE);
	
    //����ת��,��DMA������ݰ���
    ADC_SoftwareStartConv(ADC2);
}

//����DMA�ɼ��Ļ�������ƽ��ֵ
int get_DMA_SlideRes(void)
{
	int tmp = 0;
	for(u8 i=0;i<akm_servo_bufsize;i++)
	{
		tmp += adc_OriBuf[i][0];
	}
	tmp = ( tmp/(float)akm_servo_bufsize ) - 2048;
	return tmp;
}

//�������Ͻǵ�λ��������ƽ��ֵ
int get_DMA_ServoBias(void)
{
	int tmp = 0;
	for(u8 i=0;i<akm_servo_bufsize;i++)
	{
		tmp += adc_OriBuf[i][1];
	}
	tmp = ( tmp/(float)akm_servo_bufsize ) - 2048;
	return tmp;
}

//���������������ʼ��
void Akm_ServoParam_Init(AKM_SERVO_ADC* p)
{
	p->Max = 2000;
	p->Min = 1000;
	p->Mid = SERVO_INIT;
}

/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
�������ܣ�AD����
��ڲ�����ADC��ͨ��
����  ֵ��ADת�����
**************************************************************************/
u16 Get_ADC1(volatile u8 ch)
{
    //Sets the specified ADC rule group channel, one sequence, and sampling time
    //����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��

    //ADC1,ADCͨ��,����ʱ��Ϊ480����
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );
	
    //Enable the specified ADC1 software transformation startup function
    //ʹ��ָ����ADC1�����ת����������
    ADC_SoftwareStartConv(ADC1);
	
    //Wait for the conversion to finish
    //�ȴ�ת������
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	
    //Returns the result of the last ADC1 rule group conversion
    //�������һ��ADC1�������ת�����
    return ADC_GetConversionValue(ADC1);
}

/**************************************************************************
Function: Collect multiple ADC values to calculate the average function
Input   : ADC channels and collection times
Output  : AD conversion results
�������ܣ��ɼ����ADCֵ��ƽ��ֵ����
��ڲ�����ADCͨ���Ͳɼ�����
�� �� ֵ��ADת�����
**************************************************************************/
u16 Get_ADC1_Average(u8 chn, u8 times)
{
    u32 temp_val=0;
    u8 t;
    for(t=0; t<times; t++)
    {
        temp_val+=Get_ADC1(chn);
    }
    return temp_val/times;
}

/**************************************************************************
Function: Read the battery voltage
Input   : none
Output  : Battery voltage in V
�������ܣ���ȡ��ص�ѹ
��ڲ�������
����  ֵ����ص�ѹ����λ v
**************************************************************************/
float Get_battery_volt(void)
{
    float Volt;
    //The resistance partial voltage can be obtained by simple analysis according to the schematic diagram
    //�����ѹ���������ԭ��ͼ�򵥷������Եõ�
    Volt=Get_ADC1(Battery_Ch)*3.3*11.0/1.0/4096;
    return Volt;
}


