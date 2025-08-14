#include "timx_callback.h"

/**************************************************************************
Function: Model aircraft remote control receiving interrupt
Input   : none
Output  : none
�������ܣ���ģң�ؽ����жϣ���ʱ��X�����жϣ�
��ڲ�������
�� �� ֵ����
**************************************************************************/
void REMOTE_TIM_IRQHandler(void)
{
	//Input the capture flag for channel,
	//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
	//ͨ�����벶���־������λ�������־����6λ�������־
	static u8 RemoteCH1_CAPTURE_STA = 0;
	static u8 RemoteCH2_CAPTURE_STA = 0;
	static u8 RemoteCH3_CAPTURE_STA = 0;
	static u8 RemoteCH4_CAPTURE_STA = 0;

	static u16 RemoteCH1_CAPTURE_UPVAL;
	static u16 RemoteCH1_CAPTURE_DOWNVAL;

	static u16 RemoteCH2_CAPTURE_UPVAL;
	static u16 RemoteCH2_CAPTURE_DOWNVAL;

	static u16 RemoteCH3_CAPTURE_UPVAL;
	static u16 RemoteCH3_CAPTURE_DOWNVAL;

	static u16 RemoteCH4_CAPTURE_UPVAL;
	static u16 RemoteCH4_CAPTURE_DOWNVAL;
	
	//ң����ֵ����ʷ��¼
	static REMOTER_t last_remoter;
	
	//ƫ��ֵ,�������㲶����ֵ
	u32 CH1_Diff;
	u32 CH2_Diff;
	u32 CH3_Diff;
	u32 CH4_Diff;

	//ͨ��ͻ���˲�
    static u8 ch1_filter_times=0,ch2_filter_times=0,ch3_filter_times=0,ch4_filter_times=0;

    //���Ӻ�ģңң��������Ҫ����ǰ���ˣ��ſ�����ʽ��ģ����С��
    //After connecting the remote controller of the model aircraft,
    //you need to push down the forward lever to officially control the car of the model aircraft
	static uint16_t state_filter=0;//״̬�˲�,��ֹ��ģ����ģʽ����
    if(remoter.ch2>1600&& remoter.check_count< Filter_Threshold &&Get_Control_Mode(_RC_Control)==0&&SysVal.Time_count>=CONTROL_DELAY)
    {
		state_filter++;
		if( state_filter > 250 ) //���㿪����ģң�س���һ����ʱ���ٿ�����ģ����,��ֹ���ָ����ź�.250û�о��嵥λ,�����붨ʱ���ж�Ƶ���й�.
		{
			Set_Control_Mode(_RC_Control);//����Ϊ��ģң��ģʽ 
		}     
    }
	else
	{
		state_filter = 0;//�Ǻ�ģң�ص��ж�����,��λ�˲�ֵ
	}
	
    //Channel 1 //ͨ��һ
    if ((RemoteCH1_CAPTURE_STA & 0X80) == 0)
    {
        if ( Get_CH1_State != RESET) //A capture event occurred on channel 1 //ͨ��1���������¼�
        {
            Clear_CH1_State; //Clear the interrupt flag bit //����жϱ�־λ
            if (RemoteCH1_CAPTURE_STA & 0X40)	//A falling edge is caught //����һ���½���
            {
                RemoteCH1_CAPTURE_DOWNVAL = Get_CH1_CNT ; //Record the timer value at this point //��¼�´�ʱ�Ķ�ʱ������ֵ
                if (RemoteCH1_CAPTURE_DOWNVAL < RemoteCH1_CAPTURE_UPVAL)
                {
                    CH1_Diff = 9999;
                }
                else
                    CH1_Diff = 0;
                remoter.ch1 = RemoteCH1_CAPTURE_DOWNVAL - RemoteCH1_CAPTURE_UPVAL + CH1_Diff;	//Time to get the total high level //�õ��ܵĸߵ�ƽ��ʱ��
				
                if(abs(remoter.ch1-last_remoter.ch1)>500)
                {
                    ch1_filter_times++;
                    if( ch1_filter_times<=5 ) remoter.ch1=last_remoter.ch1; //Filter //�˲�
                    else ch1_filter_times=0;
                }
                else
                {
                    ch1_filter_times=0;
                }
                last_remoter.ch1 = remoter.ch1;

                RemoteCH1_CAPTURE_STA = 0; //Capture flag bit to zero	//�����־λ����
				Set_CH1_Rising; //Set to rising edge capture //����Ϊ�����ز���
            }
            else
            {
                //When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
                //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
                RemoteCH1_CAPTURE_UPVAL = Get_CH1_CNT; //Obtain rising edge data //��ȡ����������
                RemoteCH1_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //����Ѳ���������
                Set_CH1_Falling; //Set to Falling Edge Capture //����Ϊ�½��ز���
            }
        }
    }
    //Channel 2 //ͨ����
    if ((RemoteCH2_CAPTURE_STA & 0X80) == 0)
    {
        if ( Get_CH2_State != RESET)	//A capture event occurred on channel 2 //ͨ��2���������¼�
        {
            Clear_CH2_State; //Clear the interrupt flag bit //����жϱ�־λ
            if (RemoteCH2_CAPTURE_STA & 0X40)	//A falling edge is caught //����һ���½���
            {
                RemoteCH2_CAPTURE_DOWNVAL = Get_CH2_CNT; //Record the timer value at this point //��¼�´�ʱ�Ķ�ʱ������ֵ
                if (RemoteCH2_CAPTURE_DOWNVAL < RemoteCH2_CAPTURE_UPVAL)
                {
                    CH2_Diff = 9999;
                }
                else
                    CH2_Diff = 0;
                remoter.ch2 = RemoteCH2_CAPTURE_DOWNVAL - RemoteCH2_CAPTURE_UPVAL + CH2_Diff; //Time to get the total high level //�õ��ܵĸߵ�ƽ��ʱ��
                if(abs(remoter.ch2-last_remoter.ch2)>500)
                {
                    ch2_filter_times++;
                    if( ch2_filter_times<=5 ) remoter.ch2=last_remoter.ch2; //Filter //�˲�
                    else ch2_filter_times=0;
                }
                else
                {
                    ch2_filter_times=0;
                }
                last_remoter.ch2=remoter.ch2;

                RemoteCH2_CAPTURE_STA = 0; //Capture flag bit to zero	//�����־λ����
                Set_CH2_Rising; //Set to rising edge capture //����Ϊ�����ز���
            }
            else
            {
                //When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
                //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
                RemoteCH2_CAPTURE_UPVAL = Get_CH2_CNT; //Obtain rising edge data //��ȡ����������
                RemoteCH2_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //����Ѳ���������
                Set_CH2_Falling; //Set to Falling Edge Capture //����Ϊ�½��ز���
            }
        }
    }
    //Channel 3 //ͨ����
    if ((RemoteCH3_CAPTURE_STA & 0X80) == 0)
    {
        if (Get_CH3_State != RESET)	//A capture event occurred on channel 3 //ͨ��3���������¼�
        {
			remoter.check_count = 0;//��ģң�ؼ�鸴λ,�����λ����Ƶ����λ,˵���Ǹ����ź�.
			
            Clear_CH3_State; //Clear the interrupt flag bit //����жϱ�־λ
            if (RemoteCH3_CAPTURE_STA & 0X40)	//A falling edge is caught //����һ���½���
            {
                RemoteCH3_CAPTURE_DOWNVAL = Get_CH3_CNT; //Record the timer value at this point //��¼�´�ʱ�Ķ�ʱ������ֵ
                if (RemoteCH3_CAPTURE_DOWNVAL < RemoteCH3_CAPTURE_UPVAL)
                {
                    CH3_Diff = 9999;
                }
                else
                    CH3_Diff = 0;
                remoter.ch3 = RemoteCH3_CAPTURE_DOWNVAL - RemoteCH3_CAPTURE_UPVAL + CH3_Diff; //Time to get the total high level //�õ��ܵĸߵ�ƽ��ʱ��
                if(abs(remoter.ch3-last_remoter.ch3)>500)
                {
                    ch3_filter_times++;
                    if( ch3_filter_times<=5 ) remoter.ch3=last_remoter.ch3; //Filter //�˲�
                    else ch3_filter_times=0;
                }
                else
                {
                    ch3_filter_times=0;
                }
                last_remoter.ch3=remoter.ch3;
                RemoteCH3_CAPTURE_STA = 0; //Capture flag bit to zero	//�����־λ����
                Set_CH3_Rising; //Set to rising edge capture //����Ϊ�����ز���
            }
            else
            {
                //When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
                //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
                RemoteCH3_CAPTURE_UPVAL = Get_CH3_CNT; //Obtain rising edge data //��ȡ����������
                RemoteCH3_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //����Ѳ���������
                Set_CH3_Falling; //Set to Falling Edge Capture //����Ϊ�½��ز���
            }
        }
    }
	
    //Channel 4 //ͨ����
    if ((RemoteCH4_CAPTURE_STA & 0X80) == 0)
    {
        if (Get_CH4_State != RESET)	//A capture event occurred on channel 4 //ͨ��4���������¼�
        {
            Clear_CH4_State; //Clear the interrupt flag bit //����жϱ�־λ
            if (RemoteCH4_CAPTURE_STA & 0X40)	//A falling edge is caught //����һ���½���
            {
                RemoteCH4_CAPTURE_DOWNVAL = Get_CH4_CNT; //Record the timer value at this point //��¼�´�ʱ�Ķ�ʱ������ֵ
                if (RemoteCH4_CAPTURE_DOWNVAL < RemoteCH4_CAPTURE_UPVAL)
                {
                    CH4_Diff = 9999;
                }
                else
                    CH4_Diff = 0;
                remoter.ch4 = RemoteCH4_CAPTURE_DOWNVAL - RemoteCH4_CAPTURE_UPVAL + CH4_Diff; //Time to get the total high level //�õ��ܵĸߵ�ƽ��ʱ��
                if(abs(remoter.ch4-last_remoter.ch4)>500)
                {
                    ch4_filter_times++;
                    if( ch4_filter_times<=5 ) remoter.ch4=last_remoter.ch4; //Filter //�˲�
                    else ch4_filter_times=0;
                }
                else
                {
                    ch4_filter_times=0;
                }
                last_remoter.ch4=remoter.ch4;
                RemoteCH4_CAPTURE_STA = 0; //Capture flag bit to zero	//�����־λ����
                Set_CH4_Rising; //Set to rising edge capture //����Ϊ�����ز���
            }
            else
            {
                //When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
                //��������ʱ�䵫�����½��أ���һ�β��������أ���¼��ʱ�Ķ�ʱ������ֵ
                RemoteCH4_CAPTURE_UPVAL = Get_CH4_CNT; //Obtain rising edge data //��ȡ����������
                RemoteCH4_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //����Ѳ���������
                Set_CH4_Falling; //Set to Falling Edge Capture //����Ϊ�½��ز���
            }
        }
    }
	
}





