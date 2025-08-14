#include "can.h"
#include "system.h"

//CAN�ӿڶ�Ӧ�����ų�ʼ��
static void GPIO_CANPort_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	
	ENABLE_CAN1_TX_PIN_CLOCK;
	ENABLE_CAN1_RX_PIN_CLOCK;
	
	//��ʼ��GPIO
	//CAN1
	GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //����
	GPIO_Init(CAN1_TX_PORT, &GPIO_InitStructure);           
	
	GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //����
	GPIO_Init(CAN1_RX_PORT, &GPIO_InitStructure);    
	
	//���Ÿ���ӳ������
	GPIO_PinAFConfig(CAN1_TX_PORT,CAN1_TX_Soure,GPIO_AF_CAN1); 
	GPIO_PinAFConfig(CAN1_RX_PORT,CAN1_RX_Soure,GPIO_AF_CAN1);
}

//CAN�ӿڶ�Ӧ�����ų�ʼ��
static void V1_0_GPIO_CANPort_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	
	V1_0_ENABLE_CAN1_TX_PIN_CLOCK;
	V1_0_ENABLE_CAN1_RX_PIN_CLOCK;
	
	//��ʼ��GPIO
	//CAN1
	GPIO_InitStructure.GPIO_Pin = V1_0_CAN1_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //����
	GPIO_Init(V1_0_CAN1_TX_PORT, &GPIO_InitStructure);           
	
	GPIO_InitStructure.GPIO_Pin = V1_0_CAN1_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //����
	GPIO_Init(V1_0_CAN1_RX_PORT, &GPIO_InitStructure);    
	
	//���Ÿ���ӳ������
	GPIO_PinAFConfig(V1_0_CAN1_TX_PORT,V1_0_CAN1_TX_Soure,GPIO_AF_CAN1); 
	GPIO_PinAFConfig(V1_0_CAN1_RX_PORT,V1_0_CAN1_TX_Soure,GPIO_AF_CAN1);
}

/**************************************************************************
Function: CAN1 initialization
Input   : tsjw��Resynchronize the jump time unit, Scope: 1 ~ 3;
 			    tbs2��Time unit of time period 2, range :1~8;
 			    tbs1��Time unit of time period 1, range :1~16;
 			    brp ��Baud rate divider, range :1 to 1024;(We're actually going to add 1, which is 1 to 1024) tq=(brp)*tpclk1
 			    mode��0, normal mode;1. Loop mode;
Output  : 0- Initialization successful;Other - initialization failed
Note: none of the entry parameters (except mode) can be 0
�������ܣ�CAN1��ʼ��
��ڲ�����tsjw������ͬ����Ծʱ�䵥Ԫ����Χ:1~3;
 			    tbs2��ʱ���2��ʱ�䵥Ԫ����Χ:1~8;
 			    tbs1��ʱ���1��ʱ�䵥Ԫ����Χ:1~16;
 			    brp �������ʷ�Ƶ������Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
 			    mode��0,��ͨģʽ;1,�ػ�ģʽ;
����  ֵ��0-��ʼ���ɹ�; ����-��ʼ��ʧ��
ע�⣺��ڲ���(����mode)������Ϊ0
������/Baud rate=Fpclk1/((tbs1+tbs2+1)*brp)��Fpclk1Ϊ36M
                =42M/((3+2+1)*6)
						    =1M
**************************************************************************/
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	//-1����Ϊ��Щ����ʵ����ֵ��0��ʼ
	//brp����Ҫ-1����Ϊ��������ֵ�Ǵ�1��ʼ
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0) return 1;
	tsjw-=1; //Subtract 1 before setting //�ȼ�ȥ1.����������
	tbs2-=1;
	tbs1-=1;
	
	//CAN�ڶ�Ӧ��GPIO��ʼ��
	GPIO_CANPort_Init();
	
	//ʹ�����ʱ��
	ENABLE_CAN1_CLOCK;
	
	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
	CAN_InitStructure.CAN_AWUM=DISABLE; //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= mode;	//ģʽ���� 
	
	//�������������
	CAN_InitStructure.CAN_SJW=tsjw;	    //����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1;     //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;     //Tbs2��ΧCAN_BS2_1tq ~CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;//��Ƶϵ��(Fdiv)Ϊbrp+1	
	
	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 	
	
	//���ù�����
	//CAN1
	CAN_FilterInitStructure.CAN_FilterNumber=0;	                   //������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  //����ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;               //32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000; 	

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
	CAN_FilterInit(&CAN_FilterInitStructure);            //�˲�����ʼ��
	
	//CAN1 FIFO0 �ж�ʹ��
	#if CAN1_RX0_INT_ENABLE
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;// �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	//CAN2 FIFO1 �ж�ʹ��
	#if CAN2_RX1_INT_ENABLE
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//FIFO1��Ϣ�Һ��ж�����.		    
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	return 0;
}   

//V1.00�汾CAN�ڳ�ʼ������
u8 V1_0_CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	//-1����Ϊ��Щ����ʵ����ֵ��0��ʼ
	//brp����Ҫ-1����Ϊ��������ֵ�Ǵ�1��ʼ
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0) return 1;
	tsjw-=1; //Subtract 1 before setting //�ȼ�ȥ1.����������
	tbs2-=1;
	tbs1-=1;
	
	//CAN�ڶ�Ӧ��GPIO��ʼ��
	V1_0_GPIO_CANPort_Init();
	
	//ʹ�����ʱ��
	ENABLE_CAN1_CLOCK;       
	
	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
	
	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
	CAN_InitStructure.CAN_AWUM=DISABLE; //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= mode;	//ģʽ���� 
	
	//�������������
	CAN_InitStructure.CAN_SJW=tsjw;	    //����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1;     //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;     //Tbs2��ΧCAN_BS2_1tq ~CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;//��Ƶϵ��(Fdiv)Ϊbrp+1	
	
	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 	
	
	//���ù�����
	//CAN1
	CAN_FilterInitStructure.CAN_FilterNumber=0;	                   //������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  //����ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;               //32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000; 	

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
	CAN_FilterInit(&CAN_FilterInitStructure);            //�˲�����ʼ��
	
	//CAN1 FIFO0 �ж�ʹ��
	#if CAN1_RX0_INT_ENABLE
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;// �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	//CAN2 FIFO1 �ж�ʹ��
	#if CAN2_RX1_INT_ENABLE
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//FIFO1��Ϣ�Һ��ж�����.		    
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	return 0;
}   

// CAN1���ͺ���
u8 CAN1_Send_Num(u32 id,u8 *data)
{
	//CAN�������ݽṹ��
	CanTxMsg msg;
	
	u16 i;   //��ʱʱ��
	u8 mbox; //
	
	#if 1 //���ñ�׼֡��id
	msg.StdId = id;
	msg.IDE = CAN_Id_Standard;
	#else //������չ֡��id
	msg.ExtId = id;
	msg.IDE = CAN_Id_Extended;
	#endif
	
	//���͵ı�����������֡
	//����֡ CAN_RTR_DATA
	//ң��֡ CAN_RTR_REMOTE
	msg.RTR = CAN_RTR_DATA;
	
	//���͵����ݳ���
	// 1~8
	msg.DLC = 8;
	
	//��Ҫ���͵����ݸ����� msg.Data
	memcpy(msg.Data,data,8);
	
	//CAN1 ����
	CAN_Transmit(CAN1,&msg);
	
	//�ȴ�CAN1�������
	while( CAN_TransmitStatus(CAN1,mbox)== CAN_TxStatus_Failed && i<0xffff ) i++;
	
	//���ͳ�ʱ
	if(i>=0xffff) return 1;
	
	return 0;
}

u8 CAN1_Send_EXTid_Num(u32 id,u8 *data)
{
	//CAN�������ݽṹ��
	CanTxMsg msg;
	
	u16 i;   //��ʱʱ��
	u8 mbox; //
	
	#if 0 //���ñ�׼֡��id
	msg.StdId = id;
	msg.IDE = CAN_Id_Standard;
	#else //������չ֡��id
	msg.ExtId = id;
	msg.IDE = CAN_Id_Extended;
	#endif
	
	//���͵ı�����������֡
	//����֡ CAN_RTR_DATA
	//ң��֡ CAN_RTR_REMOTE
	msg.RTR = CAN_RTR_DATA;
	
	//���͵����ݳ���
	// 1~8
	msg.DLC = 8;
	
	//��Ҫ���͵����ݸ����� msg.Data
	memcpy(msg.Data,data,8);
	
	//CAN1 ����
	CAN_Transmit(CAN1,&msg);
	
	//�ȴ�CAN1�������
	while( CAN_TransmitStatus(CAN1,mbox)== CAN_TxStatus_Failed && i<0xffff ) i++;
	
	//���ͳ�ʱ
	if(i>=0xffff) return 1;
	
	return 0;
}
