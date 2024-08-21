#include "usartx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
SEND_AutoCharge_DATA Send_AutoCharge_Data;
extern int Time_count;

/**************************************************************************
Function: Usartx3, Usartx1 and CAN send data task 
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú3¡¢´®¿Ú1¡¢CAN·¢ËÍÊı¾İÈÎÎñ
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void data_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			//The task is run at 20hz
			//´ËÈÎÎñÒÔ20HzµÄÆµÂÊÔËĞĞ
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));
			//Assign the data to be sent
			//¶ÔÒª½øĞĞ·¢ËÍµÄÊı¾İ½øĞĞ¸³Öµ
			data_transition(); 
			//To enable serial port 1 to send data;
			//¿ªÆô´®¿Ú1·¢ËÍÊı¾İ
			USART1_SEND(); 
			USART3_SEND();     //Serial port 3 (ROS) sends data  //´®¿Ú3(ROS)·¢ËÍÊı¾İ
			CAN_SEND();        //CAN send data //CAN·¢ËÍÊı¾İ		

		}
}
/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú·¢ËÍµÄÊı¾İ½øĞĞ¸³Öµ
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //Ö¡Í·
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //Ö¡Î²
	
//Forward kinematics solution, from the current speed of each wheel to calculate the current speed of the three axis
	//ÔË¶¯Ñ§Õı½â£¬´Ó¸÷³µÂÖµ±Ç°ËÙ¶ÈÇó³öÈıÖáµ±Ç°ËÙ¶È
	Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; //Ğ¡³µxÖáËÙ¶È
	Send_Data.Sensor_Str.Y_speed = 0;
	Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/Wheel_spacing)*1000;//Ğ¡³µzÖáËÙ¶
	
	//The acceleration of the triaxial acceleration //¼ÓËÙ¶È¼ÆÈıÖá¼ÓËÙ¶È
	Send_Data.Sensor_Str.Accelerometer.X_data= accel[1]; //The accelerometer Y-axis is converted to the ros coordinate X axis //¼ÓËÙ¶È¼ÆYÖá×ª»»µ½ROS×ø±êXÖá
	Send_Data.Sensor_Str.Accelerometer.Y_data=-accel[0]; //The accelerometer X-axis is converted to the ros coordinate y axis //¼ÓËÙ¶È¼ÆXÖá×ª»»µ½ROS×ø±êYÖá
	Send_Data.Sensor_Str.Accelerometer.Z_data= accel[2]; //The accelerometer Z-axis is converted to the ros coordinate Z axis //¼ÓËÙ¶È¼ÆZÖá×ª»»µ½ROS×ø±êZÖá
	
	//The Angle velocity of the triaxial velocity //½ÇËÙ¶È¼ÆÈıÖá½ÇËÙ¶È
	Send_Data.Sensor_Str.Gyroscope.X_data= gyro[1]; //The Y-axis is converted to the ros coordinate X axis //½ÇËÙ¶È¼ÆYÖá×ª»»µ½ROS×ø±êXÖá
	Send_Data.Sensor_Str.Gyroscope.Y_data=-gyro[0]; //The X-axis is converted to the ros coordinate y axis //½ÇËÙ¶È¼ÆXÖá×ª»»µ½ROS×ø±êYÖá
	if(Flag_Stop==0) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //Èç¹ûµç»ú¿ØÖÆÎ»Ê¹ÄÜ×´Ì¬£¬ÄÇÃ´Õı³£·¢ËÍZÖá½ÇËÙ¶È
		Send_Data.Sensor_Str.Gyroscope.Z_data=gyro[2];  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //Èç¹û»úÆ÷ÈËÊÇ¾²Ö¹µÄ£¨µç»ú¿ØÖÆÎ»Ê§ÄÜ£©£¬ÄÇÃ´·¢ËÍµÄZÖá½ÇËÙ¶ÈÎª0		
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;  
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//µç³ØµçÑ¹(ÕâÀï½«¸¡µãÊı·Å´óÒ»Ç§±¶´«Êä£¬ÏàÓ¦µÄÔÚ½ÓÊÕ¶ËÔÚ½ÓÊÕµ½Êı¾İºóÒ²»áËõĞ¡Ò»Ç§±¶)
	Send_Data.Sensor_Str.Power_Voltage = Voltage*1000; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //Ö¡Í·
  Send_Data.buffer[1]=Flag_Stop; //Car software loss marker //Ğ¡³µÈí¼şÊ§ÄÜ±êÖ¾Î»
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//Ğ¡³µÈıÖáËÙ¶È,¸÷Öá¶¼²ğ·ÖÎªÁ½¸ö8Î»Êı¾İÔÙ·¢ËÍ
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed >>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed ;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed >>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed ;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU¼ÓËÙ¶È¼ÆÈıÖá¼ÓËÙ¶È,¸÷Öá¶¼²ğ·ÖÎªÁ½¸ö8Î»Êı¾İÔÙ·¢ËÍ
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU½ÇËÙ¶È¼ÆÈıÖá½ÇËÙ¶È,¸÷Öá¶¼²ğ·ÖÎªÁ½¸ö8Î»Êı¾İÔÙ·¢ËÍ
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//µç³ØµçÑ¹,²ğ·ÖÎªÁ½¸ö8Î»Êı¾İ·¢ËÍ
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //Êı¾İĞ£ÑéÎ»¼ÆËã£¬Ä£Ê½1ÊÇ·¢ËÍÊı¾İĞ£Ñé
	Send_Data.buffer[22]=Check_Sum(22,1); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //Ö¡Î²
	
	///////////////////////×Ô¶¯»Ø³äÏà¹Ø±äÁ¿¸³Öµ/////////////////////
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Header = AutoCharge_HEADER;   //Ö¡Í·¸³Öµ0x7C
	Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail = AutoCharge_TAIL;		//Ö¡Î²¸³Öµ0x7F
	Send_AutoCharge_Data.AutoCharge_Str.Charging_Current = (short)Charging_Current;//³äµçµçÁ÷¸³Öµ
	

	Send_AutoCharge_Data.AutoCharge_Str.RED = RED_STATE; //ºìÍâ±êÖ¾Î»¸³Öµ
	Send_AutoCharge_Data.AutoCharge_Str.Charging = Charging; //ÊÇ·ñÔÚ³äµç±êÖ¾Î»¸³Öµ

	Send_AutoCharge_Data.buffer[0] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Header;		//Ö¡Í·0x7C
	Send_AutoCharge_Data.buffer[1] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current>>8;//³äµçµçÁ÷¸ß8Î»
	Send_AutoCharge_Data.buffer[2] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current;	//³äµçµçÁ÷µÍ8Î»
	Send_AutoCharge_Data.buffer[3] = Send_AutoCharge_Data.AutoCharge_Str.RED;				//ÊÇ·ñ½ÓÊÕµ½ºìÍâ±êÖ¾Î»
	Send_AutoCharge_Data.buffer[4] = Send_AutoCharge_Data.AutoCharge_Str.Charging;			//ÊÇ·ñÔÚ³äµç±êÖ¾Î»
	Send_AutoCharge_Data.buffer[5] = Allow_Recharge;										//ÊÇ·ñ¿ªÆô×Ô¶¯»Ø³ä
	Send_AutoCharge_Data.buffer[6] = Check_Sum_AutoCharge(6,1);								//Ğ£ÑéÎ»
	Send_AutoCharge_Data.buffer[7] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail;		//Ö¡Î²0x7F
	///////////////////////×Ô¶¯»Ø³äÏà¹Ø±äÁ¿¸³Öµ/////////////////////
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú1·¢ËÍÊı¾İ
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void USART1_SEND(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		usart1_send(Send_Data.buffer[i]);
	}	
	if(Get_Charging_HardWare==1)
	{
		//´æÔÚ»Ø³ä×°±¸Ê±£¬ÏòÉÏ²ã·¢ËÍ×Ô¶¯»Ø³äÏà¹Ø±äÁ¿
		for(i=0; i<8; i++)
		{
			usart1_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}		
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú3·¢ËÍÊı¾İ
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void USART3_SEND(void)
{
  unsigned char i = 0;	
	for(i=0; i<24; i++)
	{
		usart3_send(Send_Data.buffer[i]);
	}	 
	if(Get_Charging_HardWare==1)
	{
		//´æÔÚ»Ø³ä×°±¸Ê±£¬ÏòÉÏ²ã·¢ËÍ×Ô¶¯»Ø³äÏà¹Ø±äÁ¿
		for(i=0; i<8; i++)
		{
			usart3_send(Send_AutoCharge_Data.buffer[i]);
		}	
	}
}
/**************************************************************************
Function: CAN sends data
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£ºCAN·¢ËÍÊı¾İ
Èë¿Ú²ÎÊı£ºÎŞ
·µ »Ø Öµ£ºÎŞ
**************************************************************************/
void CAN_SEND(void) 
{
	u8 CAN_SENT[8],i;
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i];
	}
	CAN1_Send_Num(0x101,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+8];
	}
	CAN1_Send_Num(0x102,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+16];
	}
	CAN1_Send_Num(0x103,CAN_SENT);
	
	//////////////////×Ô¶¯»Ø³äÏà¹ØÊı¾İ·¢ËÍ//////////////////
	if(Get_Charging_HardWare) CAN_Send_AutoRecharge();
}
/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú1³õÊ¼»¯
Èë¿Ú²ÎÊı£ºÎŞ
·µ »Ø Öµ£ºÎŞ
**************************************************************************/
void uart1_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock //Ê¹ÄÜGPIOÊ±ÖÓ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //Ê¹ÄÜUSARTÊ±ÖÓ

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //Êä³öÄ£Ê½
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //¸ßËÙ50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //ÉÏÀ­
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //³õÊ¼»¯
	
  //UsartNVIC configuration //UsartNVICÅäÖÃ
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //ÇÀÕ¼ÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //×ÓÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQÍ¨µÀÊ¹ÄÜ
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//¸ù¾İÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯VIC¼Ä´æÆ÷	
	NVIC_Init(&NVIC_InitStructure);	
	
  //USART Initialization Settings ³õÊ¼»¯ÉèÖÃ
	USART_InitStructure.USART_BaudRate = bound; //Port rate //´®¿Ú²¨ÌØÂÊ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //×Ö³¤Îª8Î»Êı¾İ¸ñÊ½
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //Ò»¸öÍ£Ö¹Î»
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //ÎŞÆæÅ¼Ğ£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //ÎŞÓ²¼şÊı¾İÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //ÊÕ·¢Ä£Ê½
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //³õÊ¼»¯´®¿Ú1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //¿ªÆô´®¿Ú½ÓÊÜÖĞ¶Ï
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //Ê¹ÄÜ´®¿Ú1
}
/**************************************************************************
Function: Serial port 4 initialization
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú4³õÊ¼»¯
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void uart4_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //Ê¹ÄÜGPIOÊ±ÖÓ
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //Enable the Usart clock //Ê¹ÄÜUSARTÊ±ÖÓ
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11 ,GPIO_AF_UART4);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //Êä³öÄ£Ê½
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //¸ßËÙ50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //ÉÏÀ­
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //³õÊ¼»¯
	
	//UsartNVIC configuration //UsartNVICÅäÖÃ
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	//Preempt priority //ÇÀÕ¼ÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //×ÓÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
  //Enable the IRQ channel //IRQÍ¨µÀÊ¹ÄÜ	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters 
	//¸ù¾İÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯VIC¼Ä´æÆ÷		
	NVIC_Init(&NVIC_InitStructure);	
	
	//USART Initialization Settings ³õÊ¼»¯ÉèÖÃ
	USART_InitStructure.USART_BaudRate = bound; //Port rate //´®¿Ú²¨ÌØÂÊ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //×Ö³¤Îª8Î»Êı¾İ¸ñÊ½
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //Ò»¸öÍ£Ö¹
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //ÎŞÆæÅ¼Ğ£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //ÎŞÓ²¼şÊı¾İÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //ÊÕ·¢Ä£Ê½
	USART_Init(UART4, &USART_InitStructure);      //Initialize serial port 2 //³õÊ¼»¯´®¿Ú2
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //¿ªÆô´®¿Ú½ÓÊÜÖĞ¶Ï
	USART_Cmd(UART4, ENABLE);                     //Enable serial port 2 //Ê¹ÄÜ´®¿Ú2 
}
/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú3³õÊ¼»¯
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void uart3_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	 //Enable the gpio clock  //Ê¹ÄÜGPIOÊ±ÖÓ
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock //Ê¹ÄÜUSARTÊ±ÖÓ
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //Êä³öÄ£Ê½
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //¸ßËÙ50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //ÉÏÀ­
	GPIO_Init(GPIOB, &GPIO_InitStructure);  		          //³õÊ¼»¯
	
  //UsartNVIC configuration //UsartNVICÅäÖÃ
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//Preempt priority //ÇÀÕ¼ÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //ÇÀÕ¼ÓÅÏÈ¼¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQÍ¨µÀÊ¹ÄÜ	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//¸ù¾İÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯VIC¼Ä´æÆ÷		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings ³õÊ¼»¯ÉèÖÃ
	USART_InitStructure.USART_BaudRate = bound; //Port rate //´®¿Ú²¨ÌØÂÊ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //×Ö³¤Îª8Î»Êı¾İ¸ñÊ½
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //Ò»¸öÍ£Ö¹
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //ÎŞÆæÅ¼Ğ£ÑéÎ»
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //ÎŞÓ²¼şÊı¾İÁ÷¿ØÖÆ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //ÊÕ·¢Ä£Ê½
  USART_Init(USART3, &USART_InitStructure);      //Initialize serial port 3 //³õÊ¼»¯´®¿Ú3
	
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //¿ªÆô´®¿Ú½ÓÊÜÖĞ¶Ï
  USART_Cmd(USART3, ENABLE);                     //Enable serial port 3 //Ê¹ÄÜ´®¿Ú3 
}
/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú1½ÓÊÕÖĞ¶Ï
Èë¿Ú²ÎÊı£ºÎŞ
·µ »Ø Öµ£ºÎŞ
**************************************************************************/
int USART1_IRQHandler(void)
{	
	static u8 Count=0;
	u8 Usart_Receive;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //ÅĞ¶ÏÊÇ·ñ½ÓÊÕµ½Êı¾İ
	{
		Usart_Receive = USART_ReceiveData(USART1);//Read the data //¶ÁÈ¡Êı¾İ
		if(Time_count<CONTROL_DELAY)
			// Data is not processed until 25 seconds after startup
		  //¿ª»ú25ÃëÇ°²»´¦ÀíÊı¾İ
			return 0;	//Ç°ÆÚ²»½øÈëÖĞ¶Ï
		
		//Fill the array with serial data
		//´®¿ÚÊı¾İÌîÈëÊı×é
    Receive_Data.buffer[Count]=Usart_Receive;
		
		//Ensure that the first data in the array is FRAME_HEADER
		//È·±£Êı×éµÚÒ»¸öÊı¾İÎªFRAME_HEADER
		if(Usart_Receive == FRAME_HEADER||Count>0) 
			Count++; 
		else 
			Count=0;
		
		if (Count == 11) //Verify the length of the packet //ÑéÖ¤Êı¾İ°üµÄ³¤¶È
		{   
				Count=0; //Prepare for the serial port data to be refill into the array //Îª´®¿ÚÊı¾İÖØĞÂÌîÈëÊı×é×ö×¼±¸
				if(Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet //ÑéÖ¤Êı¾İ°üµÄÖ¡Î²
				{
					//Data exclusionary or bit check calculation, mode 0 is sent data check
					//Êı¾İÒì»òÎ»Ğ£Ñé¼ÆËã£¬Ä£Ê½0ÊÇ·¢ËÍÊı¾İĞ£Ñé
					if(Receive_Data.buffer[9] ==Check_Sum(9,0))
				  {		
						float Vz;
						//Serial port 1 controls flag position 1, other flag position 0
						//´®¿Ú1¿ØÖÆ±êÖ¾Î»ÖÃ1£¬ÆäËü±êÖ¾Î»ÖÃ0
						PS2_ON_Flag=0;
						Remote_ON_Flag=0;
						APP_ON_Flag=0;
						CAN_ON_Flag=0;
						Usart_ON_Flag=1;
						command_lost_count=0;//´®¿Ú¡¢CAN¿ØÖÆÃüÁî¶ªÊ§¼ÆÊıÇåÁã
					  
						//Calculate the target speed of three axis from serial data, unit m/s
						//´Ó´®¿ÚÊı¾İÇóÈıÖáÄ¿±êËÙ¶È£¬ µ¥Î»m/s
						Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
						Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
						Vz    =XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
						if(Akm_Car)
						{
							Move_Z=Vz_to_Akm_Angle(Move_X, Vz);
						}
						else
						{
							Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
						}	
				  }
			}
		}
	} 
  return 0;
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú4½ÓÊÕÖĞ¶Ï
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
int UART4_IRQHandler(void)
{	
	int Usart_Receive;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //Check if data is received //ÅĞ¶ÏÊÇ·ñ½ÓÊÕµ½Êı¾İ
	{	      
		static u8 Flag_PID,i,j,Receive[50],Last_Usart_Receive;
		static float Data;
				
		Usart_Receive=UART4->DR; //Read the data //¶ÁÈ¡Êı¾İ
		
		if( AT_Command_Capture(Usart_Receive) ) return 0;
		_System_Reset_(Usart_Receive);
		
		if(Deviation_Count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //¿ª»ú10ÃëÇ°²»´¦ÀíÊı¾İ
		  return 0;	

		if(Usart_Receive==0x41&&Last_Usart_Receive==0x41&&APP_ON_Flag==0)
			//10 seconds after startup, press the forward button of APP to enter APP control mode
		  //The APP controls the flag position 1 and the other flag position 0
			//¿ª»ú10ÃëÖ®ºó£¬°´ÏÂAPPµÄÇ°½ø¼ü½øÈëAPP¿ØÖÆÄ£Ê½
		  //APP¿ØÖÆ±êÖ¾Î»ÖÃ1£¬ÆäËü±êÖ¾Î»ÖÃ0
			PS2_ON_Flag=0,Remote_ON_Flag=0,APP_ON_Flag=1,CAN_ON_Flag=0,Usart_ON_Flag=0;
    Last_Usart_Receive=Usart_Receive;			
	  
		if(Usart_Receive==0x4B) 
			//Enter the APP steering control interface
		  //½øÈëAPP×ªÏò¿ØÖÆ½çÃæ
			Turn_Flag=1;  
	  else	if(Usart_Receive==0x49||Usart_Receive==0x4A) 
      // Enter the APP direction control interface		
			//½øÈëAPP·½Ïò¿ØÖÆ½çÃæ	
			Turn_Flag=0;	
		
		if(Turn_Flag==0) 
		{
			//App rocker control interface command
			//APPÒ¡¸Ë¿ØÖÆ½çÃæÃüÁî
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				Flag_Direction=Usart_Receive-0x40;
			}
			else	if(Usart_Receive<=8)   
			{			
				Flag_Direction=Usart_Receive;
			}	
			else  Flag_Direction=0;
		}
		else if(Turn_Flag==1)
		{
			//APP steering control interface command
			//APP×ªÏò¿ØÖÆ½çÃæÃüÁî
			if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //Right rotation //ÓÒ×Ô×ª
			else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //Left rotation  //×ó×Ô×ª
			else                         Flag_Left=0,Flag_Right=0;
			if     (Usart_Receive==0x41||Usart_Receive==0x45) Flag_Direction=Usart_Receive-0x40;
			else  Flag_Direction=0;
		}
		if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s //¼ÓËÙ°´¼ü£¬+100mm/s
		if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons,   -100mm/s //¼õËÙ°´¼ü£¬-100mm/s
	  
	 // The following is the communication with the APP debugging interface
	 //ÒÔÏÂÊÇÓëAPPµ÷ÊÔ½çÃæÍ¨Ñ¶
	 if(Usart_Receive==0x7B) Flag_PID=1;   //The start bit of the APP parameter instruction //APP²ÎÊıÖ¸ÁîÆğÊ¼Î»
	 if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stops the bit    //APP²ÎÊıÖ¸ÁîÍ£Ö¹Î»
	
	if( Usart_Receive=='b' ) Allow_Recharge = !Allow_Recharge;
		
	 if(Flag_PID==1) //Collect data //²É¼¯Êı¾İ
	 {
		Receive[i]=Usart_Receive;
		i++;
	 }
	 if(Flag_PID==2) //Analyze the data //·ÖÎöÊı¾İ
	 {
			if(Receive[3]==0x50) 	 PID_Send=1;
			else  if(Receive[1]!=0x23) 
      {								
				for(j=i;j>=4;j--)
				{
					Data+=(Receive[j-1]-48)*pow(10,i-j);
				}
				switch(Receive[1])
				 {
					 case 0x30:  RC_Velocity=Data;break;
					 case 0x31:  Velocity_KP=Data;break;
					 case 0x32:  Velocity_KI=Data;break;
					 case 0x33:  Velocity_Smoother_Rate=Data/1000;break;
					 case 0x34:  Angle_Smoother_Rate=Data;break;
					 case 0x35:  Servo_max=Data;break;
					 case 0x36:  Servo_min=Data;break;
					 case 0x37:  break;
					 case 0x38:  break; 	
				 }
      }		
      //Relevant flag position is cleared			
      //Ïà¹Ø±êÖ¾Î»ÇåÁã			
			Flag_PID=0;
			i=0;
			j=0;
			Data=0;
			memset(Receive, 0, sizeof(u8)*50); //Clear the array to zero//Êı×éÇåÁã
	 }
   if(RC_Velocity<0)   RC_Velocity=0; 	 
  }
  return 0;	
}
/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú3½ÓÊÕÖĞ¶Ï
Èë¿Ú²ÎÊı£ºÎŞ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
int USART3_IRQHandler(void)
{	
	static u8 Count=0;
	u8 Usart_Receive;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //Check if data is received //ÅĞ¶ÏÊÇ·ñ½ÓÊÕµ½Êı¾İ
	{
		Usart_Receive = USART_ReceiveData(USART3);//Read the data //¶ÁÈ¡Êı¾İ
		if(Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //¿ª»ú10ÃëÇ°²»´¦ÀíÊı¾İ
		  return 0;	
		
		//Fill the array with serial data
		//´®¿ÚÊı¾İÌîÈëÊı×é
    Receive_Data.buffer[Count]=Usart_Receive;
		
		// Ensure that the first data in the array is FRAME_HEADER
		//È·±£Êı×éµÚÒ»¸öÊı¾İÎªFRAME_HEADER
		if(Usart_Receive == FRAME_HEADER||Count>0) 
			Count++; 
		else 
			Count=0;
		
		if (Count == 11) //Verify the length of the packet //ÑéÖ¤Êı¾İ°üµÄ³¤¶È
		{   
				Count=0; //Prepare for the serial port data to be refill into the array //Îª´®¿ÚÊı¾İÖØĞÂÌîÈëÊı×é×ö×¼±¸
				if(Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet //ÑéÖ¤Êı¾İ°üµÄÖ¡Î²
				{
					//Data exclusionary or bit check calculation, mode 0 is sent data check
					//Êı¾İÒì»òÎ»Ğ£Ñé¼ÆËã£¬Ä£Ê½0ÊÇ·¢ËÍÊı¾İĞ£Ñé
					if(Receive_Data.buffer[9] ==Check_Sum(9,0))	 
				   {		
					  command_lost_count=0; //CAN/´®¿Ú¿ØÖÆÃüÁî¶ªÊ§¼ÆÊıÇåÁã
						if(Receive_Data.buffer[1]==0)
						{
							Allow_Recharge=0; //¹Ø±Õ×Ô¶¯»Ø³ä£¬½ÓÊÕÕı³£Êı¾İ
							
							//All modes flag position 0, USART3 control mode
							//ËùÓĞÄ£Ê½±êÖ¾Î»ÖÃ0£¬ÎªUsart3¿ØÖÆÄ£Ê½						
							PS2_ON_Flag=0;
							Remote_ON_Flag=0;
							APP_ON_Flag=0;
							CAN_ON_Flag=0;
							Usart_ON_Flag=0;
											
							//Calculate the target speed of three axis from serial data, unit m/s
							//´Ó´®¿ÚÊı¾İÇóÈıÖáÄ¿±êËÙ¶È£¬ µ¥Î»m/s
							Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
							Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
							Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);		
						}
						//Add AutoRecharger relevant, 2022.01.18
						//Ìí¼Ó×Ô¶¯»Ø³äÏà¹Ø£¬2022.01.18
						else if( Receive_Data.buffer[1]==1 || Receive_Data.buffer[1]==2 )
						{					
							Allow_Recharge=1; //¿ªÆô×Ô¶¯»Ø³ä
							if(Receive_Data.buffer[1]==1 && RED_STATE==0) nav_walk=1; //¿ªÆô×Ô¶¯»Ø³ä£¬ÉÏÎ»»ú¿ØÖÆ»úÆ÷ÈË
							
							//Calculate the target speed of three axis from serial data, unit m/s
							//´Ó´®¿ÚÊı¾İÇóÈıÖáÄ¿±êËÙ¶È£¬µ¥Î»m/s
							Recharge_UP_Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
							Recharge_UP_Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
							Recharge_UP_Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);	
						}
						else if( Receive_Data.buffer[1]==3 )
						{
							//Set the speed of the infrared interconnection, unit m/s
							//ÉèÖÃºìÍâ¶Ô½ÓµÄËÙ¶È´óĞ¡£¬µ¥Î»m/s
							Red_Docker_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
							Red_Docker_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
							Red_Docker_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);	
						}
				  }
			}
		}
	} 
  return 0;
}

/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
º¯Êı¹¦ÄÜ£º½«ÉÏÎ»»ú·¢¹ıÀ´Ä¿±êÇ°½øËÙ¶ÈVx¡¢Ä¿±ê½ÇËÙ¶ÈVz£¬×ª»»Îª°¢¿ËÂüĞ¡³µµÄÓÒÇ°ÂÖ×ª½Ç
Èë¿Ú²ÎÊı£ºÄ¿±êÇ°½øËÙ¶ÈVx¡¢Ä¿±ê½ÇËÙ¶ÈVz£¬µ¥Î»£ºm/s£¬rad/s
·µ»Ø  Öµ£º°¢¿ËÂüĞ¡³µµÄÓÒÇ°ÂÖ×ª½Ç£¬µ¥Î»£ºrad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz)
{
	float R, AngleR, Min_Turn_Radius;
	//float AngleL;
	
	//Ackermann car needs to set minimum turning radius
	//If the target speed requires a turn radius less than the minimum turn radius,
	//This will greatly improve the friction force of the car, which will seriously affect the control effect
	//°¢¿ËÂüĞ¡³µĞèÒªÉèÖÃ×îĞ¡×ªÍä°ë¾¶
	//Èç¹ûÄ¿±êËÙ¶ÈÒªÇóµÄ×ªÍä°ë¾¶Ğ¡ÓÚ×îĞ¡×ªÍä°ë¾¶£¬
	//»áµ¼ÖÂĞ¡³µÔË¶¯Ä¦²ÁÁ¦´ó´óÌá¸ß£¬ÑÏÖØÓ°Ïì¿ØÖÆĞ§¹û
	if(Car_Mode==0||Car_Mode==1||Car_Mode==7||Car_Mode==8)
	{
		Min_Turn_Radius=SENIOR_AKM_MIN_TURN_RADIUS;
	}
	else if(Car_Mode==2||Car_Mode==3)
	{
		Min_Turn_Radius=TOP_AKM_BS_MIN_TURN_RADIUS;
	}
	else if(Car_Mode==4||Car_Mode==5)
	{
		Min_Turn_Radius=TOP_AKM_DL_MIN_TURN_RADIUS;
	}
	
	if(Vz!=0 && Vx!=0)
	{
		//If the target speed requires a turn radius less than the minimum turn radius
		//Èç¹ûÄ¿±êËÙ¶ÈÒªÇóµÄ×ªÍä°ë¾¶Ğ¡ÓÚ×îĞ¡×ªÍä°ë¾¶
		if(float_abs(Vx/Vz)<=Min_Turn_Radius)
		{
			//Reduce the target angular velocity and increase the turning radius to the minimum turning radius in conjunction with the forward speed
			//½µµÍÄ¿±ê½ÇËÙ¶È£¬ÅäºÏÇ°½øËÙ¶È£¬Ìá¸ß×ªÍä°ë¾¶µ½×îĞ¡×ªÍä°ë¾¶
			if(Vz>0)
				Vz= float_abs(Vx)/(Min_Turn_Radius);
			else	
				Vz=-float_abs(Vx)/(Min_Turn_Radius);		
		}		
		R=Vx/Vz;
		//AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
		AngleR=atan(Axle_spacing/(R+0.5f*Wheel_spacing));
	}
	else
	{
		AngleR=0;
	}
	
	return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
º¯Êı¹¦ÄÜ£º½«ÉÏÎ»»ú·¢¹ıÀ´µÄ¸ß8Î»ºÍµÍ8Î»Êı¾İÕûºÏ³ÉÒ»¸öshortĞÍÊı¾İºó£¬ÔÙ×öµ¥Î»»¹Ô­»»Ëã
Èë¿Ú²ÎÊı£º¸ß8Î»£¬µÍ8Î»
·µ»Ø  Öµ£º»úÆ÷ÈËX/Y/ZÖáµÄÄ¿±êËÙ¶È
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//Êı¾İ×ª»»µÄÖĞ¼ä±äÁ¿
	short transition; 
	
	//½«¸ß8Î»ºÍµÍ8Î»ÕûºÏ³ÉÒ»¸ö16Î»µÄshortĞÍÊı¾İ
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //µ¥Î»×ª»», mm/s->m/s	
}

/**************************************************************************
Function: The current XYZ three axis velocity of the robot is calculated from the data of the encoder
Input   : Data from various encoders
Output  : none
º¯Êı¹¦ÄÜ£ºÍ¨¹ı±àÂëÆ÷µÄÊı¾İ¼ÆËã»úÆ÷ÈËµ±Ç°µÄXYZÈıÖáËÙ¶È
Èë¿Ú²ÎÊı£º¸÷Â·±àÂëÆ÷µÄÊı¾İ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
#if Mec
void Motion_analysis_transformation(float Encoder_A,float Encoder_B,float Encoder_C,float Encoder_D)
{
	Send_Data.Sensor_Str.X_speed = ((Encoder_A+Encoder_B+Encoder_C+Encoder_D)/4)*1000; 
	Send_Data.Sensor_Str.Y_speed = ((Encoder_A-Encoder_B+Encoder_C-Encoder_D)/4)*1000; 
	Send_Data.Sensor_Str.Z_speed = ((-Encoder_A-Encoder_B+Encoder_C+Encoder_D)/4/(Axle_spacing+Wheel_spacing))*1000;
}
#elif Omni
void Motion_analysis_transformation(float Encoder_A,float Encoder_B,float Encoder_C)
{
	Send_Data.Sensor_Str.X_speed = ((Encoder_C-Encoder_B)/2/X_PARAMETER)*1000;
	Send_Data.Sensor_Str.Y_speed = ((Encoder_A*2-Encoder_B-Encoder_C)/3)*1000; 
	Send_Data.Sensor_Str.Z_speed = ((Encoder_A+Encoder_B+Encoder_C)/3/Omni_turn_radiaus)*1000;
}
#endif
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú1·¢ËÍÊı¾İ
Èë¿Ú²ÎÊı£ºÒª·¢ËÍµÄÊı¾İ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú2·¢ËÍÊı¾İ
Èë¿Ú²ÎÊı£ºÒª·¢ËÍµÄÊı¾İ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void usart4_send(u8 data)
{
	UART4->DR = data;
	while((UART4->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
º¯Êı¹¦ÄÜ£º´®¿Ú3·¢ËÍÊı¾İ
Èë¿Ú²ÎÊı£ºÒª·¢ËÍµÄÊı¾İ
·µ»Ø  Öµ£ºÎŞ
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}
/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
º¯Êı¹¦ÄÜ£º¼ÆËãÒª·¢ËÍ/½ÓÊÕµÄÊı¾İĞ£Ñé½á¹û
Èë¿Ú²ÎÊı£ºCount_Number£ºĞ£ÑéµÄÇ°¼¸Î»Êı£»Mode£º0-¶Ô½ÓÊÕÊı¾İ½øĞĞĞ£Ñé£¬1-¶Ô·¢ËÍÊı¾İ½øĞĞĞ£Ñé
·µ»Ø  Öµ£ºĞ£Ñé½á¹û
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//¶ÔÒª·¢ËÍµÄÊı¾İ½øĞĞĞ£Ñé
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//¶Ô½ÓÊÕµ½µÄÊı¾İ½øĞĞĞ£Ñé
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}

//×Ô¶¯»Ø³ä·¢ËÍ×Ö½Ú×¨ÓÃĞ£Ñéº¯Êı
u8 Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//¶ÔÒª·¢ËÍµÄÊı¾İ½øĞĞĞ£Ñé
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_AutoCharge_Data.buffer[k];
	}
	
	return check_sum;	
}


//À¶ÑÀATÖ¸Áî×¥°ü£¬·ÀÖ¹Ö¸Áî¸ÉÈÅµ½»úÆ÷ÈËÕı³£µÄÀ¶ÑÀÍ¨ĞÅ
u8 AT_Command_Capture(u8 uart_recv)
{
	/*
	À¶ÑÀÁ´½ÓÊ±·¢ËÍµÄ×Ö·û£¬00:11:22:33:44:55ÎªÀ¶ÑÀµÄMACµØÖ·
	+CONNECTING<<00:11:22:33:44:55\r\n
	+CONNECTED\r\n
	¹²44¸ö×Ö·û
	
	À¶ÑÀ¶Ï¿ªÊ±·¢ËÍµÄ×Ö·û
	+DISC:SUCCESS\r\n
	+READY\r\n
	+PAIRABLE\r\n
	¹²34¸ö×Ö·û
	\r -> 0x0D
	\n -> 0x0A
	*/
	
	static u8 pointer = 0; //À¶ÑÀ½ÓÊÜÊ±Ö¸Õë¼ÇÂ¼Æ÷
	static u8 bt_line = 0; //±íÊ¾ÏÖÔÚÔÚµÚ¼¸ĞĞ
	static u8 disconnect = 0;
	static u8 connect = 0;
	
	//¶Ï¿ªÁ¬½Ó
	static char* BlueTooth_Disconnect[3]={"+DISC:SUCCESS\r\n","+READY\r\n","+PAIRABLE\r\n"};
	
	//¿ªÊ¼Á¬½Ó
	static char* BlueTooth_Connect[2]={"+CONNECTING<<00:00:00:00:00:00\r\n","+CONNECTED\r\n"};


	//ÌØÊâ±êÊ¶·û£¬¿ªÊ¼¾¯Ìè(Ê¹ÓÃÊ±Òª-1)
	if(uart_recv=='+') 
	{
		bt_line++,pointer=0; //ÊÕµ½¡®+¡¯£¬±íÊ¾ÇĞ»»ÁËĞĞÊı	
		disconnect++,connect++;
		return 1;//×¥°ü£¬½ûÖ¹¿ØÖÆ
	}

	if(bt_line!=0) 
	{	
		pointer++;

		//¿ªÊ¼×·×ÙÊı¾İÊÇ·ñ·ûºÏ¶Ï¿ªµÄÌØÕ÷£¬·ûºÏÊ±È«²¿ÆÁ±Î£¬²»·ûºÏÊ±È¡ÏûÆÁ±Î
		if(uart_recv == BlueTooth_Disconnect[bt_line-1][pointer])
		{
			disconnect++;
			if(disconnect==34) disconnect=0,connect=0,bt_line=0,pointer=0;
			return 1;//×¥°ü£¬½ûÖ¹¿ØÖÆ
		}			

		//×·×ÙÁ¬½ÓÌØÕ÷ (bt_line==1&&connect>=13)Çø¶ÎÊÇÀ¶ÑÀMACµØÖ·£¬Ã¿Ò»¸öÀ¶ÑÀMACµØÖ·¶¼²»ÏàÍ¬£¬ËùÒÔÖ±½ÓÆÁ±Î¹ıÈ¥
		else if(uart_recv == BlueTooth_Connect[bt_line-1][pointer] || (bt_line==1&&connect>=13) )
		{		
			connect++;
			if(connect==44) connect=0,disconnect=0,bt_line=0,pointer=0;		
			return 1;//×¥°ü£¬½ûÖ¹¿ØÖÆ
		}	

		//ÔÚ×¥°üÆÚ¼äÊÕµ½ÆäËûÃüÁî£¬Í£Ö¹×¥°ü
		else
		{
			disconnect = 0;
			connect = 0;
			bt_line = 0;		
			pointer = 0;
			return 0;//·Ç½ûÖ¹Êı¾İ£¬¿ÉÒÔ¿ØÖÆ
		}			
	}
	
	return 0;//·Ç½ûÖ¹Êı¾İ£¬¿ÉÒÔ¿ØÖÆ
}

//Èí¸´Î»½øBootLoaderÇøÓò
void _System_Reset_(u8 uart_recv)
{
	static u8 res_buf[5];
	static u8 res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//½ÓÊÜµ½ÉÏÎ»»úÇëÇóµÄ¸´Î»×Ö·û¡°reset¡±£¬Ö´ĞĞÈí¼ş¸´Î»
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			NVIC_SystemReset();//½øĞĞÈí¼ş¸´Î»£¬¸´Î»ºóÖ´ĞĞ BootLoader ³ÌĞò
		}
	}
}




