#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int Time_count;
u8 oled_refresh_flag;
u8 oled_page=0;
/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
函数功能：读取电池电压、蜂鸣器报警、开启自检、向APP发送数据、OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
int Buzzer_count=25;
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
   {	
		int i=0;
		static int LowVoltage_1=0, LowVoltage_2=0;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //此任务以10Hz的频率运行
		SERVO_BIAS=(Get_Adc(SERVO_BALANCE_ADC)-2048)/5;
		//开机时蜂鸣器短暂蜂鸣，开机提醒
		//The buzzer will beep briefly when the machine is switched on
		if(Time_count<50)Buzzer=1; 
		else if(Time_count>=51 && Time_count<100)Buzzer=0;
		 
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
		if(Buzzer_count<5)Buzzer_count++;
		if(Buzzer_count<5)Buzzer=1; //The buzzer is buzzing //蜂鸣器蜂鸣
		else if(Buzzer_count==5)Buzzer=0;
		
		//Read the battery voltage //读取电池电压
		for(i=0;i<100;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/100;
		Voltage = VolMean_Filter(Voltage);
		Voltage_All=0;
		
		if( Get_Charging_HardWare==1 )
		{	//电压过低，收到红外信号，开启自动回充功能
			if(Voltage<20&&RED_STATE) Allow_Recharge=1;
		}
		
		if(LowVoltage_1==1)LowVoltage_1++; //Make sure the buzzer only rings for 0.5 seconds //确保蜂鸣器只响0.5秒
		if(LowVoltage_2==1)LowVoltage_2++; //Make sure the buzzer only rings for 0.5 seconds //确保蜂鸣器只响0.5秒					
		APP_Show();	 //Send data to the APP //向APP发送数据
		if(oled_refresh_flag) OLED_Clear(),oled_refresh_flag=0;
		else oled_show(); //Tasks are displayed on the screen //显示屏显示任务
   }
}  

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
函数功能：OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
	int Car_Mode_Show;
	
	if(oled_page==0)
	{
		//Collect the tap information of the potentiometer, 
		//and display the car model to be fitted when the car starts up in real time
		//采集电位器档位信息，实时显示小车开机时要适配的小车型号
		Car_Mode_Show=(int) (Get_adc_Average(CAR_MODE_ADC,10)/Divisor_Mode);
	//	if(Car_Mode_Show>7)Car_Mode_Show=7;
		Voltage_Show=Voltage*100;
		
		 //The first line of the display displays the content//
		 //显示屏第1行显示内容//	
		if(Allow_Recharge) OLED_ShowString(0,0,"RCM :"); 
		else OLED_ShowString(0,0,"TYPE:");    
		 if      (robot_mode_check_flag==0)  OLED_ShowNumber(40,0,Car_Mode_Show,1,12); //Display robot type //显示机器人类型
		 else if (robot_mode_check_flag==1)  OLED_ShowString(40,0,"X");                //Type mismatch displays "X" //类型不适配时显示”X“
			
		 // 显示充电电流
		OLED_ShowString(80,0," ");
		OLED_ShowString(85,0,"   ");
		OLED_ShowString(60,0,"Cur:");
		oled_showfloat(Charging_Current/1000.0f,90,0,1,2);

		 //The second line of the display displays the content//
		 //显示屏第2行显示内容//	
		 //Display Z-axis angular velocity //显示Z轴角速度
		 OLED_ShowString(00,10,"GYRO_Z"); 
		 if( gyro[2]<0)   OLED_ShowString(65,10,"-"),
											OLED_ShowNumber(75,10,-gyro[2],5,12);
		 else             OLED_ShowString(65,10,"+"),
											OLED_ShowNumber(75,10, gyro[2],5,12);		

		//The third line of the display displays the content//
		//显示屏第3行显示内容//
		//Display the target speed and current speed of motor A
		//显示电机A的目标速度和当前速度	 
	  OLED_ShowString(0,20,"DL:");
	  if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
													OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
	  else                 	OLED_ShowString(15,20,"+"),
													OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12); 
		
	  if( MOTOR_A.Encoder<0)OLED_ShowString(60,20,"-"),
													OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
	  else                 	OLED_ShowString(60,20,"+"),
													OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);
			 
		//The fourth line of the display displays the content//
		//显示屏第4行显示内容//	
		//Display the target speed and current speed of motor B
		//显示电机B的目标速度和当前速度
	  OLED_ShowString(0,30,"DR:");
	  if( MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
													OLED_ShowNumber(20,30,- MOTOR_B.Target*1000,5,12);
	  else                 	OLED_ShowString(15,30,"+"),
													OLED_ShowNumber(20,30,  MOTOR_B.Target*1000,5,12); 
			
	  if( MOTOR_B.Encoder<0)OLED_ShowString(60,30,"-"),
													OLED_ShowNumber(75,30,-MOTOR_B.Encoder*1000,5,12);
	  else                 	OLED_ShowString(60,30,"+"),
													OLED_ShowNumber(75,30, MOTOR_B.Encoder*1000,5,12);

		//Line 5 of the display displays the content//
		//显示屏第5行显示内容//
		//Display the current PWM value of A and B motors
		//显示A、B电机当前的PWM值
		if(BrushlessEnable==0)
		{
															OLED_ShowString(00,40,"MA");
			if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(20,40,"-"),
															OLED_ShowNumber(30,40,-MOTOR_A.Motor_Pwm,4,12);
			else                 	  OLED_ShowString(20,40,"+"),
															OLED_ShowNumber(30,40, MOTOR_A.Motor_Pwm,4,12); 
															OLED_ShowString(60,40,"MB");
			if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(80,40,"-"),
															OLED_ShowNumber(90,40,-MOTOR_B.Motor_Pwm,4,12);
			else                 	  OLED_ShowString(80,40,"+"),
															OLED_ShowNumber(90,40, MOTOR_B.Motor_Pwm,4,12);
	  }
		if(BrushlessEnable==1)
		{
															  OLED_ShowString(00,40,"RPM");
			if( MOTOR_A.Encoder_Rpm<0)OLED_ShowString(30,40,"-"),
															  OLED_ShowNumber(50,40,-MOTOR_A.Encoder_Rpm,4,12);
			else                 	    OLED_ShowString(30,40,"+"),
															  OLED_ShowNumber(50,40, MOTOR_A.Encoder_Rpm,4,12); 
															
			if(MOTOR_B.Encoder_Rpm<0) OLED_ShowString(80,40,"-"),
															  OLED_ShowNumber(100,40,-MOTOR_B.Encoder_Rpm,4,12);
			else                 	    OLED_ShowString(80,40,"+"),
															  OLED_ShowNumber(100,40, MOTOR_B.Encoder_Rpm,4,12);
	  }
		

		//显示屏第6行显示内容
		//Line 6 of the display displays the contents
		//Displays the current control mode //显示当前控制模式
		if(PS2_ON_Flag==1)         OLED_ShowString(0,50,"PS2  ");
		else if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP  ");
		else if (Remote_ON_Flag==1)OLED_ShowString(0,50,"R-C  ");
		else if (CAN_ON_Flag==1)   OLED_ShowString(0,50,"CAN  ");
		else if (Usart_ON_Flag==1) OLED_ShowString(0,50,"USART");
		else if (nav_walk==1)      OLED_ShowString(0,50,"nav");
		else                       OLED_ShowString(0,50,"ROS ");

		//Displays whether controls are allowed in the current car
		//显示当前小车是否允许控制	
		if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N"); 
		else                      OLED_ShowString(45,50,"OFF");

		//Displays the current battery voltage
	  //显示当前电池电压	
															OLED_ShowString(88,50,".");
															OLED_ShowString(110,50,"V");
															OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
															OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
		if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);
		
		//Refresh the screen //刷新屏幕
		OLED_Refresh_Gram();		
	}
	else if(oled_page==1)
	{
		//自动回充套件Debug信息
		OLED_ShowString(07,00,"LA  LB  RB  RA");
		OLED_ShowNumber(0+9,10,L_A,1,12);
		OLED_ShowNumber(30+9,10,L_B,1,12);
		OLED_ShowNumber(60+9,10,R_B,1,12);
		OLED_ShowNumber(90+9,10,R_A,1,12);
		OLED_ShowString(0,30,"cur:"); 
		OLED_ShowString(75,30,"A"); 
		oled_showfloat(Charging_Current/1000.0f,30,30,2,2);
//		oled_showfloat((float)accel[2]/1671.84f,0,40,2,2);
		OLED_Refresh_Gram();	
	}

}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //对电池电压处理成百分比形式
	 Voltage_Show=(Voltage*100-2000)*5/26;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //车轮速度单位转换为0.01m/s，方便在APP显示
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	 
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //用于交替打印APP数据和显示波形
	 flag_show=!flag_show;
	
 if(PID_Send==1)
	 {
		 //Send parameters to the APP, the APP is displayed in the debug screen
		 //发送参数到APP，APP在调试界面显示
		 printf("{C%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI);//打印到APP上面
		 PID_Send=0;	
	 }	
	 else	if(flag_show==0)
	 {
		 //Send parameters to the APP and the APP will be displayed on the front page
		 //发送参数到APP，APP在首页显示
	   printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)gyro[2]); 
	 }
	 else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //发送参数到APP，APP在波形界面显示
	   printf("{B%d:%d:%d}$",(int)gyro[0],(int)gyro[1],(int)gyro[2]);
	 }

}

float base_vol=22.5f;
#define VOL_COUNT 100
float VolMean_Filter(float data)
{
    u8 i;
    double Sum_Speed = 0;
    float Filter_Speed;
    static  float Speed_Buf[VOL_COUNT]= {0};
	
	/*----------- 数组初始化 -----------*/
	static u8 once=1;
	if(once)
	{
		once=0;
		for(i=0;i<VOL_COUNT;i++)
			Speed_Buf[i]=base_vol;
	}
	/*-------------------------------*/
	
    for(i = 1 ; i<VOL_COUNT; i++)
    {
        Speed_Buf[i - 1] = Speed_Buf[i];
    }
    Speed_Buf[VOL_COUNT - 1] =data;

    for(i = 0 ; i < VOL_COUNT; i++)
    {
        Sum_Speed += Speed_Buf[i];
    }
    Filter_Speed = (float)(Sum_Speed / VOL_COUNT);
    return Filter_Speed;
}

