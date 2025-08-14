#include "balance.h"

//Whether the robot model is incorrectly marked
//机器人型号是否错误标志位
int robot_mode_check_flag=0;

//The current position of the slide represents the current rotation Angle of the front wheel, 
//which is specially designed for Ackerman car
//滑轨当前位置，代表前轮当前转角角度，顶配阿克曼小车专用
int SLIDE_POSITION=0;

//The target position of the slide track, representing the front wheel target rotation Angle, 
//is specially equipped with Ackerman car
//滑轨目标位置，代表前轮目标转角角度，顶配阿克曼小车专用
float Slide_target=0;

//Front wheel Angle zero
//前轮转角零点
int SERVO_BIAS;

//Maximum and minimum PWM values of steering gear
//舵机PWM值最大值与最小值
int Servo_max=2000, Servo_min=900;

//Front wheel Angle steering speed
//前轮转角转向速度
float Angle_Smoother_Rate=20;

//Forward and backward velocity acceleration
//前进后退速度加速度
float Velocity_Smoother_Rate=0.02;

//Time variable //计时变量
int Time_count=0;

u8 command_lost_count=0;//串口、CAN控制命令丢失时间计数，丢失一秒后停止控制

//========== PWM清除使用变量 ==========//
u8 start_check_flag = 0;//标记是否需要清空PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //标记开始清除PWM
u8 clear_done_once = 0; //清除完成标志位
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
/*------------------------------------*/

//自检参数
int check_a,check_b,check_c,check_d;
u8 check_end=0;

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vz)
{   
		//Wheel target speed limit //车轮目标速度限幅
	  float amplitude=4.5;
	 
	    #if Akm_Car
	
		if(Car_Mode==0||Car_Mode==1||Car_Mode==7||Car_Mode==8) //SENIOR_AKM - 高配阿克曼   测试专用Car_Mode==8
		{
			//Ackerman car specific related variables //阿克曼小车专用相关变量
			float R, Ratio=636.56, AngleR, Angle_Servo;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//对于阿克曼小车Vz代表右前轮转向角度
			AngleR=Vz;
			R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
			
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
			AngleR=target_limit_float(AngleR,-0.50f,0.34f);
			
			//Inverse kinematics //运动学逆解
			if(AngleR!=0)
			{
				MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
				MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;
			}
			else 
			{
				MOTOR_A.Target = Vx;
				MOTOR_B.Target = Vx;
			}
			// The PWM value of the servo controls the steering Angle of the front wheel
			//舵机PWM值，舵机控制前轮转向角度
			Angle_Servo    =  -0.2137f*pow(AngleR, 2) + 1.439f*AngleR + 0.009599f;
			Servo=SERVO_INIT + SERVO_BIAS + Angle_Servo*Ratio;
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target, -amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target, -amplitude,amplitude); 
			Servo=target_limit_int(Servo, Servo_min, Servo_max);	//Servo PWM value limit //舵机PWM值限幅
		}
	  
		else if(Car_Mode==2||Car_Mode==3) //TOP_AKM_BS - 顶配阿克曼小车摆式悬挂
		{
			//Ackerman car specific related variables //阿克曼小车专用相关变量
			int TargetServo;	
			float R, AngleR;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//对于阿克曼小车Vz代表右前轮转向角度
			AngleR=Vz;
			R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
			AngleR=target_limit_float(AngleR,-0.47f,0.34f);
			
			//The trolley accelerates larger, adding smoothing processing
			//该小车加速度较大，添加平滑处理
			if(APP_ON_Flag==1||PS2_ON_Flag==1||Remote_ON_Flag==1||CAN_ON_Flag==1||Usart_ON_Flag==1)
			{
				//Smoothing the input speed //对输入速度进行平滑处理
			  Smooth_control(Vx, 0.04); 
        //Get the smoothed data //获取平滑处理后的数据					
	      Vx=smooth_control.VX;   
			}
			
      //Inverse kinematics //运动学逆解
			if(AngleR!=0)
			{
				MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
				MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;		
			}
			else 
			{
				MOTOR_A.Target = Vx;
				MOTOR_B.Target = Vx;
			}
			Slide_target= -2450*pow(AngleR, 3) + 1290*pow(AngleR, 2) + 3242*AngleR - 228.6f;
			Slide_target= Slide_target + SERVO_BIAS;		
		  
			//Closed-loop feedback control of the front wheel Angle
			//闭环反馈控制前轮转角
			TargetServo=Incremental_SERVO(SLIDE_POSITION, Slide_target);
			Servo=Smooth_steering(Servo, TargetServo, Angle_Smoother_Rate);
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target, -amplitude,amplitude);
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target, -amplitude,amplitude);
			//Servo PWM value limit //舵机PWM值限幅
			Servo=target_limit_int(Servo, Servo_min, Servo_max);
		}
		
		else if(Car_Mode==4||Car_Mode==5) //TOP_AKM_DL - 顶配阿克曼小车独立悬挂
		{
			//Ackerman car specific related variables //阿克曼小车专用相关变量
			int TargetServo;	
			float R, AngleR;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//对于阿克曼小车Vz代表右前轮转向角度
			AngleR=Vz;
			R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
			AngleR=target_limit_float(AngleR, -0.48f, 0.32f);
					
			//The trolley accelerates larger, adding smoothing processing
			//该小车加速度较大，添加平滑处理
			if((APP_ON_Flag==1||PS2_ON_Flag==1||Remote_ON_Flag==1||CAN_ON_Flag==1||Usart_ON_Flag==1)&&(Car_Mode==5))
			{
			 Smooth_control(Vx, Velocity_Smoother_Rate); //对输入速度进行平滑处理	    
	     Vx=smooth_control.VX;   //获取平滑处理后的数据
			}			
			
			//Inverse kinematics //运动学逆解
			if(AngleR!=0)
			{
				MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
				MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;
			}
			else
			{
				MOTOR_A.Target = Vx;
				MOTOR_B.Target = Vx;
			}
					
      Slide_target= 4515*pow(AngleR, 3) - 174.2f*pow(AngleR, 2) - 4323*AngleR + 42.01f;
			Slide_target= Slide_target + SERVO_BIAS;		
			
			//Closed-loop feedback control of the front wheel Angle
			//闭环反馈控制前轮转角
			TargetServo=Incremental_SERVO(-SLIDE_POSITION, Slide_target);
			Servo=TargetServo;
			//Servo=Smooth_steering(Servo, TargetServo, Angle_Smoother_Rate);
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target, -amplitude,amplitude);
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target, -amplitude,amplitude);
			//Servo PWM value limit //舵机PWM值限幅
			Servo=target_limit_int(Servo, Servo_min, Servo_max);		
		}
		
		#elif Diff_Car
		
		//速度限幅
		Vx=target_limit_float(Vx,-amplitude,amplitude);
		Vz=target_limit_float(Vz,-amplitude,amplitude);
		
		if(Allow_Recharge==0)
			Smooth_control(Vx,Vz); //Smoothing the input speed //对输入速度进行平滑处理
		else
			smooth_control.VX = Vx,
			smooth_control.VZ = Vz;
		
		//平滑后的速度
		Vx=smooth_control.VX;
		Vz=smooth_control.VZ;
		
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; 
		MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; 
		
		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude);
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude);
		
		#endif
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//此任务以100Hz的频率运行（10ms控制一次）
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
			
			//Time count is no longer needed after 30 seconds
			//时间计数，30秒后不再需要
			if(Time_count<3000)Time_count++;
			//and convert to transposition international units
			//获取编码器数据，即车轮实时速度，并转换位国际单位
			Get_Velocity_Form_Encoder();

			if( Allow_Recharge==1 )
				if( Get_Charging_HardWare==0 ) Allow_Recharge=0,Find_Charging_HardWare();
		
			#if Akm_Car
			//Gets the position of the slide, representing the front wheel rotation Angle
			//获取滑轨位置,代表前轮转角角度
			SLIDE_POSITION=Get_Adc(WEITIAO)-2048;
			SLIDE_POSITION=Mean_Filter(SLIDE_POSITION);
			#endif
			
			//Do not enter the control before the end of self-check to prevent the PID control from starting integration
			//自检结束前不进入控制，防止PID控制开始积分
			if(Time_count>CONTROL_DELAY+230) 
			{
//				command_lost_count++;//串口、CAN控制命令丢失时间计数，丢失1秒后停止控制
//				if(command_lost_count>RATE_100_HZ&&APP_ON_Flag==0&&Remote_ON_Flag==0&&PS2_ON_Flag==0)
//					Move_X=0,Move_Y=0,Move_Z=0;
				
				if(Get_Charging_HardWare==1)
				{   //存在回充装备时，对回充装备的状态进行检测
					charger_check++;
					if( charger_check>RATE_100_HZ) charger_check=RATE_100_HZ+1,Allow_Recharge=0,RED_STATE=0,Recharge_Red_Move_X = 0,Recharge_Red_Move_Y = 0,Recharge_Red_Move_Z = 0;
				}
			
				if(Allow_Recharge==1)
				{
					//如果开启了导航回充，同时没有接收到红外信号，接收来自上位机的的回充控制命令
					if      (nav_walk==1 && RED_STATE==0) Drive_Motor(Recharge_UP_Move_X,Recharge_UP_Move_Z); 
					//接收到了红外信号，接收来自回充装备的回充控制命令
					else if (RED_STATE!=0) nav_walk = 0,Drive_Motor(Recharge_Red_Move_X,Recharge_Red_Move_Z); 
					//防止没有红外信号时小车运动
					if (nav_walk==0&&RED_STATE==0) Drive_Motor(0,0); 
				}
				else
				{
					if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //处理APP遥控命令
					else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //处理航模遥控命令
					else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //处理PS2手柄控制命令

					//CAN, Usart 1, Usart 3 control can directly get the 2 axis target speed, 
					//without additional processing
					//CAN、串口1、串口3(ROS)控制直接得到2轴目标速度，无须额外处理
					else                      Drive_Motor(Move_X, Move_Z);  //CAN、串口1、串口3(ROS)控制
				}
			}

				//等陀螺仪初始化完成后,检测机器人型号是否选择错误
				//When the gyroscope is initialized, check whether the robot model is selected incorrectly
				if(CONTROL_DELAY<Time_count && Time_count<CONTROL_DELAY+200) //Advance 1 seconds to test //前进1秒进行测试
				{
					Drive_Motor(0.2, 0);
					robot_mode_check(); //Detection function //检测函数
				}
				else if(CONTROL_DELAY+200<Time_count && Time_count<CONTROL_DELAY+230)
				{
					check_end=1;
					Drive_Motor(0, 0); //The stop forward control is completed //检测完成停止前进控制
				}
				
				//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
				//and the software failure flag is 0, or the model detection marker is 0
				//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0，或者型号检测标志位为0
				if((Turn_Off(Voltage)==0&&robot_mode_check_flag==0)||(Allow_Recharge&&EN&&!Flag_Stop))
				{ 		
					 //Speed closed-loop control to calculate the PWM value of each motor, 
					 //PWM represents the actual wheel speed					 
					 //速度闭环控制计算各电机PWM值，PWM代表车轮实际转速					 
					 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					 //if( MOTOR_A.Target>0) MOTOR_A.Motor_Pwm=16800;
	 				 //if( MOTOR_A.Target<0) MOTOR_A.Motor_Pwm=-16800;
					 //if( MOTOR_A.Target==0) MOTOR_A.Motor_Pwm=0;
					 //if( MOTOR_B.Target>0) MOTOR_B.Motor_Pwm=16800;
	 				 //if( MOTOR_B.Target<0) MOTOR_B.Motor_Pwm=-16800;
					 //if( MOTOR_B.Target==0) MOTOR_B.Motor_Pwm=0;
					 
					//检测是否需要清除PWM并自动执行清理
					auto_pwm_clear();
					
					 #if Akm_Car
					 //Set different PWM control polarity according to different car models
					 //根据不同小车型号设置不同的PWM控制极性
					 if      (Car_Mode==0)	Set_Pwm( MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,Servo); //MD36
					 else if (Car_Mode==1)	Set_Pwm( MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,Servo); //MD36
					 else if (Car_Mode==2)	Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm,Servo); //MD60
					 else if (Car_Mode==3)	Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm,Servo); //MD60
					 else if (Car_Mode==4)	Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm,Servo); //MD60
					 else if (Car_Mode==5)	Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm,Servo); //MD60
					 else if (Car_Mode==6)	Set_Pwm( MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,Servo); //MD36
					 	else if (Car_Mode==7)	Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm,Servo); //测试专用
						else if (Car_Mode==8)	Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm,Servo); //测试专用
					 #elif Diff_Car
					 //Set different PWM control polarity according to different car models
					 //根据不同小车型号设置不同的PWM控制极性
								if (Car_Mode==0)	Set_Pwm( MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, 0); //MD36
					 else if (Car_Mode==1)	Set_Pwm( MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, 0); //MD36
					 else if (Car_Mode==2)	Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm, 0); //MD60
					 else if (Car_Mode==3)	Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm, 0); //MD60
					 else if (Car_Mode==4)	Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm, 0); //MD60
					 else if (Car_Mode==5)	Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm, 0); //MD60
					 else if (Car_Mode==6)	Set_Pwm(-MOTOR_A.Motor_Pwm,MOTOR_B.Motor_Pwm, 0); //MD60
					  
					 #endif
					
				}
				//If Turn_Off(Voltage) returns to 1, or the model detection marker is 1, the car is not allowed to move, and the PWM value is set to 0
				//如果Turn_Off(Voltage)返回值为1，或者型号检测标志位为1，不允许控制小车进行运动，PWM值设置为0
				else	Set_Pwm(0,0,0);

				
			      //Click the user button to update the gyroscope zero
			//单击用户按键更新陀螺仪零点
				Key();
    }	
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int servo)
{  
	  //Forward and reverse control of motor
	  //电机正反转控制
		if(motor_a<0)			AIN2=0,		AIN1=1;   
		else 	            AIN2=1,		AIN1=0;
    //Motor speed control 
	  //电机转速控制	
	 TIM_SetCompare4(TIM8,myabs(motor_a));
	
		if(motor_b>0)			BIN2=0,		BIN1=1;   
		else 	            BIN2=1,			BIN1=0;
	  //Motor speed control 
	  //电机转速控制
		TIM_SetCompare3(TIM8,myabs(motor_b));
		Servo_PWM =servo; 
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( int voltage)
{
		u8 temp;
	  //static int stop_count, enable_count;
		if(voltage<10||EN==0||Flag_Stop==1)
		{	                                                
			temp=1;      
			PWMA=0;
			PWMB=0;
			PWMC=0;
			PWMD=0;
			AIN1=0;AIN2=0;
			BIN1=0;BIN2=0;
			CIN1=0;CIN2=0;
			DIN1=0;DIN2=0;					
		}
		else
			temp=0;
		return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

u32 int_abs(int a)
{
	u32 temp;
	if(a<0) temp=-a;
	else temp = a;
	return temp;
}

/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	
	if( start_clear ) 
	{
		if(Pwm>50) Pwm-=50;
		if(Pwm<-50) Pwm+=50;
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;

		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<0;
		else clear_state &= ~(1<<0);
	}
	
	 return Pwm; 
}
int Incremental_PI_B (float Encoder,float Target)
{
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	
	if( start_clear ) 
	{
		if(Pwm>50) Pwm-=50;
		if(Pwm<-50) Pwm+=50;
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<1;
		else clear_state &= ~(1<<1);
		
		//2个电机均清除完毕，则关闭清除任务
		if( (clear_state&0xff)==0x03 ) start_clear = 0,clear_done_once=1,clear_state=0;
	}
	
	 return Pwm; 
}
int Incremental_SERVO (float SlidePosition,float SlideTarget)
{
	 static float Bias,Pwm=1700,Last_bias;
	 Bias=SlideTarget-SlidePosition; //Calculate the deviation //计算偏差
	 if(Car_Mode==2||Car_Mode==3)
	 {
		 //The mechanical structure determines that the right turn stroke is longer, so the steering speed should be accelerated
		 //机械结构决定右转行程比较长，此时加快转向速度
	   if(SlideTarget<0||SlidePosition<-200) Pwm+=0.004*1.6*(Bias-Last_bias)+0.009*1.6*Bias; 
	   else Pwm+=0.004f*(Bias-Last_bias)+0.009f*Bias;   
	 }
	 if(Car_Mode==4||Car_Mode==5)
	 {
		 //The mechanical structure determines that the right turn stroke is longer, so the steering speed should be accelerated
		 //机械结构决定右转行程比较长，此时加快转向速度
	   if(SlideTarget>0||SlidePosition<200) Pwm+=0.004*1.6*(Bias-Last_bias)+0.009*1.6*Bias;
	   else Pwm+=0.004f*(Bias-Last_bias)+0.009f*Bias;   
	 }
   if(Pwm>Servo_max)Pwm=Servo_max;
	 if(Pwm<Servo_min)Pwm=Servo_min;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;
}

/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
   { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
		  case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/4;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/4;   	 break;
		 
			#if Akm_Car                          //AKM car Z stands for front wheel steering Angle //Akm车Z代表前轮转向角
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/4;     break;
			#elif Diff_Car                       //Differential car Z stands for clockwise(<0) and counterclockwise(>0) rotation //差速车Z代表顺(<0)逆(>0)时针旋转
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=+PI/4;     break;
			#endif
		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;
		 
		  #if Akm_Car                          //AKM car Z stands for front wheel steering Angle //Akm车Z代表前轮转向角
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/4;    break;
		  #elif Diff_Car                       //Differential car Z stands for clockwise(<0) and counterclockwise(>0) rotation //差速车Z代表顺(<0)逆(>0)时针旋转
		  case 6:      Move_X=-RC_Velocity;  	 Move_Z=-PI/4;    break;
		  #endif
		 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/4;    break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/4;    break; 
			default:     Move_X=0;               Move_Z=0;        break;
   }
	
	 //AKM car Z stands for front wheel steering Angle 
	 //Akm车Z代表前轮转向角
	 #if Akm_Car 
	 //Different Ackerman cars have different maximum steering angles
	 //不同阿克曼小车的最大转向角不一样
	 if     (Car_Mode==2||Car_Mode==3) Move_Z=Move_Z*2/3;
	 else if(Car_Mode==4||Car_Mode==5) Move_Z=Move_Z/2;
	 else if(Car_Mode==6)              Move_Z=Move_Z*0.4f;
	 else Move_Z=Move_Z/2;
	 
	 //Differential car Z stands for clockwise(<0) and counterclockwise(>0) rotation 
	 //差速车Z代表顺(<0)逆(>0)时针旋转
	 #elif Diff_Car
	 //The greater the forward speed, the greater the rotation speed
	 //前进速度越大旋转速度越大
	 Move_Z=Move_Z*RC_Velocity/800; 
	 #endif

	 //Unit conversion, mm/s -> m/s
   //单位转换，mm/s -> m/s	
	 Move_X=Move_X/1000;
	 
	 //Control target value is obtained and kinematics analysis is performed
	 //得到控制目标值，进行运动学分析
	 Drive_Motor(Move_X,Move_Z);
}
/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY;
		int Yuzhi=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
			
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
		LY=-(PS2_LX-128);
		LX=-(PS2_LY-128);
		RY=-(PS2_RX-128);

	  //Ignore small movements of the joystick //忽略摇杆小幅度动作
		if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
		if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
		if(RY>-Yuzhi&&RY<Yuzhi)RY=0;

	  if     (PS2_KEY==11) RC_Velocity+=5;  //To accelerate//加速
		else if(PS2_KEY==9)	 RC_Velocity-=5;  //To slow down //减速
	
		if(RC_Velocity<0)    RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //对PS2手柄控制命令进行处理
		Move_X=LX;
		Move_Z=RY;
	    Move_X=Move_X*RC_Velocity/128;
		Move_Z=Move_Z*(PI/4)/128;
		
	  //AKM car Z stands for front wheel steering Angle 
	  //Akm车Z代表前轮转向角
		#if Akm_Car
	  //Different Ackerman cars have different maximum steering angles
	  //不同阿克曼小车的最大转向角不一样
	  if     (Car_Mode==2||Car_Mode==3) Move_Z=Move_Z*2/3;
		else if(Car_Mode==4||Car_Mode==5) Move_Z=Move_Z/2;
		else if(Car_Mode==6)              Move_Z=Move_Z*0.4f;
		else Move_Z=Move_Z/2;
	  #endif
		
		//Differential car Z stands for clockwise(<0) and counterclockwise(>0) rotation 
	  //差速车Z代表顺(<0)逆(>0)时针旋转
		#if Diff_Car 
		//The greater the forward speed, the greater the rotation speed
	  //前进速度越大旋转速度越大
		if(Move_X<0)Move_Z=-Move_Z*(RC_Velocity/500);
		#endif

	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s
		Move_X=Move_X/1000;

	  //Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Z);		
} 

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    static u8 thrice=100;
    int Yuzhi=100; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
	
	  //limiter //限幅
    int LX,LY,RY,RX,Remote_RCvelocity; 					//
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

		//Front and back direction of left rocker. Control forward and backward.
	  //左摇杆前后方向。控制前进后退。
    LX=Remoter_Ch2-1500;
	  //The channel is not currently in use
	  //该通道暂时没有使用到
    LY=Remoter_Ch4-1500;
	
		  //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//右摇杆前后方向。油门/加减速。
	  RX=Remoter_Ch3-1500;											//
	  //Right stick left and right. To control the rotation. 
		//右摇杆左右方向。控制自转。
    RY=-(Remoter_Ch1-1500);//自转

    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
		if(RX>-Yuzhi&&RX<Yuzhi)RX=0;							//
    if(RY>-Yuzhi&&RY<Yuzhi)RY=0;
		
		
		//Throttle related //油门相关
		Remote_RCvelocity=RC_Velocity+RX;				//
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;					//
			
		//The remote control command of model aircraft is processed
		//对航模遥控控制命令进行处理
		Move_X=LX;
		Move_Z=RY;
		Move_X=Move_X*Remote_RCvelocity/500;					//
		Move_Z=Move_Z*(PI/4)/500;
		
	  //AKM car Z stands for front wheel steering Angle 
	  //Akm车Z代表前轮转向角
		#if Akm_Car
	  //Different Ackerman cars have different maximum steering angles
	  //不同阿克曼小车的最大转向角不一样
	  if     (Car_Mode==2||Car_Mode==3) Move_Z=Move_Z*2/3;
		else if(Car_Mode==4||Car_Mode==5) Move_Z=Move_Z/2;
		else if(Car_Mode==6)              Move_Z=Move_Z*0.4f;
		else Move_Z=Move_Z/2;
	  #endif
		
		//Differential car Z stands for clockwise(<0) and counterclockwise(>0) rotation 
	  //差速车Z代表顺(<0)逆(>0)时针旋转
		#if Diff_Car 
		//The greater the forward speed, the greater the rotation speed
	  //前进速度越大旋转速度越大
		if(Move_X<0)Move_Z=-Move_Z*(RC_Velocity/500);
		#endif
			 
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s
		Move_X=Move_X/1000;

	  //Data within 1 second after entering the model control mode will not be processed
	  //对进入航模控制模式后1秒内的数据不处理
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;

		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	//传入任务的频率
	tmp=KEY_Scan(RATE_100_HZ,0);
	
	//单击
	if(tmp==single_click)
	{
		Allow_Recharge=!Allow_Recharge;
		memcpy(Deviation_gyro,Original_gyro,sizeof(gyro)),memcpy(Deviation_accel,Original_accel,sizeof(accel));
	}
	
	//双击
	else if(tmp==double_click) memcpy(Deviation_gyro,Original_gyro,sizeof(gyro)),memcpy(Deviation_accel,Original_accel,sizeof(accel));
	
	//长按
	else if(tmp==long_click) 
	{
		oled_refresh_flag=1;
		oled_page++;
		if(oled_page>OLED_MAX_Page-1) oled_page=0;
	}
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	//Retrieves the original data of the encoder
	//获取编码器的原始数据
	float Encoder_A_pr,Encoder_B_pr; 
	Encoder_A_pr=-Read_Encoder(2);  
	Encoder_B_pr= Read_Encoder(3);

    //未完成自检时收集编码器数据
    if( check_end==0 )
    {
        check_a+=-Encoder_A_pr;
        check_b+=Encoder_B_pr;
    }
	
  //The encoder converts the raw data to wheel speed in m/s
	//编码器原始数据转换为车轮速度，单位m/s
	MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY/Encoder_precision*Wheel_perimeter;
	MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY/Encoder_precision*Wheel_perimeter;
/*	if(Car_Mode==6){
	  MOTOR_A.Encoder= MOTOR_A.Target/2;
	  MOTOR_B.Encoder= MOTOR_B.Target/2;
	}*/
}
/**************************************************************************
Function: Smoothing the target velocity
Input   : Target velocity
Output  : none
函数功能：对目标速度做平滑处理
入口参数：目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx, float vz)
{
    float step=4.00; //平滑处理步进值

    //X轴速度平滑
    if(vx>smooth_control.VX)
    {
        smooth_control.VX+=step;
        if(smooth_control.VX>vx) smooth_control.VX=vx;
    }
    else if (vx<smooth_control.VX)
    {
        smooth_control.VX-=step;
        if(smooth_control.VX<vx) smooth_control.VX=vx;
    }
    else
        smooth_control.VX =vx;

    //Z轴速度平滑
    if(vz>smooth_control.VZ)
    {
        smooth_control.VZ+=step;
        if(smooth_control.VZ>vz) smooth_control.VZ=vz;
    }
    else if (vz<smooth_control.VZ)
    {
        smooth_control.VZ-=step;
        if(smooth_control.VZ<vz) smooth_control.VZ=vz;
    }
    else
        smooth_control.VZ =vz;

    //0速时保证静止稳定
    if(vx==0&&smooth_control.VX<0.05f&&smooth_control.VX>-0.05f) smooth_control.VX=0;
    if(vz==0&&smooth_control.VZ<0.05f&&smooth_control.VZ>-0.05f) smooth_control.VZ=0;
}

/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.
Input   : none
Output  : none
函数功能：防止电位器选错模式，导致初始化出错引发电机乱转。
入口参数：无
返回  值：无
**************************************************************************/
void robot_mode_check(void)
{
#define ERROR_PWM 8000 //pwm预警值
    static u8 once=1;
    if( once ) check_a=0,check_b=0,check_c=0,check_d=0,once=0;

    if( EN==1 && robot_mode_check_flag==0) //保留可以使用急停开关跳过自检的功能
    {
        //拥有一定pwm参数值后，若编码器数据不变。错误类型为：编码器未接线、驱动未接线或负责超重
        if(float_abs(MOTOR_A.Motor_Pwm)>5500 && check_a<500) robot_mode_check_flag=1,LED_B=0;
        if(float_abs(MOTOR_B.Motor_Pwm)>5500 && check_b<500) robot_mode_check_flag=1,LED_B=0;
        if(float_abs(MOTOR_C.Motor_Pwm)>5500 && check_c<500) robot_mode_check_flag=1,LED_B=0;
        if(float_abs(MOTOR_D.Motor_Pwm)>5500 && check_d<500) robot_mode_check_flag=1,LED_B=0;

        //若存在负数，说明存在方向相反的情况：错误类型为：车型选错、驱动接线错误或编码器接线错误
        if( check_a<-3000 ||check_b<-3000 ||check_c<-3000 ||check_d<-3000 ) robot_mode_check_flag=1,LED_G=0;

        //最后防线，正常0.2m/s速度无法到达的PWM数值，错误类型：负载已经超出电机能承受的范围
        if( float_abs(MOTOR_A.Motor_Pwm)>ERROR_PWM||float_abs(MOTOR_B.Motor_Pwm)>ERROR_PWM||\
                float_abs(MOTOR_C.Motor_Pwm)>ERROR_PWM||float_abs(MOTOR_D.Motor_Pwm)>ERROR_PWM )
        {
            robot_mode_check_flag = 1;
            LED_B=1,LED_G=1;
        }

    }
}


//PWM消除函数
void auto_pwm_clear(void)
{
	//小车姿态简易判断
	float y_accle = (float)(accel[1]/1671.84f);//Y轴加速度实际值
	float z_accle = (float)(accel[2]/1671.84f);//Z轴加速度实际值
	float diff;
	
	//计算Y、Z加速度融合值，该值越接近9.8，表示小车姿态越水平
	if( y_accle > 0 ) diff  = z_accle - y_accle;
	else diff  = z_accle + y_accle;
	
//	debug_show_diff = diff;
	
	//PWM消除检测
	if( smooth_control.VX !=0.0f || smooth_control.VZ != 0.0f || smooth_control.VY != 0.0f )
	{
		start_check_flag = 1;//标记需要清空PWM
		wait_clear_times = 0;//复位清空计时
		start_clear = 0;     //复位清除标志
		
		
		//运动时斜坡检测的数据复位
		clear_done_once = 0;
		clear_again_times=0;
	}
	else //当目标速度由非0变0时，开始计时 2.5 秒，若小车不在斜坡状态下，清空pwm
	{
		if( start_check_flag==1 )
		{
			wait_clear_times++;
			if( wait_clear_times >= 250 )
			{
				//小车在水平面上时才标记清空pwm，防止小车在斜坡上运动出现溜坡
				if( diff > 8.8f )	start_clear = 1,clear_state = 0;//开启清除pwm
				else clear_done_once = 1;//小车在斜坡上，标记已完成清除
				
				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	//完成了清除后，若出现推车行为，pwm积累一定数值后将在10秒后再次清空
	if( clear_done_once )
	{
		//小车接近于水平面时才作积累消除，防止小车在斜坡上溜车
		if( diff > 8.8f )
		{
			//完成清除后pwm再次积累，重新清除
			if( int_abs(MOTOR_A.Motor_Pwm)>300 || int_abs(MOTOR_B.Motor_Pwm)>300 || int_abs(MOTOR_C.Motor_Pwm)>300 || int_abs(MOTOR_D.Motor_Pwm)>300 )
			{
				clear_again_times++;
				if( clear_again_times>1000 )
				{
					clear_done_once = 0;
					start_clear = 1;//开启清除pwm
					clear_state = 0;
				}
			}
			else
			{
				clear_again_times = 0;
			}
		}
		else
		{
			clear_again_times = 0;
		}

	}
}




