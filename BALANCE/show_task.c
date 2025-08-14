#include "show_task.h"
#include "bsp_gamepad.h"

OLED_t oled;

void OLED_ShowGamePadState(void)
{
	oled.page = 0;
}

//OLED�ṹ���ʼ��
void OLED_Param_Init(OLED_t* p)
{
	p->page = 1;
	p->last_page = 1;
	p->refrsh = 0;
	p->MAX_PAGE = OLED_MAX_PAGE;
}

/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
�������ܣ���ȡ��ص�ѹ�������������������Լ졢��APP�������ݡ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
TaskHandle_t show_TaskHandle = NULL;
void show_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, F2T(SHOW_TASK_RATE));//This task runs at 10Hz //��������10Hz��Ƶ������

        //����������,ʹ�� Buzzer_AddTask(a,b) ��������ӷ���������
		Buzzer_task(SHOW_TASK_RATE);
		
        //��ȡ��ص�ѹ����
		uint8_t i=0;
		float tmp = 0;
        for(i=0; i<100; i++)
        {
            tmp+=Get_battery_volt();
        }
        robot.voltage=tmp/100.0f;
        robot.voltage = VolMean_Filter(robot.voltage);
		
		//�͵�������1��
		static uint8_t low_power = 0;
		if( robot.voltage<20 && low_power==0 ) Buzzer_AddTask(1,100),low_power=1;
		if( robot.voltage>21.0f ) low_power = 0;
		
		//������,���ڻس�װ�����յ����׮�ĺ����ź�,�Զ������س书��
		if( 1 == low_power && 1 == SysVal.HardWare_charger && charger.RED_STATE>=2 )
		{
			if( SysVal.Time_count>CONTROL_DELAY ) charger.AllowRecharge = 1;
		}
		
        //APP��ʾ��������
		APP_ShowTask();
		
		//OLED��ʾ��������
		OLED_ShowTask();
    }
}


/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
�������ܣ���APP��������
��ڲ�������
����  ֵ����
**************************************************************************/
static void APP_ShowTask(void)
{
    static u8 flag_show;
    int Left_Figure,Right_Figure,Voltage_Show;

    //The battery voltage is processed as a percentage
    //�Ե�ص�ѹ����ɰٷֱ���ʽ
    Voltage_Show=(robot.voltage*100-2000)*5/26;
    if(Voltage_Show>100)Voltage_Show=100;

    //Wheel speed unit is converted to 0.01m/s for easy display in APP
    //�����ٶȵ�λת��Ϊ0.01m/s��������APP��ʾ
	#if defined AKM_CAR || defined DIFF_CAR //��������������ʾA��B��
		Left_Figure=robot.MOTOR_A.Encoder*100;
		if(Left_Figure<0)Left_Figure=-Left_Figure;
		Right_Figure=robot.MOTOR_B.Encoder*100;
		if(Right_Figure<0)Right_Figure=-Right_Figure;
	#elif defined MEC_CAR || defined _4WD_CAR //���֡�������ʾA��D��
		Left_Figure=robot.MOTOR_A.Encoder*100;
		if(Left_Figure<0)Left_Figure=-Left_Figure;
		Right_Figure=robot.MOTOR_D.Encoder*100;
		if(Right_Figure<0)Right_Figure=-Right_Figure;
	#else                                      //ȫ������ʾB��C��
		Left_Figure=robot.MOTOR_B.Encoder*100;
		if(Left_Figure<0)Left_Figure=-Left_Figure;
		Right_Figure=robot.MOTOR_C.Encoder*100;
		if(Right_Figure<0)Right_Figure=-Right_Figure;
	#endif

    //Used to alternately print APP data and display waveform
    //���ڽ����ӡAPP���ݺ���ʾ����
    flag_show=!flag_show;

    if(appkey.ParamSendflag==1)
    {
        //Send parameters to the APP, the APP is displayed in the debug screen
        //���Ͳ�����APP��APP�ڵ��Խ�����ʾ
        printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",
               (int)robot_control.rc_speed,
               (int)robot.V_KP,
               (int)robot.V_KI,
               (int)(robot_control.smooth_MotorStep*1000),
               (int)(robot_control.smooth_ServoStep),
               (int)(Akm_Servo.Max),
               (int)(Akm_Servo.Min),
               (int)(Akm_Servo.Mid),
			   robot_control.LineDiffParam);
				   

        appkey.ParamSendflag=0;
    }
    else if(flag_show==0)
    {
        //Send parameters to the APP and the APP will be displayed on the front page
        //���Ͳ�����APP��APP����ҳ��ʾ
        printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)imu.gyro.z);

    }
    else
    {
        //Send parameters to the APP, the APP is displayed in the waveform interface
        //���Ͳ�����APP��APP�ڲ��ν�����ʾ
        printf("{B%d:%d:%d}$",(int)imu.gyro.x,(int)imu.gyro.y,(int)imu.gyro.z);
    }
}

/**************************************************************************
Functionality: Battery voltage sliding filter function.
Input Parameters: Collected battery voltage data.
Return Value: Filtered battery voltage data.
Author: WHEELTEC
�������ܣ���ص�ѹ�����˲�����
��ڲ������ɼ����ĵ�ص�ѹ����
����  ֵ�������˲���ĵ�ص�ѹ����
��    �ߣ�WHEELTEC
**************************************************************************/
#define VOL_COUNT 100
static float VolMean_Filter(float data)
{
    u8 i;
    double Sum_Speed = 0;
    float Filter_Speed;
    static  float Speed_Buf[VOL_COUNT]= {0};

    /*----------- �����ʼ�� -----------*/
    static u8 once=1;
    if(once)
    {
        once=0;
        for(i=0; i<VOL_COUNT; i++)
            Speed_Buf[i]= robot.voltage ;
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

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
�������ܣ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
static void OLED_ShowTask(void)
{
	//���OLEDҳ���Ƿ���Ҫˢ��
	if( oled.page!=oled.last_page ) 
	{
		OLED_Clear(),OLED_Refresh_Gram();
		oled.last_page = oled.page;
		return;
	}
	oled.last_page = oled.page;
	
	//��ҳ��ʾС���ĸ�����������Ϣ
	if( 1 == oled.page ) 
	{
		//������ʾ��Ϣ,���г���,ĩ�е�ص�ѹ����Ʒ�ʽ
		uint16_t TypeNum = 4096/CAR_NUMBER;
		TypeNum = Get_ADC1_Average(CarMode_Ch,10)/TypeNum;
		
		//��һ����벿��,��ʾ��λ������.ͬʱ����ʾ�Ƿ������Զ��س书��
		if( 0 == charger.AllowRecharge) OLED_ShowString(0,0,"TYPE:");
		else                           OLED_ShowString(0,0,"RCM :");
		
		//���ͺ�,��������ʾX
		if( 0 == robot_check.errorflag) OLED_ShowNumber(40,0,TypeNum,2,12);
		else                           OLED_ShowString(38,0," X");
		
		//��һ���Ұ벿��,Z����ٶ�
		OLED_ShowString(60,0,"GZ");
		if( imu.gyro.z < 0)  OLED_ShowString(80,0,"-"),OLED_ShowNumber(90,0,-imu.gyro.z,5,12);
		else                 OLED_ShowString(80,0,"+"),OLED_ShowNumber(90,0, imu.gyro.z,5,12);
		//oled_showfloat(debug.u8_val,80,0,3,2);
		
		
		//���һ����벿��,��ʾ�������� �� ��ʾС���Ƿ���������
			  if( Get_Control_Mode(_ROS_Control) )    OLED_ShowString(0,50,"ROS  ");
		else if(  Get_Control_Mode(_PS2_Control) )   OLED_ShowString(0,50,"PS2  ");
		else if(  Get_Control_Mode(_APP_Control) )   OLED_ShowString(0,50,"APP  ");
		else if(  Get_Control_Mode(_RC_Control)  )   OLED_ShowString(0,50,"R-C  ");
		else if(  Get_Control_Mode(_CAN_Control) )   OLED_ShowString(0,50,"CAN  ");
		else if(  Get_Control_Mode(_USART_Control) ) OLED_ShowString(0,50,"USART");
		
		//��ʾС���Ƿ���������
		if( 0 == robot_control.FlagStop ) OLED_ShowString(45,50," ON");
		else                              OLED_ShowString(45,50,"OFF");
			
		//�Ұ벿����ʾ��ص�ѹ
		oled_showfloat(robot.voltage,75,50,2,2);
		OLED_ShowString(75,50," ");
		OLED_ShowString(120,50,"V");
		
		//��2��3��4��5�зǹ�������,���ݳ��Ͳ�ͬ��ʾ���ض���Ϣ
		#if defined AKM_CAR
		oled_akm_show();
		#elif defined DIFF_CAR
			oled_diff_show();
		#elif defined MEC_CAR || defined _4WD_CAR || defined OMNI_CAR
			oled_mec_4wd_omni_show();
		#endif
	}
	
	//�ڶ�ҳ��ʾ�Զ��س������Ϣ
	else if( 2 == oled.page ) 
	{
		//�Զ��س��׼�Debug��Ϣ
		OLED_ShowString(07,00,"LA  LB  RB  RA");
		OLED_ShowNumber(0+9,10,charger.L_A,1,12);
		OLED_ShowNumber(30+9,10,charger.L_B,1,12);
		OLED_ShowNumber(60+9,10,charger.R_B,1,12);
		OLED_ShowNumber(90+9,10,charger.R_A,1,12);
		OLED_ShowString(0,30,"cur:"); 
		OLED_ShowString(75,30,"A"); 
		oled_showfloat(charger.ChargingCurrent/1000.0f,30,30,2,2);
	}
	
	//����ҳ��ʾС���İ汾
	else if( 3 == oled.page )
	{
		//��1�� ��ʾ�����ֳ�
		OLED_ShowString(0,0,"CarMode:");
		#if defined AKM_CAR
			OLED_ShowString(66,0,"AKM");
		#elif defined DIFF_CAR
			OLED_ShowString(66,0,"DIFF");
		#elif defined MEC_CAR
			OLED_ShowString(66,0,"MEC");
		#elif defined _4WD_CAR
			OLED_ShowString(66,0,"4WD");
		#elif defined OMNI_CAR
			OLED_ShowString(66,0,"OMNI");
		#endif
		
		//�ڶ�����ʾ���ʹ���
		OLED_ShowString(0,15,"CarType:");
		OLED_ShowNumber(66,15,robot.type,2,12);
		
		//��������ʾӲ���汾
		OLED_ShowString(0,30,"HW_Ver:");
		OLED_ShowString(66,30,getHW_Ver(SysVal.HardWare_Ver));
		
		//��������ʾ����汾
		OLED_ShowString(0,45,"SW_Ver:");
		OLED_ShowString(66,45,getSW_Ver(SysVal.SoftWare_Ver));
	}
	
	//0ҳ,�û��޷����з��ʵ�ҳ,������ʾ usb ps2 �ֱ�״̬
	else if( 0 == oled.page )
	{
		if( GamePadDebug.enmu_state == EnumWait ) //ö�ٵȴ���
		{
			OLED_DrawBMP(32,1,96,7,gImage_usb_bmp);//����usb��ʾ
			OLED_ShowString(12,50,"USB Init..");
			OLED_Refresh_Line();
			return;
		}
		else if( GamePadDebug.enmu_state == EnumNULL )
		{
			OLED_ClearBuf();
			oled.page = 1;
		}
		else if( GamePadDebug.enmu_state == EnumDone ) //ö�����
		{
			//ö�����,��ʾö�ٽ����,�ȴ�һ��ʱ��ָ�������ʾ
			static uint16_t showtime=0;
			if(++showtime >= SHOW_TASK_RATE*3 )
			{
				showtime = 0;
				oled.page = 1;
			}
			
			//��ջ���������ˢ��,�ȴ�д���µ�����
			OLED_ClearBuf();
			
			//ps2��ʼ�����
			OLED_ShowString(0,0,"USB Init OK.");
			OLED_ShowString(0,15,"PS2 Info:");
			
			if( GamePadDebug.type == PS2_USB_Wired || GamePadDebug.type == PS2_USB_WiredV2 )
			{
				OLED_ShowString(0,30,"Wired USBPS2");
			}
			else if( GamePadDebug.type == PS2_USB_Wiredless )
			{
				OLED_ShowString(0,30,"2.4G USBPS2 ");
			}
			else if( GamePadDebug.type == Xbox360 )
			{
				OLED_ShowString(0,30,"xbox 360    ");
			}
			else OLED_ShowString(0,30,"UnKnown Dev ");
			
			//�Ƿ�ɹ���ȡ��ps2������
			OLED_ShowString(0,45,"Data Ready:");
			if( 1 == GamePadDebug.ready ) //�ɹ���ȡps2����
			{
				OLED_ShowString(90,45,"Yes");
			}
			else //������
			{
				OLED_ShowString(90,45,"No ");
			}
			
		}
	}
	
	//oledˢ��
	OLED_Refresh_Gram();
}


//��ͬ���Ͷ�Ӧ����ʾҳ��
#if defined AKM_CAR
static void oled_akm_show(void)
{
	//���̲���ģʽ,���ڻ�е��װʹ��
	if( robot.type==7 || robot.type==8 )
	{
		OLED_ShowString(00,10,"SLID:");
		if( robot.SERVO.Encoder < 0 )	OLED_ShowString(60,10,"-"),
								OLED_ShowNumber(80,10,-(int)robot.SERVO.Encoder,4,12);
		else                 	OLED_ShowString(60,10,"+"),
								OLED_ShowNumber(80,10, (int)robot.SERVO.Encoder,4,12); 
	}
	else
	{
		OLED_ShowString(00,10,"ACCEL ");
		oled_showfloat(imu.accel.z/1671.84f,80,10,2,2);
	}
	
	 //The third line of the display displays the content//
	 //��ʾ����3����ʾ����//		
	 //Display the target speed and current speed of motor A
	 //��ʾ���A��Ŀ���ٶȺ͵�ǰ�ٶ�
	 OLED_ShowString(0,20,"L:");
	 if( robot.MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
									OLED_ShowNumber(20,20,-robot.MOTOR_A.Target*1000,5,12);
	 else                 	        OLED_ShowString(15,20,"+"),
									OLED_ShowNumber(20,20, robot.MOTOR_A.Target*1000,5,12); 
		
	 if( robot.MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
									OLED_ShowNumber(75,20,-robot.MOTOR_A.Encoder*1000,5,12);
	 else                 	        OLED_ShowString(60,20,"+"),
									OLED_ShowNumber(75,20, robot.MOTOR_A.Encoder*1000,5,12);
	
	 //The fourth line of the display displays the content//
	 //��ʾ����4����ʾ����//
	 //Display the target speed and current speed of motor B
	 //��ʾ���B��Ŀ���ٶȺ͵�ǰ�ٶ�
	 OLED_ShowString(0,30,"R:");
	 if( robot.MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
									OLED_ShowNumber(20,30,-robot.MOTOR_B.Target*1000,5,12);
	 else                 	        OLED_ShowString(15,30,"+"),
									OLED_ShowNumber(20,30,  robot.MOTOR_B.Target*1000,5,12); 
			
	 if( robot.MOTOR_B.Encoder<0)	OLED_ShowString(60,30,"-"),
									OLED_ShowNumber(75,30,-robot.MOTOR_B.Encoder*1000,5,12);
     else                 	        OLED_ShowString(60,30,"+"),
									OLED_ShowNumber(75,30, robot.MOTOR_B.Encoder*1000,5,12);

	 //��ʾ��ǰ���PWM����ֵ
	 //Displays the current steering gear PWM control value
	 OLED_ShowString(00,40,"SERVO:");
	 
	 if( 1 == ServoState.UnLock ) //������ģʽ��ʾ���һ������������PWMֵ
	 {
		OLED_ShowString(60,40," "),
		OLED_ShowNumber(80,40, ServoState.UnLock_Output,4,12); 
	 }
	 else //����ģʽ����ʾʵʱPWMֵ
	 {
		 if( robot.SERVO.Output<0)	OLED_ShowString(60,40,"-"),
									OLED_ShowNumber(80,40,-robot.SERVO.Output,4,12);
		 else                 	    OLED_ShowString(60,40,"+"),
									OLED_ShowNumber(80,40, robot.SERVO.Output,4,12); 
	 }

}


#elif defined DIFF_CAR
static void oled_diff_show(void)
{
	//��ʾ���ٶ�z������
	OLED_ShowString(00,10,"ACCEL ");
	oled_showfloat(imu.accel.z/1671.84f,80,10,2,2);
	
	//The third line of the display displays the content//
	//��ʾ����3����ʾ����//
	//Display the target speed and current speed of motor A
	//��ʾ���A��Ŀ���ٶȺ͵�ǰ�ٶ�	 
	OLED_ShowString(0,20,"DL:");
	if( robot.MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
	                            OLED_ShowNumber(20,20,-robot.MOTOR_A.Target*1000,5,12);
	else                 	    OLED_ShowString(15,20,"+"),
	                            OLED_ShowNumber(20,20, robot.MOTOR_A.Target*1000,5,12); 

	if( robot.MOTOR_A.Encoder<0) OLED_ShowString(60,20,"-"),
	                             OLED_ShowNumber(75,20,-robot.MOTOR_A.Encoder*1000,5,12);
	else                 	     OLED_ShowString(60,20,"+"),
	                             OLED_ShowNumber(75,20, robot.MOTOR_A.Encoder*1000,5,12);

	//The fourth line of the display displays the content//
	//��ʾ����4����ʾ����//	
	//Display the target speed and current speed of motor B
	//��ʾ���B��Ŀ���ٶȺ͵�ǰ�ٶ�
	OLED_ShowString(0,30,"DR:");
	if( robot.MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
	                            OLED_ShowNumber(20,30,- robot.MOTOR_B.Target*1000,5,12);
	else                 	    OLED_ShowString(15,30,"+"),
	                            OLED_ShowNumber(20,30,  robot.MOTOR_B.Target*1000,5,12); 

	if( robot.MOTOR_B.Encoder<0)OLED_ShowString(60,30,"-"),
	                            OLED_ShowNumber(75,30,-robot.MOTOR_B.Encoder*1000,5,12);
	else                 	    OLED_ShowString(60,30,"+"),
	                            OLED_ShowNumber(75,30, robot.MOTOR_B.Encoder*1000,5,12);
	
	OLED_ShowString(00,40,"MA");
	if( robot.MOTOR_A.Output < 0 )   OLED_ShowString(20,40,"-"),
	                                 OLED_ShowNumber(30,40,-robot.MOTOR_A.Output,4,12);
	else                 	         OLED_ShowString(20,40,"+"),
	                                 OLED_ShowNumber(30,40, robot.MOTOR_A.Output,4,12); 
	OLED_ShowString(60,40,"MB");
	if(robot.MOTOR_B.Output<0)       OLED_ShowString(80,40,"-"),
	                                 OLED_ShowNumber(90,40,-robot.MOTOR_B.Output,4,12);
	else                 	         OLED_ShowString(80,40,"+"),
	                                 OLED_ShowNumber(90,40, robot.MOTOR_B.Output,4,12);
	
}


#elif defined MEC_CAR || defined _4WD_CAR || defined OMNI_CAR
static void oled_mec_4wd_omni_show(void)
{
	//The second line of the display displays the content//
	//��ʾ����2����ʾ����//	
	//Display the target speed and current speed of motor A
	//��ʾ���A��Ŀ���ٶȺ͵�ǰ�ٶ�
	OLED_ShowString(0,10,"A");
	if( robot.MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
								OLED_ShowNumber(20,10,-robot.MOTOR_A.Target*1000,5,12);
	else                 	    OLED_ShowString(15,10,"+"),
								OLED_ShowNumber(20,10, robot.MOTOR_A.Target*1000,5,12); 
	
	if( robot.MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
								OLED_ShowNumber(75,10,-robot.MOTOR_A.Encoder*1000,5,12);
	else                 	    OLED_ShowString(60,10,"+"),
								OLED_ShowNumber(75,10, robot.MOTOR_A.Encoder*1000,5,12);
	
	//The third line of the display displays the content//
	//��ʾ����3����ʾ����//	
	//Display the target speed and current speed of motor B
	//��ʾ���B��Ŀ���ٶȺ͵�ǰ�ٶ�
	OLED_ShowString(0,20,"B");		
	if( robot.MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
								OLED_ShowNumber(20,20,-robot.MOTOR_B.Target*1000,5,12);
	else                 	    OLED_ShowString(15,20,"+"),
								OLED_ShowNumber(20,20, robot.MOTOR_B.Target*1000,5,12); 
	
	if( robot.MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
								OLED_ShowNumber(75,20,-robot.MOTOR_B.Encoder*1000,5,12);
	else                 	    OLED_ShowString(60,20,"+"),
								OLED_ShowNumber(75,20, robot.MOTOR_B.Encoder*1000,5,12);
	
	//The fourth line of the display displays the content//
	//��ʾ����4����ʾ����//
	//Display the target speed and current speed of motor C
	//��ʾ���C��Ŀ���ٶȺ͵�ǰ�ٶ�
	OLED_ShowString(0,30,"C");
	if( robot.MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
								OLED_ShowNumber(20,30,- robot.MOTOR_C.Target*1000,5,12);
	else                 	    OLED_ShowString(15,30,"+"),
								OLED_ShowNumber(20,30,  robot.MOTOR_C.Target*1000,5,12); 
		
	if( robot.MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
								OLED_ShowNumber(75,30,-robot.MOTOR_C.Encoder*1000,5,12);
	else                     	OLED_ShowString(60,30,"+"),
								OLED_ShowNumber(75,30, robot.MOTOR_C.Encoder*1000,5,12);
	
	
	//���֡�������ʾD���
	#if !defined OMNI_CAR
	
	//Line 5 of the display displays the content//
	//��ʾ����5����ʾ����//
	OLED_ShowString(0,40,"D");
	if( robot.MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
								OLED_ShowNumber(20,40,- robot.MOTOR_D.Target*1000,5,12);
	else                 	    OLED_ShowString(15,40,"+"),
								OLED_ShowNumber(20,40,  robot.MOTOR_D.Target*1000,5,12); 
		
	if( robot.MOTOR_D.Encoder<0)OLED_ShowString(60,40,"-"),
								OLED_ShowNumber(75,40,-robot.MOTOR_D.Encoder*1000,5,12);
	else                 	    OLED_ShowString(60,40,"+"),
								OLED_ShowNumber(75,40, robot.MOTOR_D.Encoder*1000,5,12);
	
	//ȫ����û��D���,��ʾZ����ٶ�
	#else
	
	OLED_ShowString(0,40,"MOVE_Z"); 
	oled_showfloat(robot_control.Vz,60,40,3,2);
	
	#endif
}


#endif


