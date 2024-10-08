#include "led.h"

int Led_Count=500; //LED flicker time control //LED闪烁时间控制

/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin =  LED_R_PIN|LED_G_PIN|LED_B_PIN;//LED对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO
	GPIO_SetBits(GPIOD,LED_R_PIN);
	GPIO_SetBits(GPIOD,LED_G_PIN);
	GPIO_SetBits(GPIOD,LED_B_PIN);
}
/**************************************************************************
Function: Buzzer interface initialized
Input   : none
Output  : none
函数功能：蜂鸣器接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void Buzzer_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin =  Buzzer_PIN;//LED对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO
}
/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
函数功能：LED灯闪烁任务
入口参数：无 
返回  值：无
**************************************************************************/
void led_task(void *pvParameters)
{
    while(1)
    {
		static u8 led_state=0;
		
		led_state = !led_state;
		
		if( Allow_Recharge )
		{
			if( Charging )LED_Purple(led_state);
			else LED_Yellow(led_state);
		}
		else
			LED_Red(led_state);
			
		vTaskDelay(Led_Count); 
    }
}  

/**************************************************************************
Function: The LED flashing
Input   : none
Output  : blink time
函数功能：LED闪烁
入口参数：闪烁时间
返 回 值：无
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if(0==time) LED_R=0;
	  else		if(++temp==time)	LED_R=~LED_R,temp=0;
}

