#ifndef __UARTX_CALLBACK_H
#define __UARTX_CALLBACK_H 

#include "system.h"

//机器人接收控制命令的数据长度
#define RECEIVE_DATA_SIZE 11

//机器人接收控制命令的结构体
typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1个字节
		float X_speed;	            //4 bytes //4个字节
		float Y_speed;              //4 bytes //4个字节
		float Z_speed;              //4 bytes //4个字节
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}Control_Str;
}RECEIVE_DATA;

//内部函数
static float XYZ_Target_Speed_transition(u8 High,u8 Low);
static u8 AT_Command_Capture(u8 uart_recv);
static void _System_Reset_(u8 uart_recv);

#endif

