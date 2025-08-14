#ifndef __PS2_GAMEPAD_H
#define __PS2_GAMEPAD_H

#include <stdint.h>

#include "bsp_gamepad.h"

//PS2����λ��ö��(bit0~bit15�ֱ�Ϊ�����0~15)
enum 
{
	PS2KEY_SELECT	   = 0, //ѡ�񰴼�
	PS2KEY_LROCKER      , //����ҡ�˰��¼�ֵ
	PS2KEY_RROCKER      ,
	PS2KEY_START        , //��ʼ����
	PS2KEY_UP           , //�󰴼�����
	PS2KEY_RIGHT        ,
	PS2KEY_DOWN         ,
	PS2KEY_LEFT         ,
	PS2KEY_L2           ,	//���Ұ������ֵ
	PS2KEY_R2           ,
	PS2KEY_L1           ,  
	PS2KEY_R1           ,
	PS2KEY_1GREEN       , //�Ұ�������
	PS2KEY_2RED         , 
	PS2KEY_3BLUE        , 
	PS2KEY_4PINK           
};

//���ݽ��뺯��
void PS2_Wired_Decode(const uint8_t *data,uint8_t datalen);
void PS2_Wiredless_Android_Decode(const uint8_t *data,uint8_t datalen);
void PS2_Wiredless_PC_Decode(const uint8_t *data,uint8_t datalen);

//�����ṩ�Ľӿ�
extern GamePadType_t ps2_gamepad;

#endif /* __PS2_GAMEPAD_H */

