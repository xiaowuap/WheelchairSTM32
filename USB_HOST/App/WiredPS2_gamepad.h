#ifndef __WiredPS2_GAMEPAD_H
#define __WiredPS2_GAMEPAD_H

#include <stdint.h>

#include "usbh_hid.h"
#include "usbh_hid_GamePad.h"
#include "bsp_gamepad.h"

//�����ֱ���Ʒ��ʶ��
#define Wired_PS2_VID 0x0810
#define Wired_PS2_PID 0x0001

//�ڶ�������
#define WiredV2_PS2_VID 0x0079
#define WiredV2_PS2_PID 0x0006

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


//�ֱ����뺯��
void Wired_USB_PS2gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen);
void Wired_USB_V2_PS2gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen);
//�����ṩ�ֱ�����
extern GamePadType_t Wired_USB_PS2Gamepad;

#endif /* __Xbox360_GAMEPAD_H */
