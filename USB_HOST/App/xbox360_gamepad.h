#ifndef __XBOX360_GAMEPAD_H
#define __XBOX360_GAMEPAD_H

#include <stdint.h>

#include "usbh_hid.h"
#include "usbh_hid_GamePad.h"
#include "bsp_gamepad.h"

//�ֱ���ƷID�Լ����к�
#define Xbox360_Manufacturer "Xbox360"
#define Xbox360_SerialNum "5E904C0C"

//�ֱ���ƷPID��VID
#define Xbox360_GamePad_VID 0x045E
#define Xbox360_GamePad_PID 0x028E

//�ֱ������ݳ�����Ϣ
#define Xbox360_GamePad_DataLen 64

//15���������
enum{
	Xbox360KEY_UP = 0, //ת��ʮ��-��
	Xbox360KEY_DOWN,   //ת��ʮ��-��
	Xbox360KEY_LEFT,   //ת��ʮ��-��
	Xbox360KEY_RIGHT,  //ת��ʮ��-��
	Xbox360KEY_Menu,   // �˵�/star����
	Xbox360KEY_SELECT, // ����/select����
	Xbox360KEY_LJoy,   //��ҡ�˰���
	Xbox360KEY_RJoy,   //��ҡ�˰���
	Xbox360KEY_LB ,    //���ϰ���LB
	Xbox360KEY_RB ,    //���ϰ���RB
	Xbox360KEY_HOME ,  //logo����Home
	Xbox360_PaddingBit,//���λ,Ϊ�˶��밴������ʹ��,ʵ�ʸ�λ������
	Xbox360KEY_A ,     //����A
	Xbox360KEY_B ,     //����B
	Xbox360KEY_X ,     //����X
	Xbox360KEY_Y ,     //����Y
	//�������Ԥ��
};

//�ֱ����뺯��
void Xbox360_gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen);

//�����ṩ�ֱ�����
extern GamePadType_t Xbox360Gamepad;

#endif /* __Xbox360_GAMEPAD_H */
