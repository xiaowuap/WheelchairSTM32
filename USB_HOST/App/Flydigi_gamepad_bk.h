#ifndef __FLYDIGI_GAMEPAD_H
#define __FLYDIGI_GAMEPAD_H

#include <stdint.h>

#include "usbh_hid.h"
#include "usbh_hid_GamePad.h"
#include "bsp_gamepad.h"

//�ֱ���ƷID�Լ����к�
#define FLYDIGI_Manufacturer "Flydigi"
#define FLYDIGI_SerialNum "5E904C0C"

//�ֱ���ƷPID��VID
#define FLYDIGI_GamePad_VID 0x045E
#define FLYDIGI_GamePad_PID 0x028E

//�ֱ������ݳ�����Ϣ
#define FLYDIGI_GamePad_DataLen 64

//15���������
enum{
	FlydigiKEY_UP = 0, //ת��ʮ��-��
	FlydigiKEY_DOWN,   //ת��ʮ��-��
	FlydigiKEY_LEFT,   //ת��ʮ��-��
	FlydigiKEY_RIGHT,  //ת��ʮ��-��
	FlydigiKEY_Menu,   // �˵�/star����
	FlydigiKEY_SELECT, // ����/select����
	FlydigiKEY_LJoy,   //��ҡ�˰���
	FlydigiKEY_RJoy,   //��ҡ�˰���
	FlydigiKEY_LB ,    //���ϰ���LB
	FlydigiKEY_RB ,    //���ϰ���RB
	FlydigiKEY_HOME ,  //logo����Home
	Flydigi_PaddingBit,//���λ,Ϊ�˶��밴������ʹ��,ʵ�ʸ�λ������
	FlydigiKEY_A ,     //����A
	FlydigiKEY_B ,     //����B
	FlydigiKEY_X ,     //����X
	FlydigiKEY_Y ,     //����Y
	//�������Ԥ��
};

//�ֱ����뺯��
void Flydigi_gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen);

//�����ṩ�ֱ�����
extern GamePadType_t flydigiGamepad;

#endif /* __FLYDIGI_GAMEPAD_H */
