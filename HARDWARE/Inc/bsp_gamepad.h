#ifndef __BSP_GAMEPAD_H
#define __BSP_GAMEPAD_H

#include <stdint.h>

//��Ϸ�ֱ�����ͨ���¼�
typedef enum{
    GamePadKeyEvent_NONE,         //�ް����¼�
    GamePadKeyEvent_SINGLECLICK,  //�����¼�
    GamePadKeyEvent_DOUBLECLICK,  //˫���¼�
    GamePadKeyEvent_LONGCLICK     //�����¼�
}GamePadKeyEventType_t;

//��Ϸ�ֱ�ͨ�ð���״̬
typedef enum{
	GamePadKeyState_Release = 0,  //�ɿ�
	GamePadKeyState_Press = 1     //����
}GamePadKeyStateType_t;

//ͨ���ֱ��������״̬��
typedef enum{
    WaitToPress = 0,
    WaitToRelease ,
    KEYPress    ,
    KEYUp       ,
    LONG_CLICK  ,
}GamePad_StateMachineType_t;

//ͨ���ֱ�������������
typedef struct 
{
    uint8_t keystate;
    uint32_t timebase;
    uint32_t statetime;
    GamePad_StateMachineType_t statemachine;
}GamePad_CheckEventType_t;

//��Ϸ�ֱ�����ͨ�ü��ʱ��,��λms
#define GamePad_LONGPRESS_TIEM 1000 //�������ʱ��
#define GamePad_CLICK_TIME     400  //��˫�����ʱ��
#define GamePad_KEYFILTER_TIME 50   //�����˲�ʱ��

//ͨ����Ϸ�ֱ�����
typedef struct{
	uint8_t LX;  //4������ҡ��ֵ,ȡֵ0~255
	uint8_t LY;
	uint8_t RX;
	uint8_t RY;
	uint8_t LT;  //�����������Ű��
	uint8_t RT;
	void (*SetVibration)(uint8_t m1,uint8_t m2);//�𶯽ӿ�
	GamePadKeyStateType_t (*getKeyState)(uint8_t keybit);
	uint8_t StartFlag;
	uint8_t Vib_EN;
}GamePadType_t;


//��Ϸ�ֱ�Ʒ��
typedef enum{
	UnKnown_Dev,          //δ֪�豸
	PS2_Classic,           //����PS2�ֱ�
	PS2_USB_Wired,         //����ps2�ֱ�
	PS2_USB_WiredV2,       //�ڶ�������ps2
	PS2_USB_Wiredless,    //����ps2�ֱ�
	Xbox360,    //xbox360��ʶ��
	SwitchPro,
}GamePadBrandType_t;

//�ֱ����Խṹ��
enum{
	EnumNULL = 0,
	EnumWait ,
	EnumDone ,
};
typedef struct{
	uint8_t ready;
	uint8_t enmu_state;
	GamePadBrandType_t type;
}GamePadDebugValType_t;

//����Ϸ�ֱ��ṩ�������Ľӿں���
GamePadKeyEventType_t GamePadKey_CheckEvent(uint16_t keysource,GamePad_CheckEventType_t* key,uint8_t bit);

//�����ṩ��Ϸ�ֱ�Ʒ�Ʋ���
extern GamePadBrandType_t gamepad_brand;

//�ֱ��豸������γ��ص�����
void USB_GamePad_InsertCallback(void);
void USB_GamePad_PullOutCallback(void);

//ͨ����Ϸ�ֱ��ӿ�
extern GamePadType_t* GamePadInterface;
extern GamePadType_t GamePadDefalut;

//ͨ���ֱ����Խӿ�
extern GamePadDebugValType_t GamePadDebug;

#endif /* __BSP_GAMEPAD_H */
