#include "WiredPS2_gamepad.h"
#include "FreeRTOS.h"
#include "task.h"

#include "system.h"

__weak void Wired_USB_PS2GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	printf("%d,%d\r\n",keyid,event);
}

static GamePadKeyStateType_t WiredPS2_GetKeyState(uint8_t bit);

//����ps2��Ϸ�ֱ�
GamePadType_t Wired_USB_PS2Gamepad = {
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.LT = 0,
	.RT = 0,
	.StartFlag = 0,
	.Vib_EN = 0,
	.SetVibration = NULL,
	.getKeyState = WiredPS2_GetKeyState
};

//����ֵ
static uint16_t GamePad_KeyOriginalVal = 0;

//��ȡ�ֱ���ֵ״̬����
static GamePadKeyStateType_t WiredPS2_GetKeyState(uint8_t bit)
{
	if( (GamePad_KeyOriginalVal>>bit)&0x01 )
		return GamePadKeyState_Press;
	else
		return GamePadKeyState_Release;
}


//���尴������¼��ĸ�������ֵ
static GamePad_CheckEventType_t GamePadKeyCheckEvent[16] = { 0 };


//��־λ���ú���,���ڸ���ps2�ֱ�����
static void ps2_set_bit(uint16_t* state,uint8_t state_bit,uint8_t bit)
{
	if(state_bit==1) //ָ����λ(bit)����Ϊ1,����λ����
	{
		*state |= (1U<<bit);
	}
	else //ָ����λ(bit)����Ϊ0,����λ����
	{
		*state &= ~(1U<<bit);
	}
}

//���ݽ���
void Wired_USB_PS2gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen)
{
	(void)phost;
	
	uint8_t tmp_bool = 0 ;
	
//	static uint8_t times = 0;
//	if( times<2 )
//	{
//		times++;
//		uint8_t vibration_data[2] = {1,1};
//		USBH_InterruptSendData(phost, vibration_data, 2, HID_Handle->OutPipe);
//	}
	
	Wired_USB_PS2Gamepad.LX = buffer[3];
	Wired_USB_PS2Gamepad.LY = 255-buffer[4];
	Wired_USB_PS2Gamepad.RX = buffer[1];
	Wired_USB_PS2Gamepad.RY = buffer[2];
	
	tmp_bool = (buffer[6]>>4)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,0); //seltec key ѡ�񰴼�
	
	tmp_bool = (buffer[6]>>6)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,1); //��ҡ�˰���
	
	tmp_bool = (buffer[6]>>7)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,2); //��ҡ�˰���
	
	tmp_bool = (buffer[6]>>5)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,3); //start
	
	tmp_bool = buffer[5]&0x0F;//ȡ����4λ
	if(tmp_bool==0x0F)//û���κΰ�������
	{
		ps2_set_bit(&GamePad_KeyOriginalVal,0,4); //��
		ps2_set_bit(&GamePad_KeyOriginalVal,0,5); //��
		ps2_set_bit(&GamePad_KeyOriginalVal,0,6); //��
		ps2_set_bit(&GamePad_KeyOriginalVal,0,7); //��
	}
	else if( (tmp_bool&0x01)==0 )
	{	
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4); //��
				break;
			case 0x01://��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //��
				break;
			case 0x02://��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //��
				break;
			case 0x03://��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //��
				break;
			default:
				break;
		}
	}
	else if( (tmp_bool&0x01)==1 ) //��λΪ1,�����������2���������µ����
	{
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://����
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4);//��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //��
				break;
			case 0x01://����
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //��
				break;
			case 0x02://����
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //��
				break;
			case 0x03://����
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4); //��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //��
				break;
			default:
				break;
		}
	}
	
	tmp_bool = (buffer[6]>>2)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,8); //����2��
	if( tmp_bool ) Wired_USB_PS2Gamepad.LT = 255;
	else Wired_USB_PS2Gamepad.LT = 0;
	
	tmp_bool = (buffer[6]>>3)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,9); //�Ұ��2��
	if( tmp_bool ) Wired_USB_PS2Gamepad.RT = 255;
	else Wired_USB_PS2Gamepad.RT = 0;
	
	tmp_bool = (buffer[6]>>0)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,10); //����1��
	
	tmp_bool = (buffer[6]>>1)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,11); //�Ұ��1��

	tmp_bool = (buffer[5]>>4)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,12); //һ��,��ɫGREEN
	
	tmp_bool = (buffer[5]>>5)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,13); //����,��ɫRED

	tmp_bool = (buffer[5]>>6)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,14); //����,����BLUE
	
	tmp_bool = (buffer[5]>>7)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,15); //�ĺ�,��ɫPINK
	
	//�����ص���������
	for (uint8_t key = PS2KEY_SELECT; key <= PS2KEY_4PINK; key++) 
	{
		GamePadKeyEventType_t event = GamePadKey_CheckEvent(GamePad_KeyOriginalVal,
                                		&GamePadKeyCheckEvent[key], key);
		
		if (event == GamePadKeyEvent_NONE) continue;
		
		//�����ص�����
		Wired_USB_PS2GamePad_KeyEvent_Callback(key,event);
	}

}

//���ݽ���
void Wired_USB_V2_PS2gamepad_Decode(USBH_HandleTypeDef *phost,uint8_t* buffer,uint8_t datalen)
{
	(void)phost;
	
	uint8_t tmp_bool = 0 ;
	
//	static uint8_t times = 0;
//	if( times<2 )
//	{
//		times++;
//		uint8_t vibration_data[2] = {1,1};
//		USBH_InterruptSendData(phost, vibration_data, 2, HID_Handle->OutPipe);
//	}
	
	Wired_USB_PS2Gamepad.LX = buffer[0];
	Wired_USB_PS2Gamepad.LY = 255-buffer[1];
	Wired_USB_PS2Gamepad.RX = buffer[3];
	Wired_USB_PS2Gamepad.RY = buffer[4];
	
	tmp_bool = (buffer[6]>>4)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,0); //seltec key ѡ�񰴼�
	
	tmp_bool = (buffer[6]>>6)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,1); //��ҡ�˰���
	
	tmp_bool = (buffer[6]>>7)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,2); //��ҡ�˰���
	
	tmp_bool = (buffer[6]>>5)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,3); //start
	
	tmp_bool = buffer[5]&0x0F;//ȡ����4λ
	if(tmp_bool==0x0F)//û���κΰ�������
	{
		ps2_set_bit(&GamePad_KeyOriginalVal,0,4); //��
		ps2_set_bit(&GamePad_KeyOriginalVal,0,5); //��
		ps2_set_bit(&GamePad_KeyOriginalVal,0,6); //��
		ps2_set_bit(&GamePad_KeyOriginalVal,0,7); //��
	}
	else if( (tmp_bool&0x01)==0 )
	{	
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4); //��
				break;
			case 0x01://��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //��
				break;
			case 0x02://��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //��
				break;
			case 0x03://��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //��
				break;
			default:
				break;
		}
	}
	else if( (tmp_bool&0x01)==1 ) //��λΪ1,�����������2���������µ����
	{
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://����
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4);//��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //��
				break;
			case 0x01://����
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,5); //��
				break;
			case 0x02://����
				ps2_set_bit(&GamePad_KeyOriginalVal,1,6); //��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //��
				break;
			case 0x03://����
				ps2_set_bit(&GamePad_KeyOriginalVal,1,4); //��
				ps2_set_bit(&GamePad_KeyOriginalVal,1,7); //��
				break;
			default:
				break;
		}
	}
	
	tmp_bool = (buffer[6]>>2)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,8); //����2��
	if( tmp_bool ) Wired_USB_PS2Gamepad.LT = 255;
	else Wired_USB_PS2Gamepad.LT = 0;
	
	tmp_bool = (buffer[6]>>3)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,9); //�Ұ��2��
	if( tmp_bool ) Wired_USB_PS2Gamepad.RT = 255;
	else Wired_USB_PS2Gamepad.RT = 0;
	
	tmp_bool = (buffer[6]>>0)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,10); //����1��
	
	tmp_bool = (buffer[6]>>1)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,11); //�Ұ��1��

	tmp_bool = (buffer[5]>>4)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,12); //һ��,��ɫGREEN
	
	tmp_bool = (buffer[5]>>5)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,13); //����,��ɫRED

	tmp_bool = (buffer[5]>>6)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,14); //����,����BLUE
	
	tmp_bool = (buffer[5]>>7)&0x01;
	ps2_set_bit(&GamePad_KeyOriginalVal,tmp_bool,15); //�ĺ�,��ɫPINK
	
	//�����ص���������
	for (uint8_t key = PS2KEY_SELECT; key <= PS2KEY_4PINK; key++) 
	{
		GamePadKeyEventType_t event = GamePadKey_CheckEvent(GamePad_KeyOriginalVal,
                                		&GamePadKeyCheckEvent[key], key);
		
		if (event == GamePadKeyEvent_NONE) continue;
		
		//�����ص�����
		Wired_USB_PS2GamePad_KeyEvent_Callback(key,event);
	}

}





