#include "bsp_gamepad.h"

#include "FreeRTOS.h"
#include "task.h"


GamePadDebugValType_t GamePadDebug = { 0 };

//������Ϸ�ֱ�Ʒ��
GamePadBrandType_t gamepad_brand = UnKnown_Dev;

extern void OLED_ShowGamePadState(void);

//�ֱ��豸���뺯��_���񻷾�
void USB_GamePad_InsertCallback(void)
{
	OLED_ShowGamePadState();
	GamePadDebug.enmu_state = EnumWait;
//	vTaskDelay(pdMS_TO_TICKS(1000)); //�ӳ�1000ms����OLED��ʾ
	GamePadDebug.enmu_state = EnumDone;
	GamePadDebug.type = gamepad_brand;
}

//�ֱ��豸�γ�_���񻷾�
void USB_GamePad_PullOutCallback(void)
{
	//�궨Ϊδ֪�豸
	gamepad_brand = UnKnown_Dev;
	
	//����ǰ���ֱ�ҡ��ֵ��λ,��ֹ�´�ʶ��ʱ��ֵ
	GamePadInterface->LX = 127;
	GamePadInterface->RX = 127;
	GamePadInterface->LY = 127;
	GamePadInterface->RY = 127;
	GamePadInterface->LT = 0;
	GamePadInterface->RT = 0;
	
	//���л���Ĭ�Ͻӿڷ�ֹ���ֿ�����
	GamePadInterface = &GamePadDefalut;
	
	//�������
	GamePadDebug.enmu_state = EnumNULL;
	GamePadDebug.type = gamepad_brand;
	GamePadDebug.ready = 0;
}

//ͨ����Ϸ�ֱ�����
GamePadType_t* GamePadInterface = 0;

GamePadType_t GamePadDefalut = {
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.LT = 0,
	.RT = 0,
	.StartFlag = 0,
	.Vib_EN = 1,
	.SetVibration = 0,
	.getKeyState = 0
};

//��ڲ���������ԭʼֵ���������ԣ���Ҫ���İ���
GamePadKeyEventType_t GamePadKey_CheckEvent(uint16_t keysource,GamePad_CheckEventType_t* key,uint8_t bit)
{
    //��ȡ��Ӧ�ļ�ֵ״̬
    key->keystate = (keysource>>bit)&0x01;

    switch (key->statemachine)
    {
        case WaitToPress:
            if( GamePadKeyState_Press == key->keystate )
            {
                key->timebase = xTaskGetTickCount();
                key->statemachine = KEYPress;
            } 
            break;
        case KEYPress:
            //ͳ�Ƶ�һ�ΰ��°����Ժ��ʱ��(�޷������ʱ��Ȼ����.����Ҫ����ж�)
            key->statetime = xTaskGetTickCount() - key->timebase;

            //��鰴���Ƿ����ɿ�
            if( GamePadKeyState_Release == key->keystate )
            {
                //���ΰ������µ�ʱ��̫��,����.��Ϊ�˲�����
                if( key->statetime < GamePad_KEYFILTER_TIME ) key->statemachine = WaitToPress;
                else
                {
                   key->timebase = xTaskGetTickCount();//���¸���ʱ��,������һ��״̬�ļ��
                   key->statemachine = KEYUp;    //��������һ��ʱ��,�ֵ���,������һ�����״̬
                }
            }
            else if( key->statetime > GamePad_LONGPRESS_TIEM ) 
            {   //����δ�ɿ�,�ұ���һ����ʱ��,��Ϊ����.
                key->statemachine = LONG_CLICK;
            }

            break;
        case KEYUp:
            //ͳ�Ƶ�һ�ΰ��°����Ժ��ʱ��(�޷������ʱ��Ȼ����.����Ҫ����ж�)
            key->statetime = xTaskGetTickCount() - key->timebase;

            if( GamePadKeyState_Press == key->keystate && key->statetime < GamePad_CLICK_TIME && key->statetime > GamePad_KEYFILTER_TIME )
            {
                key->statemachine = WaitToRelease;
                return GamePadKeyEvent_DOUBLECLICK;
            }
            else if( key->statetime >= GamePad_CLICK_TIME )
            {
                key->statemachine = WaitToRelease;
                return GamePadKeyEvent_SINGLECLICK;
            }
            break;
        case LONG_CLICK:
            key->statemachine = WaitToRelease;
            return GamePadKeyEvent_LONGCLICK;
        case WaitToRelease:
            //����������,���û��ɿ�������,�ٻָ�״̬����״̬
            if( GamePadKeyState_Release == key->keystate ) key->statemachine = WaitToPress;
            break;
        default:
            break;
    }

    return GamePadKeyEvent_NONE;
}

