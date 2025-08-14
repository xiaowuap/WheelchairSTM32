#ifndef __AUTORECHARGE_H
#define __AUTORECHARGE_H
#include "sys.h"

typedef struct{
	u8 L_A;          //��·�����ź����. 1�յ��ź� 0���ź�
	u8 L_B;
	u8 R_A;
	u8 R_B;
	u8 RED_STATE;    //��·�����źŻ��� ȡֵ0~4
	u8 NavWalk;      //�����ӹܻس���Ƶı�־λ 1�ѽӹ� 0��״̬
	u8 AllowRecharge;//�Զ��س�ģʽ��־λ 1�س�ģʽ 0����ģʽ
	u8 Charging;     //����־λ 1��� 0δ���
	
	u8 RedRefalsh_Time; //�����ź�ʶ��ʱˢ�µ�ʱ�䣬��λ 10ms.
	
	float ChargingCurrent;//��������С
	
	float Dock_MoveX; //���öԽ�ʱ�ٶȵĴ�С,��ֱֵ�Ӹ�ֵ��Red_Move
	float Dock_MoveY;
	float Dock_MoveZ;
	
	float Red_MoveX; //ʶ�𵽺����źŸ�����С��3���ٶ�
	float Red_MoveY;
	float Red_MoveZ;
	
	float Up_MoveX;//�����ӹ�ʱ,������С��3���ٶ�
	float Up_MoveY;
	float Up_MoveZ;
	
	u8 OutLine_Check;//���װ�����߼��
	
}CHARGER_t;


//����ӿ�
extern CHARGER_t charger;
void auto_recharge_reset(void);
void CAN_Send_AutoRecharge(void);
void Find_Charging_HardWare(void);

//�Զ��س�Ĵ����߼�����
void Handle_Normal_AutoRecharge(uint8_t* temp_rxbuf);
void Handle_AKM_AutoRecharge(uint8_t* temp_rxbuf);

#if defined AKM_CAR
//�����ǰ�����С���Զ��س�ר��.�ڰ�����ʹ���Զ��س�ʱ,�ٶ�ִ����Դ������,�������ó��װ�����ٶ��߼�.\
(ԭ���ǰ������Խ��߼���ͬ,���װ���ϵ��߼���Ҫ��������С��,����ֻ���㰢����.�ʰ����������ڿ��ư��)
#define cur_front_left do{                          \
			charger.Red_MoveX = -charger.Dock_MoveX ,\
			charger.Red_MoveZ =  0.1f ;\
}while(0)

#define cur_front_right do{                          \
			charger.Red_MoveX = -charger.Dock_MoveX ,\
			charger.Red_MoveZ = -0.1f ;\
}while(0)


#define front_left do{                          \
			charger.Red_MoveX = -charger.Dock_MoveX ,\
			charger.Red_MoveZ =  charger.Dock_MoveZ ;\
}while(0)

#define front_right do{                          \
			charger.Red_MoveX = -charger.Dock_MoveX ,\
			charger.Red_MoveZ = -charger.Dock_MoveZ ;\
}while(0)

#define back_left do{                          \
			charger.Red_MoveX = charger.Dock_MoveX ,\
			charger.Red_MoveZ = -charger.Dock_MoveZ ;\
}while(0)

#define back_right do{                          \
			charger.Red_MoveX = charger.Dock_MoveX ,\
			charger.Red_MoveZ = charger.Dock_MoveZ ;\
}while(0)

#define back do{                          \
			charger.Red_MoveX = charger.Dock_MoveX ,\
			charger.Red_MoveZ = 0 ;\
}while(0)

#define front do{                          \
			charger.Red_MoveX = -charger.Dock_MoveX ,\
			charger.Red_MoveZ = 0 ;\
}while(0)

#define stop do{                          \
			charger.Red_MoveX = 0 ,\
			charger.Red_MoveZ = 0 ;\
}while(0)
#endif //#if defined AKM_CAR


#endif
