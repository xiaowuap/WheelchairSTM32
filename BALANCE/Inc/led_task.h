#ifndef __LED_TASK_H
#define __LED_TASK_H

#include "system.h"

#define LED_TASK_PRIO		2     //Task priority //�������ȼ�
#define LED_STK_SIZE 		128   //Task stack size //�����ջ��С

void led_task(void *pvParameters);

#endif

