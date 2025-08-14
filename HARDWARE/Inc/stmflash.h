#ifndef __STMFLASH_H
#define __STMFLASH_H

#include "sys.h"

//#define use_usart_debug 

#ifdef use_usart_debug

/*
__FILE__       //��ǰ����Դ�ļ�������char*
__FUNCCTIONN__ //��ǰ���еĺ���������char*
__LINE__       //��ǰ�ĺ�����,����int
__VA_ARGS__    //���������Ŀɱ��������...���ʹ��

#  �� ��#���������ת�����ַ���
## �� ���Ӳ�������test##i  ���ݴ����i��ͬ��ʵ���ַ���test���ַ���i������
      ���⣬�ڿɱ�����ĺ����棬##������ȥǰ��һ�����ŵ�����
args... :����ɱ�����б�

*/

#define DEBUG_INFO(fmt,...) \
	any_printf(USART1,"[File:%s][Function:%s][Line:%d]" fmt,__FILE__,__FUNCTION__,__LINE__,##__VA_ARGS__)
#else
#define DEBUG_INFO(fmt,...) __nop()
#endif


#define OTP_ADDR_TEST 0x1FFF7800 //OTP����� 0 ���׵�ַ
#define OTP_LOCK_ADDR 0x1FFF7A00 //OTP�������������õ��׵�ַ

//Flash��̱���ö������
enum{
	FLASH_EOP    = ( 1 << 0 ) , //Flash ���/���� ������ɣ���λֻ�������ò��������ж�ʹ��ʱ(EOPIE=1)�Ż���Ч��
	FLASH_OPERR  = ( 1 << 1 ) , //Flash�������󣺴���Flash�������󣬵����ڲ���λ�����󡢶�������д�� ����λֻ����ʹ�ܲ��������ж�ʱ(ERRIE=1)�Ż���Ч��
	FLASH_WRPERR = ( 1 << 4 ) , //Ҫ����/��̵ĵ�ַ���� Flash �д���д����״̬������
	FLASH_PGAERR = ( 1 << 5 ) , //��̶������Flash��128λ��ģ�д������ݿ�Խ��������
	FLASH_PGPERR = ( 1 << 6 ) , //��̲���λ����PSIZE������ �� д��Flash�����ݿ�Ȳ�ƥ��
	FLASH_PGSERR = ( 1 << 7 ) , //���˳����� �� ���ƼĴ���ʱ����˳�򲻶�
	FLASH_BSY    = ( 1 << 16) , //��ǰ����Flash�������ڽ���
};
#define Get_FlashState(res,mask) ( res & mask) //��ȡFlashö�ٱ�������

//Flash����ö��
typedef enum
{
	flash_no_error = 0 , //Flash�޴���
	flash_busy ,     //Flash����æ
	flash_more_error,//Flash�������ڶ������
	flash_eop,       //Flash�������
	flash_operr,     //Flash��������
	flash_wrperr,    //Flashд��������
	flash_pgperr,    //Flash���д���
	flash_pgserr,    //Flash�������
	flash_overtime,   //Flash�ȴ���ʱ
	flash_AddrErr,   //��Ҫ������Flash��ַ�Ƿ�
	flash_AddrWillout,//Flash������ĩβ�ᳬ��������Χ
	flash_sector_out, //Ҫ���õ���������оƬ��ӵ�е�����
}FlashState;


//Flash����ַ
#define STM32_FLASH_BASE    0x08000000

//��ʹ�õ�оƬFlash��������λ kB
#define STM32_FLASH_SIZE 512

//Flashĩβ��ַ
#define FLASH_END_ADDR  STM32_FLASH_BASE + STM32_FLASH_SIZE*1024 - 1

//RAM����ַ
#define STM32_RAM_BASE 0x20000000
#define STM32_RAM_SIZE 192
#define RAM_END_ADDR  STM32_RAM_BASE + STM32_RAM_SIZE*1024 - 1

/* FLASH ��������ʼ��ַ */
#define ADDR_FLASH_SECTOR_0     ((uint32_t )0x08000000)     /* ����0��ʼ��ַ, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t )0x08004000)     /* ����1��ʼ��ַ, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t )0x08008000)     /* ����2��ʼ��ַ, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t )0x0800C000)     /* ����3��ʼ��ַ, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t )0x08010000)     /* ����4��ʼ��ַ, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t )0x08020000)     /* ����5��ʼ��ַ, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t )0x08040000)     /* ����6��ʼ��ַ, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t )0x08060000)     /* ����7��ʼ��ַ, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t )0x08080000)     /* ����8��ʼ��ַ, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t )0x080A0000)     /* ����9��ʼ��ַ, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t )0x080C0000)     /* ����10��ʼ��ַ,128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t )0x080E0000)     /* ����11��ʼ��ַ,128 Kbytes */


//����ӿ�
uint8_t Write_Flash(uint32_t* data,uint16_t datalen);
int Read_Flash(uint16_t index);

#endif

