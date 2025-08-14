#include "stmflash.h"
#include "delay.h"

//F407VE ��512kb FLash. �����ݱ��������һ������7. 0x8000000+(512-128)*1024 = 0x806000
#define FLASH_SAVE_ADDR 0x08060000

//���洢���Ľ�����ֵ
#define STM32_FLASH_KEY1    ((uint32_t)0x45670123)
#define STM32_FLASH_KEY2    ((uint32_t)0xCDEF89AB)

//OPT�û�ѡ���ֽڵĽ�����ֵ
#define STM32_FLASH_OPTKEY1 ((uint32_t)0x08192A3B)
#define STM32_FLASH_OPTKEY2 ((uint32_t)0x4C5D6E7F)

//�������洢��Flash
void stmflash_unlock(void)
{
	//ע�������LOCKλΪ0��������ٴ�д��key,������Ӳ�����󣬵�Ƭ������
	if( (FLASH->CR & (uint32_t)1<<31) != 0)
	{
		FLASH->KEYR = STM32_FLASH_KEY1;
		FLASH->KEYR = STM32_FLASH_KEY2;
	}
	else
	{
		DEBUG_INFO("LOCKλ��ǰ��0���������\r\n");
	}
}

//����OPT�û�ѡ���ֽ���Flash
static void stmflash_opt_unlock(void)
{
	if( (FLASH->OPTCR & (uint32_t)1<<0)!=0 )
	{
		/* FLASH д��������� */
		FLASH->OPTKEYR = STM32_FLASH_OPTKEY1;     
		FLASH->OPTKEYR = STM32_FLASH_OPTKEY2;
	}
	else
	{
		DEBUG_INFO("OPT LOCKλ��ǰ��0���������");
	}
}

//���洢��Flash����
static void stmflash_lock(void)
{
	/* FLASH ���� */
    FLASH->CR |= (uint32_t)1 << 31;     
}

//ѡ���ֽ�OPT��Flash����
static void stmflash_opt_lock(void)
{
	/* FLASH ���� */
    FLASH->OPTCR |= (uint32_t)1 << 0;     
}

//Flash�������ã���Ҫ�����Լ���ʱ��Ƶ�ʺ�Ӳ���������Ӧ����
void stmflash_init(void)
{	
	DEBUG_INFO("1.д��key����\r\n");
	stmflash_unlock();
	
	delay_us(100);
	
	//����Flash�ĵȴ�����(LATENCY)
	//��оƬ�����ѹ��оƬCPUʱ��Ƶ�������������
	//��оƬ�����ѹ3.3V,CPUƵ��168M,������Ϊ5 WS
	DEBUG_INFO("2.ACR���ã���LATENCY����Ϊ5ws\r\n");
	FLASH->ACR &= ~(7 << 0); //���ԭ��������
	FLASH->ACR |= 5 << 0;    //����LATENCYΪ5 ws
	
	delay_us(100);
	//����Flash������λPSIZE
	//��оƬ�����ѹ��أ���ѹ��2.7~3.6Vʱ����Ҫ���ò���λ��Ϊ32
	
	DEBUG_INFO("3.CR���ã�����PSIZE���Ϊ32λ\r\n");
	FLASH->CR &= ~(3 << 8); //���ԭ��������
	FLASH->CR |= 2<<8;      //����PSIZE���Ϊ32λ
	
	DEBUG_INFO("4.��ʼ����ϣ�����\r\n");
	stmflash_lock();
	
	/*
	 * ͨ������ACR�Ĵ���Ĭ��ֵ��F4оƬ�ϵ�ART������ ��Ԥȡʹ�ܡ�
	 * ��ָ���ʹ�ܡ� �����ݻ���ʹ�� ��Ĭ�ϴ򿪵�
	*/
}

//ָ����ַ��ȡflash����
uint32_t stmflash_read_word(uint32_t faddr)
{
	return  *(volatile uint32_t *)faddr;
}

//ָ����ַָ�����ȶ�ȡflash����
void read_flash(uint32_t addr,uint32_t* p,u8 len)
{
	u8 i=0;
	for(i=0;i<len;i++)
	{
		p[i] = stmflash_read_word(addr);
		addr += 4;
	}
}

//��ȡFlash�Ķ�д״̬
static FlashState stmflash_get_state(void)
{
    uint32_t res = 0;
	uint8_t error_count = 0;//�������ͳ��
	
	//Flash״̬���ȱ���޴���
	FlashState state = flash_no_error ;

	res = FLASH->SR; //��ȡSR�Ĵ�����ȡ״̬
	
	//��ȡFlash��״̬�����
	if(Get_FlashState(res,FLASH_WRPERR)) 
	{
		error_count++,state = flash_wrperr;
		DEBUG_INFO("Ҫд��ĵ�ַ����д����״̬\r\n");
	}
	if(Get_FlashState(res,FLASH_PGPERR)) 
	{
		error_count++,state = flash_pgperr;
		DEBUG_INFO("����Flash��̲���λ����\r\n");
	}
	if(Get_FlashState(res,FLASH_PGSERR)) 
	{
		error_count++,state = flash_pgserr;
		DEBUG_INFO("����Flash���˳�����\r\n");
	}
	if(Get_FlashState(res,FLASH_BSY)   )
	{
		error_count++,state = flash_busy;
		DEBUG_INFO("Flash���ߵȴ���...\r\n");
	}
	
	/* EOP��OPERR�ڿ����ж�ʱ�Ż���Ч������Ĭ�ϲ�ʹ�� */
	//if(Get_FlashState(res,FLASH_EOP)   ) error_count++,state = flash_eop;
	//if(Get_FlashState(res,FLASH_OPERR) ) error_count++,state = flash_operr;
	//��Ǵ��ڶ������
	if(error_count>1)
	{
		state  = flash_more_error;
		DEBUG_INFO("Flash�����ϴ��ڶ������...\r\n");
	}
    
    return state;
}

//�ȴ�Flash��ɲ���
static FlashState stmflash_wait_done(uint32_t time)
{
    FlashState state;
    do
    {
        state = stmflash_get_state();//��ȡFlash״̬

        if (state != flash_busy)
        {
            break;      /* ��æ, ����ȴ���, ֱ���˳� */
        }
        
        time--;
    } while (time);

    if (time == 0) 
	{
		DEBUG_INFO("Flash���ߵȴ���ʱ\r\n");
		state = flash_overtime;   /* ��ʱ */
	}
	
	//����״̬���
    return state;
}

//������������
//ֻ�����洢����Ҫ������OTP���޷�������ѡ���ֽ������Զ�����
FlashState stmflash_erase_sector(uint8_t saddr)
{
    FlashState res;

    res = stmflash_wait_done(0XFFFFFFFF);   /* �ȴ��ϴβ������� */
	
    if (res == flash_no_error)
    {
		stmflash_unlock();
        FLASH->CR &= ~(0X1F << 3);           /* ���ԭ������������ */
        FLASH->CR |= saddr << 3;             /* ����Ҫ���������� */
		
		FLASH->CR &= ~(7 << 0);              /* ���ԭ��ѡ���̡�����������Ƭ���������� */
        FLASH->CR |= 1 << 1;                 /* ����Ϊ�������� */
		
        FLASH->CR |= 1 << 16;                /* ��ʼ���� */
        res = stmflash_wait_done(0XFFFFFFFF);/* �ȴ��������� */
		stmflash_lock();
		//ע��STRTλ���Զ���0(λ16)�� SERλ����(λ1),���� SERλ����ǰ���֮ǰ�����ü���             
    }
    return res;
}


//��FLASHָ��λ��д��һ����
//��ڲ�������ַ��Ҫд�������
//��ַ������4�ı�������ΪPSIZEҪ���ݵ�ѹ�����ã�Ӳ����ѹ��3.3V������PSIZE����Ϊ32λ \
  Flash��128λ��ģ���ַ����4�ı���ʱ�����ᷢ����̶������
static FlashState stmflash_write_word(uint32_t faddr, uint32_t data)
{
    FlashState res;

    res = stmflash_wait_done(0XFFFFF);

    if (res == flash_no_error)
    {
		FLASH->CR &= ~(7 << 0);             /* ���ԭ��ѡ���̡�����������Ƭ���������� */
        FLASH->CR |= 1 << 0;                /* ���ʹ�� */
        *(volatile uint32_t *)faddr = data; /* д������ */
        res = stmflash_wait_done(0XFFFFF);  /* �ȴ��������,һ���ֱ�� */
		//ע�⣺���ʱ��STRTλ�޹أ����λ�ǲ���ʱ�õ�
    }

    return res;
}


/**
��ȡĳ����ַ���ڵ�flash����
faddr   : flash��ַ
0~11, ��addr���ڵ�����
*/
static uint8_t stmflash_get_flash_sector(uint32_t addr)
{
    if (addr < ADDR_FLASH_SECTOR_1)return 0;
    else if (addr < ADDR_FLASH_SECTOR_2)return 1;
    else if (addr < ADDR_FLASH_SECTOR_3)return 2;
    else if (addr < ADDR_FLASH_SECTOR_4)return 3;
    else if (addr < ADDR_FLASH_SECTOR_5)return 4;
    else if (addr < ADDR_FLASH_SECTOR_6)return 5;
    else if (addr < ADDR_FLASH_SECTOR_7)return 6;
    else if (addr < ADDR_FLASH_SECTOR_8)return 7;
    else if (addr < ADDR_FLASH_SECTOR_9)return 8;
    else if (addr < ADDR_FLASH_SECTOR_10)return 9;
    else if (addr < ADDR_FLASH_SECTOR_11)return 10;

    return 11;
}


//ָ��λ�ã�д��ָ�����ȵ�32λ����
//��ڲ�������ַ��Ҫд������ݡ�Ҫд�����������
//����д����������洢����OTP����һ���Ա������
FlashState stmflash_write(uint32_t waddr, uint32_t *pbuf, uint32_t length)
{

    FlashState status = flash_no_error;
    uint32_t endaddr = 0;
	
	/* �Ƿ���ַ��
	 * �ٴ��ڱ�оƬ���Flash������ַ
	 * ��С�ڴ洢���׵�ַ
	 * �۵�ַ����4�ı��� 
	 * ��ע�⣬��Щ��ַ�����Ƚ��ϸ񣬲�֧��OTP�ĵ�ַ�����Ҫд��OTP���򣬿���ע�͵�ַ����
	*/
    if (waddr < STM32_FLASH_BASE ||  waddr > FLASH_END_ADDR ||  waddr % 4 ) 
    {
		status = flash_AddrErr;//�Ƿ���ַ
		DEBUG_INFO("д��ĵ�ַ�Ƿ�\r\n");
        return status;
    }
	
    endaddr = waddr + length*4;
	if(endaddr>FLASH_END_ADDR) //д������ݻᳬ���ڴ������������Ϸ�
	{
		DEBUG_INFO("д�������δ���ᳬ��flash������\r\n");
		status = flash_AddrWillout;
		return status;
	}
    
    
	stmflash_unlock();       //Flash����
    FLASH->ACR &= ~(1 << 9); //��ָֹ���
	FLASH->ACR &= ~(1 << 10);//��ֹ���ݻ���
	FLASH->ACR |= 1 << 11;   //��λ(���)ָ���
	FLASH->ACR |= 1 << 12;   //��λ(���)���ݻ���
	__disable_irq();         //�ر������жϣ���֤Flashд������
	
	//�ȴ���һ�β������
	status = stmflash_wait_done(0xffff);
	
	//Flash״̬�޴��󣬿�ʼд������
	if(status == flash_no_error)
	{
		DEBUG_INFO("��ʼ�� 0x%x ��д������\r\n",waddr);
		while(waddr < endaddr)
		{
			status = stmflash_write_word(waddr , *pbuf);
			if(status!=flash_no_error) break;
			
			waddr += 4;
			pbuf ++ ;
		}	
		
	}

	FLASH->ACR |= 1<<9;    //ʹ��ָ���
	FLASH->ACR |= 1<<10;   //ʹ�����ݻ���
	FLASH->ACR &= ~(1<<11);//���ָ��渴λ��־λ
	FLASH->ACR &= ~(1<<12);//������ݻ��渴λ��־λ
	stmflash_lock();       //Flash����
	__enable_irq();        //Flashд����ϣ��ָ��ж�
	
	return status;
}


//��ָ�������� ����/ȡ�� д����
//��ڲ��������������û�ȡ����1������  0��ȡ����
FlashState set_sector_WriteProtect(uint8_t sector,uint8_t enable)
{
	FlashState state = flash_no_error;
	uint8_t set_sector;
	
	
	if(sector > stmflash_get_flash_sector(FLASH_END_ADDR))
	{
		state = flash_sector_out;
		return state;
	}
	state = stmflash_wait_done(0xFFFF);//�ȴ���һ�β�������
	
	//Flash������û������
	if(state==flash_no_error)
	{
		stmflash_opt_unlock(); //����ѡ���ֽڵļĴ�������
		
		if(sector<=11) //F40xϵ��оƬ�����ֻ��11������
		{
			set_sector = sector + 16 ; //16λ�Ĵ�������λ�õ�ƫ����
			
			//����д����
			if(enable) FLASH->OPTCR &= ~(1<<set_sector);
			
			//ȡ��д����
			else FLASH->OPTCR |= 1<<set_sector;
				
		}
		else // F42x��F43x,�������ɵ�23
		{
			set_sector = sector-12 + 16 ; //16λ�Ĵ�������λ�õ�ƫ����,-12�����ǰ��11��������ƫ��
			
			//����д����
			if(enable) FLASH->OPTCR1 &= ~(1<<set_sector);
			
			//ȡ��д����
			else FLASH->OPTCR1 |= 1<<set_sector;
		}
		
		//��ʼд��ѡ���ֽ�
		FLASH->OPTCR |= 1<<1;
		
		//�ȴ���������
		state = stmflash_wait_done(0xFFFF);
		
		//���д��ѡ���ֽڱ�־λ
		//ע����һ����ʵ����Ҫ�����۶�Flash������û�гɹ������λ���Զ���0
		//FLASH->OPTCR &= ~(1<<1);
		
		//��ѡ���ֽ�������
		stmflash_opt_lock();
	}
	
	return state;
	
}


u8 get_sector_WriteProtect(u8 sector)
{
	
	u16 temp;
	
	if(sector<=11)
	{
		temp = stmflash_read_word(0x1fffc008);
		
	}
	else
	{
		temp = stmflash_read_word(0x1ffec008);
		
		sector = sector-12; //��ȥƫ��
	}
	
	return !(temp>>sector&0x01);

}

//���⺯��,��Flashд������.
//��ڲ�����32λ���ݵĵ�ַ,д������ݳ���.
uint8_t Write_Flash(uint32_t* data,uint16_t datalen)
{
	FlashState res;
	
	static u8 init=0;
	if(init==0) 
	{
		init =1;
		stmflash_init();//��ʼ��FLash
	}
	
	//д������ǰ��Ҫ�Ȳ���
	res = stmflash_erase_sector( stmflash_get_flash_sector(FLASH_SAVE_ADDR) );//�Խ�Ҫд���FLashƬ������
	
	if( res==flash_no_error )
	{
		res = stmflash_write(FLASH_SAVE_ADDR,data,datalen);//д����ֵ
	}

	return res;
}

//���⺯��,��ȡFlash����.
//��ڲ�������ȡ���ݵ�������
int Read_Flash(uint16_t index)
{
	int tmp;
	tmp = stmflash_read_word( FLASH_SAVE_ADDR+(4*index) );
	return tmp;
}

