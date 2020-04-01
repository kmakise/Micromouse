#include "ZLG7289.h"
#include "delay.h"

#define DELAY() delay_us(100) 

void Zlg7289_Gpio__Init(void)
{
	RCC->APB2ENR |= (1<<3);   //open GPIOB clock
	RCC->APB2ENR|=1;         //open AFIO clock
	
	AFIO->MAPR|=(0x2<<24);   //forbidden JTAG start using SWD
	
	
	GPIOB->CRL &= 0xf0000fff; //reset GPIOB 3 4 5 6
	GPIOB->CRL |= 0x03338000; //GPIOB 4 5 6 is push_pull output,GPIOB 3 is pull up or pull down input
	
	GPIOB->ODR |=  (1<<4);     //CS is high level
	GPIOB->ODR &= ~(1<<5);    //data is low level
	GPIOB->ODR &= ~(1<<6);    //clock is low level
	GPIOB->ODR |=  (1<<3);    //KEY is high level
}

void Zlg7289_Write(u8 instruct)
{
	
	u8 buf2 = 0;
	u8 buf = 0x80;

	GPIOB->CRL &= 0xff0fffff;
	GPIOB->CRL |= 0x00300000;
	DATA = 0;
	
	
	buf2 = instruct;
	CS = 0;
	DELAY();
	
	
	do{
		  CK = 0;
		  DELAY();
		  if( (buf2 & buf) != 0)
			{
	      DATA = 1;
			}
			else
			{
				DATA = 0;
			}
	    DELAY();
	    CK = 1;
	    DELAY();
		  buf >>= 1;
	}while(buf != 0x00);
	CK = 0;
	CS = 1;
}

void Zlg7289_Write_Data(u8 instruct,u8 data)
{
	
	u8 buf2 = 0,buf3 = 0;
	u8 buf = 0x80,buf1 = 0x80;
	
	GPIOB->CRL &= 0xff0fffff;
	GPIOB->CRL |= 0x00300000;
	DATA = 0;
	
	
	buf2 = instruct|0xc8;
	buf3 = data;
	
	CS = 0;
	DELAY();
	
	do{
		  CK = 0;
		  DELAY();
		  if( (buf2 & buf) != 0)
			{
	      DATA = 1;
			}
			else
			{
				DATA = 0;
			}
	    DELAY();
	    CK = 1;
	    DELAY();
		  buf >>= 1;
	}while(buf != 0x00);
	
	CK = 0;
	DELAY();
	
	do{
		  CK = 0;
		  DELAY();
		  if( (buf3 & buf1) != 0)
			{
	      DATA = 1;
			}
			else
			{
				DATA = 0;
			}
	    DELAY();
	    CK = 1;
	    DELAY();
		  buf1 >>= 1;
	}while(buf1 != 0x00);
	CK = 0;
	CS = 1;
}

void LEDDisplay(int16_t dat1,int16_t dat2)
{
	dat1 = (dat1 < 0) ? -dat1 : dat1;
	dat2 = (dat2 < 0) ? -dat2 : dat2;
	
	Zlg7289_Write_Data(0,dat1 / 1000 % 10);
	Zlg7289_Write_Data(1,dat1 / 100 % 10);
	Zlg7289_Write_Data(2,dat1 / 10 % 10);
	Zlg7289_Write_Data(3,dat1 % 10);

	Zlg7289_Write_Data(4,dat2 / 1000 % 10);
	Zlg7289_Write_Data(5,dat2 / 100 % 10);
	Zlg7289_Write_Data(6,dat2 / 10 % 10);
	Zlg7289_Write_Data(7,dat2 % 10);
	
}


u8 Zlg7289_Reade(void)
{
	u8 buf2 = 0x15;
	u8 buf = 0x80;
	u8 data = 0,i = 0;
	
	GPIOB->CRL &= 0xff0fffff;
	GPIOB->CRL |= 0x00300000;
	DATA = 0;

  CS = 0;
	delay_us(10);
	
	do{
		  CK = 0;
		  delay_us(20);
		  if( (buf2 & buf) != 0)
			{
	      DATA = 1;
			}
			else
			{
				DATA = 0;
			}
	    delay_us(20);
	    CK = 1;
	    delay_us(20);
		  buf >>= 1;
	}while(buf != 0x00);
	CK = 0;
	
	GPIOB->CRL &= 0xff0fffff;
	GPIOB->CRL |= 0x00800000;
	DATA = 1;
	
	delay_us(80);
	
	do{
		  CK = 1;
		  delay_us(10);
			data <<= 1;
		  if(DATA_IN == 1)
			{
				data|= 0x01;
			}
			else
			{
				data &= 0xfe;
			}
			CK = 0;
			delay_us(10);
			i++;
	}while(i < 8);
	
	CS = 1;	
	
	return data;
}

