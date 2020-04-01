#ifndef ZLG7289_H
#define ZLG7289_H
#include "stm32f10x.h"

#define  G_ODR_B   (GPIOB_BASE  + 0x0C)  
#define  G_IDR_B   (GPIOB_BASE  + 0x08)  

#define BitBand(addr,num)  *((volatile unsigned long *) ((addr &0xf0000000)+0x2000000+((addr&0xfffff) <<5)+(num<<2)))
#define Out(n)  BitBand(G_ODR_B,n)
#define In(n)  BitBand(G_IDR_B,n)

#define CS   Out(4)
#define DATA Out(5)
#define CK   Out(6)
#define DATA_IN In(5)

u8 Zlg7289_Reade(void);
void Zlg7289_Write(u8 instruct);
void Zlg7289_Write_Data(u8 instruct,u8 data);
void Zlg7289_Gpio__Init(void);

#endif

