#include "drive.h"

void DriveInit(void)
{
	SCB->AIRCR|=0x50|0x05fa0000;                    //NVIC 2
	
  Dian_Ji_Init();
	Dian_Ji_Read();
	
	Timer4Init();
	Systick_init();
	
	Adc_Init();
	Dma_Init();
	Infrared_Send_Init();
	
	//Zlg7289_Gpio__Init();
	
	LedAnKey_Init();
	UsartInit(115200);
}
