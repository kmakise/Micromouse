#include "drive.h"
#include "process.h"


int main()
{

	DriveInit();
	
	MainSetup();

	while(1)
	{
		MainLoop();
	}
	
}

