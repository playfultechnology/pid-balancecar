
#include "include.h"


void Start(void)
{
    WDOG_Disable();	//close the WDOG	
    SCB->CPACR |=((3UL << 10*2)|(3UL << 11*2));     /* set CP10 and CP11 Full Access */
}

void NMI_Handler(void)
{	
}

