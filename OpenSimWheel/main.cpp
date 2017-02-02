
#include "Hardware/TMS320F2806.hh"
#include "Hardware/OSWComponents.hh"

int main(void)
{
	TMS320F2806 processor = TMS320F2806();
	OSWDigital digital = OSWDigital();

	processor.setup(PLL_ClkFreq_80_MHz);

	digital.setMode(GPIO_Number_34,GPIO_34_Mode_GeneralPurpose);
	digital.setMode(GPIO_Number_39,GPIO_39_Mode_GeneralPurpose);
	digital.setDirection(GPIO_Number_34,GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_39,GPIO_Direction_Output);



	while(true)
	{
		digital.toggle(GPIO_Number_34);
		DELAY_US(5e5);
		digital.toggle(GPIO_Number_39);
		DELAY_US(5e5);
	}

}
