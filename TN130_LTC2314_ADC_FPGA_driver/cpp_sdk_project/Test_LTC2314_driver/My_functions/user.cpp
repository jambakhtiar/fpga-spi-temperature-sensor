#include "user.h"

#define ADC_GAIN (4.096/8192.0)

unsigned int adc_raw;
float Vmeas;

/**
 * Initialization routine executed only once, before the first call of the main interrupt
 * To be used to configure all needed peripherals and perform all needed initializations
 */
tUserSafe UserInit(void)
{

	Clock_SetFrequency(CLOCK_0, 20e3);
	ConfigureMainInterrupt(UserInterrupt, CLOCK_0, 0.5);

	Sbi_ConfigureAsRealTime(0); // SBI_reg_00 contains the ADC value (LT2314_driver data_out)
	Sbo_WriteDirectly(0, 2);    // SBO_reg_00 is the clk postscaler (LT2314_driver postscaler_in)
	                              // postscaler = 2 -> SCK = 62.5 MHz

	return SAFE;
}

/**
 * Main interrupt routine
 */
tUserSafe UserInterrupt(void)
{
	adc_raw = Sbi_Read(0);      // read SBI_reg_00
	Vmeas = adc_raw * ADC_GAIN; // convert to Volts

	return SAFE;
}

/**
 * Routine executed when the core state goes into FAULT mode
 */
void UserError(tErrorSource source)
{

}
