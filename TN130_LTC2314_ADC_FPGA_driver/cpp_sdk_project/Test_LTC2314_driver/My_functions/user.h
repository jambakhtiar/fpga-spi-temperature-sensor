#ifndef MY_FUNCTIONS_USER_H_
#define MY_FUNCTIONS_USER_H_

#include "extern_user.h"

#include "Core/core.h"
#include "Core/interrupts.h"
#include "Driver/peripherals.h"

#include "../API/sensors.h"
#include "../API/controllers.h"

/**
 * Main interrupt routine.
 * @param	void
 * @return	tUserSafe	SAFE if the routine is executed without error, UNSAFE otherwise
 */
tUserSafe UserInterrupt(void);

/**
 * Modes of operation of the user-level application
 */
typedef enum{
	STANDBY   = 0,
	STARTUP   = 1,
	NORMAL    = 2,
	SHUTDOWN  = 3,
	EMERGENCY = 4
} tUserState;

/**
 * Routine for fully configure a PWM channel with complementary outputs and a carrier-based modulator
 * Must be called in UserInit()
 * @param	output			the PWM channel or lane to address (from tPwmOutput list)
 * @param	clock			the clock to use as reference for generating the PWM signals
 * @param	carrier			the PWM carrier shape to use (from tPwmCarrier list)
 * @param	deadTime		the dead-time duration between the high and low PWM signals in seconds
 * @param 	device			the id of the device to address (B-Box or B-Board) (optional, default = 0)
 * @return	void
 */
void CbPwm_ConfigureChannel(tPwmOutput output, tClock clock, tPwmCarrier carrier, float deadTime, unsigned int device=0){
	CbPwm_ConfigureClock(output, clock, device);
	CbPwm_ConfigureOutputMode(output, COMPLEMENTARY, device);
	CbPwm_ConfigureCarrier(output, carrier, device);
	CbPwm_ConfigureDeadTime(output, deadTime, device);
	CbPwm_SetDutyCycle(output, 0, device);
	CbPwm_SetPhase(output, 0, device);
}

#endif /* MY_FUNCTIONS_USER_H_ */
