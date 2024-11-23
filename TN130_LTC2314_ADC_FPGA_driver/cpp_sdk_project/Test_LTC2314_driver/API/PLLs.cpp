/*
 *	@title	Software Phase-lock-loop routines
 *	@author	imperix Ltd (dev@imperix.ch)
 *	@file	PLLs.c
 */

#include "PLLs.h"							                                    // Corresponding header file
#include <cmath>							                                    // Standard math library

#define PI 3.141592654
#define TWOPI 6.283185307


void ConfigDQPLL(DQPLLParameters* me, float kp, float ki, float omega0, float tsample)
{
	// Set the PLL parameters:
    me->omega0 = omega0;
    me->ts = tsample;

	// Configure the corresponding controller:
    ConfigPIDController(&(me->PI_reg), kp, ki, 0.0, 0.1*omega0, -0.1*omega0, tsample, 10);

    // Initialize the state quantities:
    me->theta = 0.0;
    me->omega = omega0;
}


void ConfigSOGIPLL1(SOGIPLL1Parameters* me, float kp, float ki, float sogigain, float omega0, float tsample)
{
	// Configure the inner SOGI object:
	ConfigSOGI3(&me->SOGI, sogigain, omega0, tsample);

	// Configure the inner PI controller:
	ConfigPIDController(&me->PI_reg, kp, ki, 0.0, 0.1*omega0, -0.1*omega0, tsample,10);

	// Set the PLL parameters:
    me->omega0 = omega0;
    me->ts = tsample;

    // Initialize the state variable:
    me->theta = 0.0;
    me->omega = omega0;
}


void ConfigSOGI3(SOGI3Parameters* me, float gain, float omega0, float tsample)
{
	// Configure the SOGI parameters:
	me->omega = omega0;
	me->gain = gain;
	me->constant = tsample/12.0;			// The constant parameter should be Ts/12

	// Initialize the inner state variables:
	me->states[0].z1 = 0.0;
	me->states[0].z2 = 0.0;
	me->states[0].z3 = 0.0;
	me->states[0].output = 0;

	me->states[1].z1 = 0.0;
	me->states[1].z2 = 0.0;
	me->states[1].z3 = 0.0;
	me->states[1].output = 0;
}


void ConfigDSOGIPLL3(DSOGIPLL3Parameters* me, float kp, float ki, float sogigain, float omega0, float tsample)
{
	// Configure the inner SOGI objects:
	ConfigSOGI3(&me->SOGIa, sogigain, omega0, tsample);
	ConfigSOGI3(&me->SOGIb, sogigain, omega0, tsample);

	// Configure the inner PI controller:
	ConfigPIDController(&me->PI_reg, kp, ki, 0.0, 0.1*omega0, -0.1*omega0, tsample,10);

	// Set the PLL parameters:
    me->omega0 = omega0;
    me->ts = tsample;

    // Initialize the state variable:
    me->theta = 0.0;
    me->omega = omega0;
}


void ConfigFAE(FAEParameters* me, float R, float L, float tsample)
{
	// Precompute the parameters offline:
	me->a = tsample/(L + R * tsample);
	me->b = L/(L + R * tsample);

	// Initialize the state quantities:
	me->state = 0.0;
}


float RunDQPLL(DQPLLParameters* me, const SpaceVector *vin_dq0)
{
	/*********************************************************************** 
	 * Begin of PI controller code
	 ***********************************************************************/
		float ui;
		float u;

		ui = me->PI_reg.ui_prev + me->PI_reg.ki/me->PI_reg.kp * vin_dq0->imaginary;

		// Compute the output:
		u = me->PI_reg.kp * (vin_dq0->imaginary + ui);

		// Apply the standard Anti-Reset Windup method:
		if (u > me->PI_reg.limup){
			me->PI_reg.ui_prev = me->PI_reg.limup / me->PI_reg.kp - vin_dq0->imaginary;
			u = me->PI_reg.limup;
		}
		else if (u < me->PI_reg.limlow){
			me->PI_reg.ui_prev = me->PI_reg.limlow / me->PI_reg.kp - vin_dq0->imaginary;
			u = me->PI_reg.limlow;
		}
		else{
			me->PI_reg.ui_prev = ui;
											// The integral term is never reset
		}
	/************************************************************************ 
	 * End of PI controller code
	 ************************************************************************/

    // Control the q axis of the voltage to zero (u is the output of the PI):
	me->omega = me->omega0 + u;

	// Integrate the angular frequency:
	me->theta += me->omega * me->ts;

    // Oscillate (modulo):
	if (me->theta > PI){me->theta -= TWOPI;}
	else if (me->theta < - PI){me->theta += TWOPI;}

    return me->theta;
}


SpaceVector RunSOGI3(SOGI3Parameters *me, float measurement)
{
	// Update the states for the first block (step z):
	me->states[0].z3 = me->states[0].z2;
	me->states[0].z2 = me->states[0].z1;

	// Compute the value of the first state of the first block:
	me->states[0].z1 += me->constant * me->omega *(-me->states[1].output + me->gain *(measurement - me->states[0].output));

	// Update the values of the states for the second block (step z):
	me->states[1].z3 = me->states[1].z2;
	me->states[1].z2 = me->states[1].z1;

	// Compute the value of the first state of the second block:
	me->states[1].z1 += me->constant * me->omega * me->states[0].output;

	// Update the output:
	me->states[0].output = 23*me->states[0].z1 - 16*me->states[0].z2 + 5*me->states[0].z3;
	me->states[1].output = 23*me->states[1].z1 - 16*me->states[1].z2 + 5*me->states[1].z3;

	// Prepare the return:
	SpaceVector myreturn;
	myreturn.real = me->states[0].output;
	myreturn.imaginary = me->states[1].output;
	myreturn.offset = 0.0;

	return myreturn;
}


float RunSOGIPLL1(SOGIPLL1Parameters* me, SpaceVector* UABG, float vin)
{
	// Run the SOGI on the alpha axis:
	(*UABG) = RunSOGI3(&me->SOGI,vin);

	// Compute the ABG-DQ0 transform for the Q axis only:
	float vin_q = -sin(me->theta) * UABG->real + cos(me->theta) * UABG->imaginary;

	/*********************************************************************** 
 	 *  Begin of PI controller code
	 ***********************************************************************/
		float ui;
		float u;

		ui = me->PI_reg.ui_prev + me->PI_reg.ki/me->PI_reg.kp * vin_q;

		// Compute the output:
		u = me->PI_reg.kp * (vin_q + ui);

		// Apply the standard Anti-Reset Windup method:
		if (u > me->PI_reg.limup){
			me->PI_reg.ui_prev = me->PI_reg.limup / me->PI_reg.kp - vin_q;
			u = me->PI_reg.limup;
		}
		else if (u < me->PI_reg.limlow){
			me->PI_reg.ui_prev = me->PI_reg.limlow / me->PI_reg.kp - vin_q;
			u = me->PI_reg.limlow;
		}
		else{
			me->PI_reg.ui_prev = ui;
											// The integral term is never reset
		}
	/************************************************************************ 
	 * End of PI controller code
	 ************************************************************************/

    // Control the q axis of the voltage to zero (u is the PI's output):
	me->omega = me->omega0 + u;

	// Integrate the angular frequency:
	me->theta += me->omega * me->ts;

    // Oscillate (modulo):
	if (me->theta > PI){me->theta -= TWOPI;}
	else if (me->theta < - PI){me->theta += TWOPI;}

    return me->theta;
}


float RunDSOGIPLL3(DSOGIPLL3Parameters* me, SpaceVector* vin_abg)
{
	// Run the two SOGIs on the measured inputs:
	SpaceVector a = RunSOGI3(&me->SOGIa,vin_abg->real);
	SpaceVector b = RunSOGI3(&me->SOGIb,vin_abg->imaginary);

	// Compute the crossed sums:
	SpaceVector UABG;
	UABG.imaginary = a.imaginary + b.real;
	UABG.real = a.real - b.imaginary;

	// Compute the ABG-DQ0 transform for the Q axis only:
	float vin_q = -sin(me->theta) * UABG.real + cos(me->theta) * UABG.imaginary;

	/*********************************************************************** 
 	 *  Begin of PI controller code
	 ***********************************************************************/
		float ui;
		float u;

		ui = me->PI_reg.ui_prev + me->PI_reg.ki/me->PI_reg.kp * vin_q;

		// Compute the output:
		u = me->PI_reg.kp * (vin_q + ui);

		// Apply the standard Anti-Reset Windup method:
		if (u > me->PI_reg.limup){
			me->PI_reg.ui_prev = me->PI_reg.limup / me->PI_reg.kp - vin_q;
			u = me->PI_reg.limup;
		}
		else if (u < me->PI_reg.limlow){
			me->PI_reg.ui_prev = me->PI_reg.limlow / me->PI_reg.kp - vin_q;
			u = me->PI_reg.limlow;
		}
		else{
			me->PI_reg.ui_prev = ui;
											// The integral term is never reset
		}
	/************************************************************************ 
	 * End of PI controller code
	 ************************************************************************/

    // Control the q axis of the voltage to zero (u is the PI's output):
	me->omega = me->omega0 + u;

	// Integrate the angular frequency:
	me->theta += me->omega * me->ts;

    // Oscillate (modulo):
	if (me->theta > PI){me->theta -= TWOPI;}
	else if (me->theta < - PI){me->theta += TWOPI;}

    return me->theta;
}


float RunFAE(FAEParameters *me, float delta)
{
	// Apply the first-order transfer function:
    me->state = me->a * delta + me->b * me->state;

    return me->state;
}
