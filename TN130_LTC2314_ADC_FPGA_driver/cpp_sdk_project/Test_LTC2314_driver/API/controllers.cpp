/*
 *	@title	Discrete closed-loop controllers
 *	@author	imperix Ltd (dev@imperix.ch)
 *	@file	controllers.c
 */


#include "controllers.h"					                                    // Corresponding header file
#include <cmath>							                                    // Standard math library

#include "Core/core.h"


/*
 * Routine to configure the PID controller and pre-compute the necessary constants.
 */
void ConfigPIDController(PIDController* me, float kp, float ki, float td, float limup, float limlow, float tsample, uint16_t N)
{
	// Set the controller parameters:
	me->kp = kp;
	me->ki = ki;
	me->N = N;
	me->limup = limup;
	me->limlow = limlow;

	// Pre-compute the controller parameters offline:
	me->b = td /(td + N * tsample);

	// Initialize the state quantities:
	me->e_prev = 0.0;
	me->ui_prev = 0.0;
	me->ud_prev = 0.0;
}


/*
 * Routine to configure the PR controller 'me' and pre-compute the necessary constants.
 * Definition and coding according to the following article:
 * "Proportional-resonant controllers and filters for grid-connected voltage-source converters"
 * from R. Teodorescu, F. Blaabjerg, M. Liserre and P.C. Loh
 * published in IEE Proc.-Electr. Power Appl., Vol. 153, N�5, September 2006
 */
void ConfigPRController(PRController* me, float kp, float ki, float wres, float wdamp, float tsample)
{
	// Set the proportional gain:
	me->kp = kp;

	// Precompute the controller parameters offline:
	float kt = 2/tsample;
	me->a1 = 2*ki*kt*wdamp;
	me->a2 = me->a1;
	me->b0 = kt*kt + 2*kt*wdamp + wres*wres;
	me->b1 = 2*kt*kt - 2*wres*wres;
	me->b2 = kt*kt + 2*kt*wdamp + wres*wres;
	
	// Initialize the state quantities:
	me->ui_prev = 0.0;
	me->ui_prev2 = 0.0;
	me->e_prev = 0.0;
	me->e_prev2 = 0.0;
}


/*
 * Routine to initialize the Maximum Power Point Tracker (MPPT) to the default value set by out_init
 */
void ConfigMPPTracker(MPPTracker* me, float ref_step, float ref_init, float limup, float limlow, float iir_lpf){
	me->power_prev = 0.0;
	me->meas_prev = 0.0;
	me->reference = ref_init;
	me->reference_step = ref_step;
	me->limup = limup;
	me->limlow = limlow;
	me->iir_lpf = iir_lpf;
}

/*
 * Routine to run the PID controller (Mixed structure).
 * Implementation accoring the book "Commande num�rique de syst�mes dynamiques", R. Longchamp, PPUR (http://www.ppur.org/)
 */
float RunPIDController(PIDController* me, float error)
{
	float ui, ud;							                                    // Integral and derivative parts of the output, respectively
	float u;								                                    // Output quantity

	ui = me->ui_prev + me->ki * error;
	ud = me->b * (me->ud_prev + me->N * (error - me->e_prev));

	// Compute the output:
	u = me->kp * (error + ui + ud);			                                    // Mixed structure (cf. Longchamp p. 355)

	// Apply the standard Anti-Reset Windup method:
	if (u > me->limup){
		me->ui_prev = me->limup / me->kp - error - ud;
		u = me->limup;
	}
	else if (u < me->limlow){
		me->ui_prev = me->limlow / me->kp - error - ud;
		u = me->limlow;
	}
	else{
		me->ui_prev = ui;
	}

	// Update the other state quantities:
	me->ud_prev = ud;
	me->e_prev = error;
	
	// Reset the integral when the outputs are inhibited (when the B-Box is blocked):
	if(GetCoreState() != OPERATING)
		me->ui_prev = 0.0;					                                    // Avoid integrating when core has been disabled

	return u;
}


/*
 * Routine to run the PID controller 'me' as a PI controller only.
 * This routine disregards the derivative term.
 */

float RunPIController(PIDController* me, float error)
{
	float ui;								                                    // Integral part of the output
	float u;								                                    // Output quantity

	ui = me->ui_prev + me->ki/me->kp * error;

	// Compute the output:
	u = me->kp * (error + ui);				                                    // Mixed structure (cf. Longchamp p. 355)

	// Apply the standard Anti-Reset Windup method:
	if (u > me->limup){
		me->ui_prev = me->limup / me->kp - error;
		u = me->limup;
	}
	else if (u < me->limlow){
		me->ui_prev = me->limlow / me->kp - error;
		u = me->limlow;
	}
	else{
		me->ui_prev = ui;
	}
	
	// Reset the integral when the outputs are inhibited (when the B-Box is blocked):
	if(GetCoreState() != OPERATING)
		me->ui_prev = 0.0;					                                    // Avoid integrating when core has been disabled

	return u;
}


/*
 * Routine to run the PID controller as a proportional controller only.
 * This routine disregards both the integral and the derivative terms.
 */
float RunPController(PIDController* me, float error)
{
	// Compute and return the output:
	float u = me->kp * (error);

	// Return the output, with saturation:
	if (u > me->limup)		{ return me->limup; }
	else if (u < me->limlow){ return me->limlow; }
	else					{ return u; }
}


/*
 * Routine to run the PID controller as an I (Mixed structure) as an integral controller only.
 * This routine disregards both the derivative and the proportional terms.
 */
float RunIController(PIDController* me, float error)
{
	float ui;								                                    // Output quantity
	ui = me->ui_prev + me->ki * error;

	// Apply the standard Anti-Reset Windup method:
	if (ui > me->limup){
		me->ui_prev = me->limup;
		ui = me->limup;
	}
	else if (ui < me->limlow){
		me->ui_prev = me->limlow;
		ui = me->limlow;
	}
	else{
		me->ui_prev = ui;
	}
	
	// Reset the integral when the outputs are inhibited (when the B-Box is blocked):
	if(GetCoreState() != OPERATING)
		me->ui_prev = 0.0;					                                    // Avoid integrating when core has been disabled

	return ui;
}


/*
 * Routine to run the pseudo-object 'me' for PR-like controllers
 */
float RunPRController(PRController* me, float error)
{
	// Compute the first half of the integral term (that which depends only on the error):
	float ua = me->a1* me->e_prev - me->a2* me->e_prev2;

	// Compute the second half of the integral terms:
	float ui = (ua + me->b1* me->ui_prev - me->b2* me->ui_prev2) / me->b0;

	// Update the delayed integral output samples:
	me->ui_prev2 = me->ui_prev;
	me->ui_prev = ui;

	// Update the delayed error samples:
	me->e_prev2 = me->e_prev;
	me->e_prev = error;

	// Reset the integral when the outputs are inhibited (when the B-Box is blocked):
	if(GetCoreState() != OPERATING){
		me->ui_prev = 0.0;
		me->ui_prev2 = 0.0;
	}

	// Return the sum of the integral terms and the proportional gain:
	return  me->kp * error + ui;
}


/*
 * Perturb and observe algorithm
 */
float RunMPPTracking(MPPTracker* me, float measurement, float power)
{
	// Forbid negative values:
	if (power < 0.0){ power = 0.0; }
	if (measurement < 0.0){ measurement = 0.0; }

	// Filter the measurement and the power:
	float power_lpf = (me->iir_lpf)*power + (1.0-me->iir_lpf)*me->power_prev;
	float measurement_lpf = (me->iir_lpf)*measurement + (1.0-me->iir_lpf)*me->meas_prev;

	// Compute the differences since the last interrupt execution:
	float delta_power = power_lpf - me->power_prev;
	float delta_measurement = measurement_lpf - me->meas_prev;

	// Execute the MPP-tracking algorithm:
	if (delta_power >= 0){
		// If there is any improvement, keep going:
		if (delta_measurement >=0 ){ me->reference += me->reference_step; }
		else{ me->reference -= me->reference_step; }
	}
	else{
		// Else, if the power exchange is degrading:

		// First check for saturations:
		if (me->reference > me->limup){
			me->reference -= me->reference_step;
		}
		else if (me->reference < me->limlow){
			me->reference += me->reference_step;
		}
		else if (delta_measurement >=0 ){ me->reference -= me->reference_step; }
		else{ me->reference += me->reference_step; }
	}

	// Update the previous values:
	me->power_prev = power_lpf;
	me->meas_prev = measurement_lpf;

	// Return the new reference value:
	return me->reference;
}
