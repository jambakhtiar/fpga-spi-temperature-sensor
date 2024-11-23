#ifndef CONTROLLERS_H_
#define CONTROLLERS_H_

#include <stdint.h>


/**
 * Pseudo-object describing a PID controller
 * Definition and operation according to R.Longchamp's book...
 */
typedef struct{
	float kp,ki;				// Proportional and integral gains
	float limup;				// Upper saturation value of the output
	float limlow;				// Lower saturation value of the output
uint16_t 	N;					// Filtering parameter of the derivative
	float b;					// Offline-computed constants (contain the Ti and Td informations)
	float ui_prev;				// Previous value of the integral component
	float ud_prev;				// Previous value of the derivative component
	float e_prev;				// Previous value of the error
} PIDController;


/**
 * Pseudo-object describing a proportional-resonant controller (PR) with one resonant term
 * Definition and coding according to the following article:
 * "Proportional-resonant controllers and filters for grid-connected voltage-source converters"
 * from R. Teodorescu, F. Blaabjerg, M. Liserre and P.C. Loh
 * published in IEE Proc.-Electr. Power Appl., Vol. 153, N�5, September 2006
 */
typedef struct{
	float kp;					// Proportional gain (the integral gain is contained in the other coefficients)
	float a1,a2,b0,b1,b2;		// Internal coefficients (computed offline)
	float ui_prev,ui_prev2;	// Previous values of the integral parts of the outputs (k-1, resp. k-2 samples)
	float e_prev,e_prev2;		// Previous values of the error
} PRController;


/**
 * Pseudo-object corresponding to a Maximum Power Point Tracking (MPPT) algorithm
 * This mainly contains the previous values of the power to be maximized as well as of the
 * quantity acting as a perturbation
 */
typedef struct{
	float power_prev;			// The previous value of the power extracted
	float meas_prev;			// The previous value of the acting quantity (e.g. the current)
	float reference_step;		// The size of the increment/decrement to be applied to the output
	float reference;			// The setpoint of the acting quantity
	float limup;				// Positive limit of the output (reference setpoint)
	float	limlow;				// Negative limit of the output (reference setpoint)
	float iir_lpf;			// Low-passe filtering coefficient
} MPPTracker;


/**
 * Routine to configure the PID controller 'me' and pre-compute the necessary constants.
 * @param *me		the PID pseudo-object to be configured
 * @param kp		proportional gain
 * @param ki		integral gain
 * @param td 		derivative time-constant
 * @param limup		upper saturation threshold of the output quantity
 * @param limup		lower saturation threshold of the output quantity
 * @param tsample 	sampling (interrupt) time
 * @param N 		filtering factor of the derivative term (10 is a good typical value)
 * @return void
 */
void ConfigPIDController(PIDController* me, float kp, float ki, float td, float limup, float limlow, float tsample, uint16_t N);


/**
 * Routine to configure the PR controller 'me' and pre-compute the necessary constants.
 * @param *me		the PID pseudo-object to be configured
 * @param kp		proportional gain
 * @param ki		integral gain
 * @param td 		derivative time-constant
 * @param wres		center resonant frequency of the resonant term (in rad/s.)
 * @param wdamp		frequency "width" of the resonant term (limits the quality factor of the resonant term)  (in rad/s.)
 * @param tsample 	sampling (interrupt) time
 * @return void
 */
void ConfigPRController(PRController* me, float kp, float ki, float wres, float wdamp, float tsample);


/**
 * Routine to construct (i.e. initialize) the pseudo-object corresponding to a perturb-and-observe
 * MPPT algorithm
 * @param *me		the pseudo-object to initialize
 * @param ref_step	the size of the increment/decrement to apply at the output (e.g. on the current)
 * @param ref_init	the default/initial value of the output (the acting quantity, e.g. the current)
 * @param limup		the positive limit (maximum value) of the output
 * @param limlow	the negative limit (minimum value) of the output
 * @param iir_lpf	iir-type low-pass filtering coefficient
 */
void ConfigMPPTracker(MPPTracker* me, float ref_step, float ref_init, float limup, float limlow, float iir_lpf);


/**
 * Routines to run the pseudo-object 'me' depending of its actual nature (PI, PID, etc. controllers)
 * @param *me		the corresponding PID pseudo-object (parameters and state quantities)
 * @param error		the setpoint value minus the measured value
 * @return			the control variable for the measured quantity (output of the controller)
 */
float RunPIDController(PIDController* me, float error);
float RunPIController(PIDController* me, float error);
float RunIController(PIDController* me, float error);
float RunPController(PIDController* me, float error);


/**
 * Routines to run the pseudo-object 'me' for PR-like controllers
 * @param *me		the corresponding PR pseudo-object (parameters and state quantities)
 * @param error		the setpoint value minus the measured value
 * @return			the control variable for the measured quantity (output of the controller)
 */
float RunPRController(PRController* me, float error);


/**
 * Routine to run a Perturb&Observe Maximum Power Point Tracking (MPPT) algorithm
 * @param *me		the pseudo-object corresponding to the power to be maximized
 * @param measurement the perturbation quantity (e.g. the current)
 * @param power		the power to maximize
 * @return			the quantity acting as a perturbation (e.g. a current)
 */
float RunMPPTracking(MPPTracker* me, float measurement, float power);

#endif /*CONTROLLERS_H_*/
