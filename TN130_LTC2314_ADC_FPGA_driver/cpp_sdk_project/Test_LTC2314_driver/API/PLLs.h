#ifndef PLLS_H_
#define PLLS_H_

#include "transformations.h"	                                                // Three-phase data types
#include "controllers.h"		                                                // Controller parameters and data types

#include <stdint.h>


/**
 *	Struct holding the state information for a triple integrator. Such a module is used to approximate
 *	1/s in a discrete environment. All the parameters stored in this struct make up a SOGI pseudo-object
 *	based on the triple integrator approximation.
 */
typedef struct {
	float z1;
	float z2;
	float z3;
	float output;
} SOGI3States;

typedef struct {
	SOGI3States states[2];
	float omega;
	float gain;
	float constant;
} SOGI3Parameters;


/**
 * Pseudo-object describing a particular estimator called FAE (Fictive-axis estimator)
 * L and R are the line-related parameters (inductance and resistance, respectively),
 * Tsample is the discretization sampling time and period of the interrupt.
 */
typedef struct{
	float a;					                                                // Coefficient a
	float b;					                                                // Coefficient b
	float state;				                                                // Previous value of the output
} FAEParameters;


/**
 *  Parameters for the DQ-based Phase-Locked Loop. Parameter omega0 refers to the nominal angular frequency of the
 *  sinusoidal signal
 */
typedef struct{
	float theta;				                                                // Phase angle of the grid voltage
	float omega;				                                                // Debug only: is not a state variable
	float omega0;				                                                // Default grid frequency (feedforward quantity)
	float ts;					                                                // Sampling interval
	PIDController PI_reg;		                                                // Corresponding PID controller pseudo-object
} DQPLLParameters;


/**
 *  Parameters for the SOGI-based single-phase PLL
 */
typedef struct{
	float theta;				                                                // Phase angle of the grid voltage
	float omega;				                                                // Debug only: is not a state variable
	float omega0;				                                                // Default grid frequency (feedforward quantity)
	float ts;					                                                // Sampling interval
	SOGI3Parameters SOGI;		                                                // Second-order generalized integrator
	PIDController PI_reg;		                                                // Corresponding PID controller pseudo-object
} SOGIPLL1Parameters;


/**
 *  Parameters for the double SOGI-based three-phase PLL
 */
typedef struct{
	float theta;				                                                // Phase angle of the grid voltage
	float omega;				                                                // Debug only: is not a state variable
	float omega0;				                                                // Default grid frequency (feedforward quantity)
	float ts;					                                                // Sampling interval
	SOGI3Parameters SOGIa;		                                                // Second-order generalized integrator, alpha axis
	SOGI3Parameters SOGIb;		                                                // Second-order generalized integrator, beta axis
	PIDController PI_reg;		                                                // Corresponding PID controller pseudo-object
} DSOGIPLL3Parameters;


/*
 * Routine to initialize the PLL based on dq transformation (loop filter on the q-axis)
 * @param *me		the corresponding PLL pseudo-object (parameters and state quantities)
 * @param kp		proportional gain of the inner PI controller
 * @param ki		integral gain of the inner PI controller
 * @param omega0	nominal angular frequency (feedforward term)
 * @param tsample 	sampling (interrupt) time
 */
void ConfigDQPLL(DQPLLParameters* me, float kp, float ki, float omega0, float tsample);


/**
 * Routine to initialize the single-phase PLL based on a second-order generalized integrator
 * @param *me		the corresponding PLL pseudo-object (parameters and state quantities)
 * @param kp		proportional gain of the inner PI controller
 * @param ki		integral gain of the inner PI controller
 * @param sogigain	the specific gain of the SOGI block
 * @param omega0	nominal angular frequency (feedforward term)
 * @param tsample 	sampling (interrupt) time
 */
void ConfigSOGIPLL1(SOGIPLL1Parameters* me, float kp, float ki, float sogigain, float omega0, float tsample);


/**
 * Initialize the SOGI module with default values.
 * This routines defines its internal parameters GAIN, OMEGA and CONST.
 * @param *me  		the corresponding SOGI pseudo-object (parameters and state quantities)
 * @param gain 		the gain value of the SOGI module
 * @param omega0	the expected angle speed of the input signal (feedforward)
 * @param tsample	the sampling time
 */
void ConfigSOGI3(SOGI3Parameters* me, float gain, float omega0, float tsample);


/**
 * Routine to initialize the three-phase PLL based on a double-SOGI-based approach
 * @param *me		the corresponding PLL pseudo-object (parameters and state quantities)
 * @param kp		proportional gain of the inner PI controller
 * @param ki		integral gain of the inner PI controller
 * @param sogigain	the specific gain of both SOGI blocks
 * @param omega0	nominal angular frequency (feedforward term)
 * @param tsample 	sampling (interrupt) time
 */
void ConfigDSOGIPLL3(DSOGIPLL3Parameters* me, float kp, float ki, float sogigain, float omega0, float tsample);


/**
 * Routine to configure the fictive-axis emulation pseudo-object.
 * @param *me		the corresponding FAE pseudo-object (parameters and state quantities)
 * @param R			the parasitic resistance of the line inductor
 * @param L			the nominal value of the line inductor
 * @param tsample 	sampling (interrupt) time
 */
void ConfigFAE(FAEParameters* me, float R, float L, float tsample);


/**
 * Update the SOGI3 module, takes the measurement value as an input.
 * @param *me		the corresponding SOGI pseudo-object (parameters and state quantities)
 * @param input		the signal to be "filtered"
 * @return 			a space vector containing the alpha- and beta-axis values corresponding to the input
 */
SpaceVector RunSOGI3(SOGI3Parameters *me, float input);


/**
 * Run the PLL  based on dq transformation only (typ. minimization of the quadrature voltage)
 * @param *me		the corresponding PLL pseudo-object (parameters and state quantities)
 * @param *ug_dq0	the reference quantity (typ. the grid voltage in dq0 reference frame)
 * @return			the phase angle (typ. of the grid voltage)
 */
float RunDQPLL(DQPLLParameters* me, const SpaceVector *ug_dq0);


/**
 * Run the single-phase SOGI-based PLL
 * @param	*me 	the SOGI pseudo-object
 * @param	*UABG	the filtered version of ug_alpha with quadrature signal. this variable is updated during the function call.
 * @param 	ug		the unfiltered input (typ. the grid voltage)
 * @return			the phase angle (typ. of the grid voltage)
 */
float RunSOGIPLL1(SOGIPLL1Parameters* me, SpaceVector* UABG, float ug);


/**
 * Run the PLL based on dq transformation and double SOGI
 * @param *me		the corresponding PLL pseudo-object (parameters and state quantities)
 * @param *ug_abg	the reference quantity (typ. the grid voltage in abg reference frame)
 * @return			the phase angle (typ. of the grid voltage)
 */
float RunDSOGIPLL3(DSOGIPLL3Parameters* me, SpaceVector* ug_abg);


/**
 * Routine to run the Fictive-axis Emulator
 * This routine is typically used to emulate the beta-axis of the grid current in a dq-controlled
 * single-phase inverter
 * @param	*me	 	the corresponding FAE pseudo-object (parameters and state quantities)
 * @param	delta	FAE's input (typ. the voltage delta on the inductor to emulate, ie. Ug.imaginary - E_vsc.imaginary)
 * @return			FAE's output (typ. the emulated current, ie. Ig.imaginary)
 */
float RunFAE(FAEParameters *me, float delta);

#endif /*PLLS_H_*/
