#ifndef TRANSFORMATIONS_H_
#define TRANSFORMATIONS_H_

#include <stdint.h>


// Three-phase quantity in complex form (ABG or DQ0 reference frames)
typedef struct{
	float real;
	float imaginary;
	float offset;
} SpaceVector;


// Three-phase quantity in time domain
typedef struct{
	float A;
	float B;
	float C;
} TimeDomain;


/**
 * Pseudo-object containing all the necessary data for the
 * Double-synchronous Reference Frame (DSRF) Park transformation
 * and sequences decomposition (Triple-synchronous Reference Frame)
 */
typedef struct{
	SpaceVector dqpos;			// Computed positive-sequence components
	SpaceVector dqneg;			// Computed negative-sequence components
	SpaceVector pos_lpf;		// Filtered positive-sequence components (inner state variable)
	SpaceVector neg_lpf;		// Filtered negative-sequence components (inner state variable)
	float k;					// Filtering coefficient
} Sequences;


/**
 * Transformation from physical (abc) to stationary (ABG) reference frame
 * @param *fixed		pointer on the space vector that will be updated
 * @param *physical		pointer on the time domain data that will be transformed
 * @return void			the return is the *rotating space vector itself
 */
void abc2ABG(SpaceVector *fixed, const TimeDomain *physical);


/**
 * Transformation from stationary (ABG) to rotating (DQ0) reference frame
 * @param *rotating		pointer on the DQ0 space vector that will be updated
 * @param *fixed		pointer on the alphabetagamma space vector that will be transformed
 * @param theta 		phase angle used for the transformation
 * @return void			the return is the *rotating structure itself
 */
void ABG2DQ0(SpaceVector *rotating, const SpaceVector *fixed, const float theta);


/**
 * Transformation from physical (abc) to rotating (DQ0) reference frame
 * @param *rotating		pointer on the space vector that will be updated
 * @param *physical		pointer on the time domain data that will be transformed
 * @param theta 		phase angle used for the transformation
 * @return void			the return is the *rotating space vector itself
 */
void abc2DQ0(SpaceVector* rotating, const TimeDomain* physical, const float theta);


/**
 * Transformation from rotating (DQ0) to stationary (ABG) reference frame
 * @param *fixed		pointer on the alphabetagamma space vector that will be updated
 * @param *rotating		pointer on the DQ0 space vector that will be transformed
 * @param theta 		phase angle used for the transformation
 * @return void			the return is the *fixed structure itself
 */
void DQ02ABG(SpaceVector *fixed, const SpaceVector *rotating, const float theta);


/**
 * Transformation from rotating (DQ0) to physical (abc) reference frame
 * @param *physical		pointer on the time domain data that will be updated
 * @param *rotating		pointer on the space vector that will be transformed
 * @param theta 		phase angle used for the transformation
 * @return void			the return is the *physical structure itself
 */
void ABG2abc(TimeDomain *physical, const SpaceVector *fixed);


/**
 * Transformation from rotating (DQ0) to physical (abc) reference frame
 * @param *physical		pointer on the time domain data that will be updated
 * @param *rotating		pointer on the space vector that will be transformed
 * @param theta 		phase angle used for the transformation
 * @return void			the return is the *physical structure itself
 */
void DQ02abc(TimeDomain *physical, const SpaceVector *rotating, const float theta);


/**
 * Routine to configure the Sequences pseudo-object (pseudo-constructor)
 * @param fcut			LPF cut-off frequency
 * @param tsample 		sampling (interrupt) time
 * @return void
 */
void ConfigSequences(Sequences* me, const float fcut, const float tsample);


/**
 * Transformation from physical (abc) to double synchronous reference frame (DSRF)
 * @param *me			pointer on the pseudo-object containing all the necessary data
 * @param *physical		pointer on the time domain (abc) data
 * @param theta 		phase angle used for the transformation
 * @return void			the return is the *physical structure itself
 */
void RunDSRF(Sequences* me, const TimeDomain* physical, const float theta);

#endif /*TRANSFORMATIONS_H_*/
