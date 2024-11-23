/*
 *	@title	Coordinate-transformation routines
 *	@author	imperix Ltd (dev@imperix.ch)
 *	@file	transformations.c
 */


#include "transformations.h"			    							        // Corresponding header file
#include <cmath>

#define PI 3.141592654
#define TWOPI 6.283185307
float ONE_OVER_SQRT_3 = 0.577350269;
float SQRT_3_OVER_2   = 0.866025403;


void abc2ABG(SpaceVector *fixed, const TimeDomain *physical)
{
	fixed->real = (1/3.) * (2*physical->A - physical->B - physical->C);         // Alpha
	fixed->imaginary = ONE_OVER_SQRT_3*(physical->B - physical->C);		        // Beta
	fixed->offset = (1/3.) * (physical->A + physical->B + physical->C);	        // Gamma
}


void ABG2abc(TimeDomain *physical, const SpaceVector *fixed)
{
	physical->A = fixed->real + fixed->offset;
	physical->B = -1/2. * fixed->real + SQRT_3_OVER_2 * fixed->imaginary + fixed->offset;
	physical->C = -1/2. * fixed->real - SQRT_3_OVER_2 * fixed->imaginary + fixed->offset;
}


void ABG2DQ0(SpaceVector *rotating, const SpaceVector *fixed, const float theta)
{
	float cosTheta; cosTheta=cos(theta);
	float sinTheta; sinTheta=sin(theta); 								        // Compute the sine and cosine only once !

	rotating->real = cosTheta * fixed->real + sinTheta * fixed->imaginary;
	rotating->imaginary = -sinTheta * fixed->real + cosTheta * fixed->imaginary;
	rotating->offset = fixed->offset;
}


void DQ02ABG(SpaceVector *fixed, const SpaceVector *rotating, const float theta)
{
	float cosTheta; cosTheta=cos(theta);
	float sinTheta; sinTheta=sin(theta); 								        // Compute the sine and cosine only once !

	fixed->real = cosTheta * rotating->real - sinTheta * rotating->imaginary;
	fixed->imaginary = sinTheta * rotating->real + cosTheta * rotating->imaginary;
	fixed->offset = rotating->offset;
}


void abc2DQ0(SpaceVector *rotating, const TimeDomain *physical, const float theta)
{
	SpaceVector fixed;
	abc2ABG(&fixed,physical);
	ABG2DQ0(rotating,&fixed,theta);
}


void DQ02abc(TimeDomain *physical, const SpaceVector *rotating, const float theta)
{
	SpaceVector fixed;
	DQ02ABG(&fixed,rotating,theta);
	ABG2abc(physical,&fixed);
}


void ConfigSequences(Sequences* me, const float fcut, const float tsample)
{
	// Set the object parameters:
	me->k = 1 - exp(-TWOPI*fcut*tsample);

	// Initialize the output quantities:
	me->dqpos.real = 0.0;
	me->dqpos.imaginary = 0.0;
	me->dqpos.offset = 0.0;

	me->dqneg.real = 0.0;
	me->dqneg.imaginary = 0.0;
	me->dqneg.offset = 0.0;

	// Initialize the state quantities (low-pass filtered version of the outputs):
	me->pos_lpf.real = 0.0;
	me->pos_lpf.imaginary = 0.0;
	me->neg_lpf.real = 0.0;
	me->neg_lpf.imaginary = 0.0;
}


void RunDSRF(Sequences* me, const TimeDomain* physical, const float theta)
{
	//Define some internal variables:
	SpaceVector fixed, pos, neg, pos_fb, neg_fb;

	//Compute some constant terms:
	float cosTheta; cosTheta=cos(theta);
	float sinTheta; sinTheta=sin(theta);
	float cos2Theta; cos2Theta=cos(2*theta);
	float sin2Theta; sin2Theta=sin(2*theta);

	//Convert to ABG:
	abc2ABG(&fixed,physical);

	//Apply the raw rotations:
	pos.real = cosTheta * fixed.real + sinTheta * fixed.imaginary;
	pos.imaginary = -sinTheta * fixed.real + cosTheta * fixed.imaginary;
	neg.real = cosTheta * fixed.real - sinTheta * fixed.imaginary;
	neg.imaginary = sinTheta * fixed.real + cosTheta * fixed.imaginary;

	//Compute the feedback terms (double-angle rotations from the filtered outputs):
	neg_fb.real = cos2Theta * me->pos_lpf.real - sin2Theta * me->pos_lpf.imaginary;
	neg_fb.imaginary = +sin2Theta * me->pos_lpf.real + cos2Theta * me->pos_lpf.imaginary;
	pos_fb.real = cos2Theta * me->neg_lpf.real + sin2Theta * me->neg_lpf.imaginary;
	pos_fb.imaginary = -sin2Theta * me->neg_lpf.real + cos2Theta * me->neg_lpf.imaginary;

	//Compute the outputs:
	me->dqpos.real = pos.real - pos_fb.real;
	me->dqpos.imaginary = pos.imaginary - pos_fb.imaginary;
	me->dqneg.real = neg.real - neg_fb.real;
	me->dqneg.imaginary = neg.imaginary - neg_fb.imaginary;

	//Compute the low-pass filtered versions of the output (eliminate cross-coupled frequencies):
	me->pos_lpf.real = (1.0-me->k)*me->pos_lpf.real + me->k * me->dqpos.real;
	me->pos_lpf.imaginary = (1.0-me->k)*me->pos_lpf.imaginary + me->k * me->dqpos.imaginary;
	me->neg_lpf.real = (1.0-me->k)*me->neg_lpf.real + me->k * me->dqneg.real;
	me->neg_lpf.imaginary = (1.0-me->k)*me->neg_lpf.imaginary + me->k * me->dqneg.imaginary;
}
