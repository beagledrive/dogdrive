/*
 * Beaglebone Open-Source Machine Drive
 * Trigonometric Functions
 * KTH Project Work - 2018 
 */


/* ================================== INCLUDES ============================== */

#include <math.h>
#include <trigonom.h>


/* ================================== FUNCTION DEFINITIONS ================== */

// Function for calculation of cosine
void cal_cos(float theta, float *costheta)
{
	float accuracy = 0.0001, x1, denominator, cosx, cosval;

	x1 = 1;

	// maps the sum along the series
	cosx = x1;

	// holds the actual value of cos(n)
	cosval = cos(theta);
	int i = 1;
	
	while (accuracy <= fabs(cosval-cosx))
	{
		denominator = 2 * i * (2 * i - 1);
		x1 = -x1 * pow(theta, 2) / denominator;
		cosx = cosx + x1;
		i = i + 1;
	};

	*costheta = cosx;
}

// Function for calculating sin value
void cal_sin(float theta, float *sintheta)
{
	float accuracy = 0.0001, denominator, sinx, sinval;

	float x1 = theta;

	// maps the sum along the series
	sinx = theta;

	// holds the actual value of sin(n)
	sinval = sin(theta);
	int i = 1;
	while (accuracy <= fabs(sinval-sinx))
	{
		denominator = 2 * i * (2 * i + 1);
		x1 = -x1 * pow(theta, 2) / denominator;
		sinx = sinx + x1;
		i = i + 1;
	};

	*sintheta = sinx;
}
