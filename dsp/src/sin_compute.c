#include <math.h>
#include <tralgo.h>

// Function for calculating sin value
float cal_sin(float n)
{
	float accuracy = 0.0001, denominator, sinx, sinval;

	float x1 = n;

	// maps the sum along the series
	sinx = n;

	// holds the actual value of sin(n)
	sinval = sin(n);
	int i = 1;
	while (accuracy <= fabs(sinval-sinx))
	{
		denominator = 2 * i * (2 * i + 1);
		x1 = -x1 * n * n / denominator;
		sinx = sinx + x1;
		i = i + 1;
	};

	return sinx;
}
