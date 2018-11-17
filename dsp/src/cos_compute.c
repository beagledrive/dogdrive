#include <math.h>
#include <tralgo.h>

// Function for calculation of cosine
float cal_cos(float n)
{
	float accuracy = 0.0001, x1, denominator, cosx, cosval;

	x1 = 1;

	// maps the sum along the series
	cosx = x1;

	// holds the actual value of cos(n)
	cosval = cos(n);
	int i = 1;
	
	while (accuracy <= fabs(cosval-cosx))
	{
		denominator = 2 * i * (2 * i - 1);
		x1 = -x1 * n * n / denominator;
		cosx = cosx + x1;
		i = i + 1;
	};

	return cosx;
}

