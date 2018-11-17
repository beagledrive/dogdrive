// CPP code for implementing cos function
#include <iostream>
#include <math.h>
using namespace std;

// Function for calculation
void cal_cos(float n)
{
	float accuracy = 0.0001, x1, denominator, cosx, cosval;

	x1 = 1;

	// maps the sum along the series
	cosx = x1;

	// holds the actual value of cos(n)
	cosval = cos(n);
	int i = 1;
	do
	{
		denominator = 2 * i * (2 * i - 1);
		x1 = -x1 * n * n / denominator;
		cosx = cosx + x1;
		i = i + 1;
	} while (accuracy <= fabs(cosval - cosx));
	cout << cosx;
}

