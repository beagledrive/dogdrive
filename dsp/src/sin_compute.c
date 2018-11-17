// CPP code for implementing sin function
#include <iostream>
#include <math.h>
using namespace std;

// Function for calculating sin value
void cal_sin(float n)
{
	float accuracy = 0.0001, denominator, sinx, sinval;

	float x1 = n;

	// maps the sum along the series
	sinx = n;

	// holds the actual value of sin(n)
	sinval = sin(n);
	int i = 1;
	do
	{
		denominator = 2 * i * (2 * i + 1);
		x1 = -x1 * n * n / denominator;
		sinx = sinx + x1;
		i = i + 1;
	} while (accuracy <= fabs(sinval - sinx));
	cout << sinx;
}
