/*
 * Beaglebone Open-Source Machine Drive
 * C Header template
 * KTH Project Work - 2018 
 */

#ifndef TRIGONOM_H_		// Header wrapper name
#define TRIGONOM_H_

/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */


/* ================================== FUNCTION PROTOTYPES =================== */

/*
 * Cos-compute function for d-q transformation
 */
void cal_cos(float theta, float *costheta);

/*
 * Sin-compute function for d-q transformation
 */
void cal_sin(float theta, float *sintheta);




#endif
