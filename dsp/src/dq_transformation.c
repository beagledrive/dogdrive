/* Beaglebone Open-Source Machine Drive
 * d-q Transformation function
 * KTH Project Work - 2018 
 */


/* ================================== INCLUDES ============================== */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <trigonom.h>


/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */


/* ================================== FUNCTION PROTOTYPES =================== */


/* ================================== INTERNAL GLOBALS ====================== */


/* ================================== FUNCTION DEFINITIONS ================== */

void DQ_Transformation(float SV_scale_const,float Ia,float Ib,float Ic,float theta1,float *Id,float *Iq)
{
    float Ialpha = 0;
    float Ibeta = 0;
    float sin_theta1 = 0;
    float cos_theta1 = 0;

    // a-b-c to alpha-beta transformation
    Ialpha = SV_scale_const*(2*Ia - Ib - Ic)/3;

    Ibeta = SV_scale_const*(Ib - Ic)/sqrt(3);

    // alpha-beta to d-q transformation
    cal_sin(theta1, &sin_theta1);
    cal_cos(theta1, &cos_theta1);

    *Id = cos_theta1*Ialpha + sin_theta1*Ibeta;
    *Iq = -sin_theta1*Ialpha + cos_theta1*Ibeta;
}

void IDQ_Transformation(float SV_scale_const,float Vd,float Vq,float theta1,
			float *Valpha,float *Vbeta,float *Va,float *Vb,float *Vc)
{
    float sin_theta1 = 0;
    float cos_theta1 = 0;

    // d-q to alpha-beta transformation
    cal_sin(theta1, &sin_theta1);
    cal_cos(theta1, &cos_theta1);

    *Valpha = cos_theta1*Vd -sin_theta1*Vq;

    *Vbeta = sin_theta1*Vd + cos_theta1*Vq;

    // alpha-beta to a-b-c transformation
    *Va = *Valpha/SV_scale_const;

    *Vb = (-(*Valpha) + sqrt(3)*(*Vbeta))/(2*SV_scale_const);

    *Vc = (-(*Valpha) - sqrt(3)*(*Vbeta))/(2*SV_scale_const);
}
