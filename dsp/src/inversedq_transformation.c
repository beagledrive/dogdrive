/* Beaglebone Open-Source Machine Drive
 * Inverse d-q Transformation function
 * KTH Project Work - 2018 
 */


/* ================================== INCLUDES ============================== */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <tralgo.h>


/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */

typedef struct
{
    float K;                  // space-vector scaling constant

} IDQ_Typedef;


/* ================================== FUNCTION PROTOTYPES =================== */

/*
 * Initialize inverse D-Q transformation structure
 */
void IDQ_StructInit(IDQ_Typedef *IDQ_Struct,float SpcVectScaling);

/*
 * Inverse D-Q Tranformation function
 */
void IDQ_Transformation(IDQ_Typedef *IDQ_Struct,float Vd,float Vq,float theta1,
			float *Valpha,float *Vbeta,float *Va,float *Vb,float *Vc);

/* ================================== INTERNAL GLOBALS ====================== */


/* ================================== FUNCTION DEFINITIONS ================== */

void IDQ_StructInit(IDQ_Typedef *IDQ_Struct,float SpcVectScaling)
{
    IDQ_Struct->K = SpcVectScaling;

}
void IDQ_Transformation(IDQ_Typedef *IDQ_Struct,float Vd,float Vq,float theta1,
			float *Valpha,float *Vbeta,float *Va,float *Vb,float *Vc)
{
    float sin_theta1 = 0;
    float cos_theta1 = 0;

    // d-q to alpha-beta transformation
    sin_theta1 = cal_sin(theta1);
    cos_theta1 = cal_cos(theta1);

    *Valpha = cos_theta1*Vd -sin_theta1*Vq;

    *Vbeta = sin_theta1*Vd + cos_theta1*Vq;

    // alpha-beta to a-b-c transformation
    *Va = *Valpha/IDQ_Struct->K;

    *Vb = (-(*Valpha) + sqrt(3)*(*Vbeta))/(2*IDQ_Struct->K);

    *Vc = (-(*Valpha) - sqrt(3)*(*Vbeta))/(2*IDQ_Struct->K);
}

