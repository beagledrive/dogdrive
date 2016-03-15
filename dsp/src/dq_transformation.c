/* Beaglebone Open-Source Machine Drive
 * d-q Transformation function
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

} DQ_Typedef;


/* ================================== FUNCTION PROTOTYPES =================== */

/*
 * Initialize D-Q transformation structure
 */
void DQ_StructInit(DQ_Typedef *DQ_Struct,float SpcVectScaling);

/*
 * D-Q Tranformation function
 */
void DQ_Transformation(DQ_Typedef *DQ_Struct,float Ia,float Ib,float Ic,float theta1,float *Id,float *Iq);


/* ================================== INTERNAL GLOBALS ====================== */


/* ================================== FUNCTION DEFINITIONS ================== */

void DQ_StructInit(DQ_Typedef *DQ_Struct,float SpcVectScaling)
{
    DQ_Struct->K = SpcVectScaling;
}

void DQ_Transformation(DQ_Typedef *DQ_Struct,float Ia,float Ib,float Ic,float theta1,float *Id,float *Iq)
{
    float Ialpha = 0;
    float Ibeta = 0;
    float sin_theta1 = 0;
    float cos_theta1 = 0;

    // a-b-c to alpha-beta transformation
    Ialpha = DQ_Struct->K*(2*Ia - Ib - Ic)/3;

    Ibeta = DQ_Struct->K*(Ib - Ic)/sqrt(3);

    // alpha-beta to d-q transformation
    sin_theta1 = cal_sin(theta1);
    cos_theta1 = cal_cos(theta1);

    *Id = cos_theta1*Ialpha + sin_theta1*Ibeta;
    *Iq = -sin_theta1*Ialpha + cos_theta1*Ibeta;
}
