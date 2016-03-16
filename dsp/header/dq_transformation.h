/*
 * Beaglebone Open-Source Machine Drive
 * DQ Transformation header
 * KTH Project Work - 2018 
 */

#ifndef DQ_TRANSFORM_H_		// Header wrapper name
#define DQ_TRANSFORM_H_


/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */


/* ================================== FUNCTION PROTOTYPES =================== */

/*
 * D-Q Tranformation function
 */
void DQ_Transformation(float SV_scale_const,float Ia,float Ib,float Ic,float theta1,float *Id,float *Iq);

/*
 * Inverse D-Q Tranformation function
 */
void IDQ_Transformation(float SV_scale_const,float Vd,float Vq,float theta1,
			float *Valpha,float *Vbeta,float *Va,float *Vb,float *Vc);


#endif
