/*
 * Beaglebone Open-Source Machine Drive
 * IM Model header
 * KTH Project Work - 2018 
 */

#ifndef IM_MODEL_H_		// Header wrapper name
#define IM_MODEL_H_


/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */

/*
 * Induction Motor descriptor
 */
typedef struct
{
    float Rs;                 // Stator resistance in dynamic T-equivalent circuit for the induction motor [Ohm]
    float Rr;		      // Rotor resistance in dynamic T-equivalent circuit for the induction motor [Ohm]
    float Ll;                 // Leakage inductance in dynamic T-equivalent circuit for the induction motor [mH]
    float Lm;                 // Magnetizing inductance in dynamic T-equivalent circuit for the induction motor [mH]
    float Ts;                 // Sampling time [s]
    float Np;                 // Number of pole pairs
    float K;                  // Space vector scaling constant
    float J;                  // Mechanical inertia constant
    float b;                  // damping constant

} IM_Typedef;


/* ================================== FUNCTION PROTOTYPES =================== */

/*
 * Initialize Induction Model structure
 */
void IM_StructInit(IM_Typedef *IM_Struct,float Rstator,float Rrotor,float Lleak,
			float Lmag,float Tsamp,float Npp,float SpcVectScaling,
			float InrConst,float DampConst);
/*
 * Induction Model Function
 */
void IM_model(IM_Typedef *IM_Struct,float Valpha,float Vbeta,float TL,
			float *Ia,float *Ib,float *Ic,float *Wr);

#endif
