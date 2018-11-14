#include <stdio.h>
#include <stdlib.h>

/*******  q-axis Current Controller ******/
// This code is not yet tested

typedef struct
{
    float Kp;                 // Proportional Gain
    float Ki;			      // q-axis Integral Gain
    float Int_pre_q;	      // Previous value of the Integral output
    float Ts;                 // Sampling time
    float Ra;                 // Active Resistance
    float Lsigma;             // Leakage Inductance
    float Vq_ulim;            // Upper limit of the q-axis voltage
    float Vq_llim;            // Lower limit of the q-axis voltage

}PI_Typedef;

void PI_StructInit(PI_Typedef *PI_Struct,float PropCoeffi,float IntCoffi,float Tsamp,float R_active,float L_leak,float Vq_max, float Vq_min);
void PI_Controller(PI_Typedef *PI_Struct,float Iq_ref,float Iq,float Id,float w1,float *Vq_ref);

void PI_StructInit(PI_Typedef *PI_Struct,float PropCoeffi,float IntCoffi,float Tsamp,float R_active,float L_leak)
{
    PI_Struct->Kp = PorpCoeffi;
    PI_Struct->Ki = IntCoffi;
    PI_Struct->Int_pre_q = 0;
    PI_Struct->Ts = Tsamp;
    PI_Struct->Ra = R_active;
    PI_Struct->Lsigma = L_leak;
    PI_Struct->Vq_ulim = Vq_max;
    PI_Struct->Vq_llim = Vq_min;

}
void PI_Controller(PI_Typedef *PI_Struct,float Iq_ref,float Iq,float Id,float w1,float *Vq_ref)
{
    float Err_q = 0;
    float Int_cur_q = 0;
    float w1L = 0;
    float V_ref = 0;

    Err_q = Iq_ref - Iq;                                                                                // compute error

    Int_cur_q = PI_Struct->Int_pre_q + PI_Struct->Ts*Err_q;                                                        // update integral part due to error, anti-windup is ignored in this step

    w1L = PI_Struct->Lsigma*w1;                                                                        // compute w1L

    V_ref = PI_Struct->Kp*Err_q + PI_Struct->Ki*Int_cur_q - PI_Struct->Ra*Iq + w1L*Id ;               // update reference voltage including active resistance and cross coupling cancellation

    // Saturation Implementation

    if(V_ref >= PI_Struct->Vq_ulim)
    {
        *Vq_ref = PI_Struct->Vq_ulim;
    }

    else if (V_ref <= PI_Struct->Vq_llim)
    {
        *Vq_ref = PI_Struct->Vq_llim;
    }

    else
    {
        *Vq_ref = V_ref;
    }

    // Anti windup implementation - this implementation is the one Luca discussed in the meeting, not the one from the course book

    if(*Vq_ref - V_ref == 0)
    {
        PI_Struct->Int_pre_q = Int_cur_q;
    }

    else
    {
        PI_Struct->Int_pre_q = 0;
    }

}
