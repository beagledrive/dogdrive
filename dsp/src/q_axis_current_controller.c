#include <stdio.h>
#include <stdlib.h>

/*******  q-axis Current Controller ******/

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
    float Pro_cur_q = 0;
    int AW_Flag = 0;
    float Int_cur_q = 0;
    float w1L = 0;
    float Cca_cur_q = 0;
    float V_ref = 0;


    // Error Computation

    Err_q = Iq_ref - Iq;                                                                                // compute error

    // Proportional part of the PI Controller

    Pro_cur_q = PI_Struct->Kp*Err_q;

    // Integral part of the PI controller
    // Depending on the status of the Anti-windup flag status, integral controller either keeps on integrating
    // or freezes its value to the previous value

    if(AW_Flag == 0)
    {
        Int_cur_q = PI_Struct->Int_pre_q + PI_Struct->Ts*Err_q;
    }

    else
    {
        Int_cur_q = PI_Struct->Int_pre_q;
    }

    // Compute cross-coupling and active resistance

    w1L = PI_Struct->Lsigma*w1;                                  //cross-coupling coefficient

    Cca_cur_q = PI_Struct->Ra*Iq - w1L*Id;                       // contribution from cross coupling and active resistance

    // Output of the PI controller

    V_ref = Pro_cur_q + PI_Struct->Ki*Int_cur_q - Cca_cur_q ;   // update reference voltage including active resistance and cross coupling cancellation

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

    // Anti windup status

    if(*Vq_ref - V_ref == 0)
    {
        AW_Flag = 0;
    }

    else
    {
        AW_Flag = 1;
    }

    PI_Struct->Int_pre_q = Int_cur_q;

}
