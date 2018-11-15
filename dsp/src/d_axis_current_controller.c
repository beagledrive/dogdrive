#include <stdio.h>
#include <stdlib.h>

/*******  d-axis Current Controller ******/

typedef struct
{
    float Kp;                 // Proportional Gain
    float Ki;			      // d-axis Integral Gain
    float Int_pre_d;	      // Previous value of the Integral output
    float Ts;                 // Sampling time
    float Ra;                 // Active Resistance
    float Lsigma;             // Leakage Inductance
    float Vd_ulim;            // Upper limit of the d-axis voltage
    float Vd_llim;            // Lower limit of the d-axis voltage

}PI_Typedef;

void PI_StructInit(PI_Typedef *PI_Struct,float PropCoeffi,float IntCoffi,float Tsamp,float R_active,float L_leak,float Vd_max, float Vd_min);
void PI_Controller(PI_Typedef *PI_Struct,float Id_ref,float Id,float Iq,float w1,float *Vd_ref);

void PI_StructInit(PI_Typedef *PI_Struct,float PropCoeffi,float IntCoffi,float Tsamp,float R_active,float L_leak)
{
    PI_Struct->Kp = PorpCoeffi;
    PI_Struct->Ki = IntCoffi;
    PI_Struct->Int_pre_d = 0;
    PI_Struct->Ts = Tsamp;
    PI_Struct->Ra = R_active;
    PI_Struct->Lsigma = L_leak;
    PI_Struct->Vd_ulim = Vd_max;
    PI_Struct->Vd_llim = Vd_min;

}
void PI_Controller(PI_Typedef *PI_Struct,float Id_ref,float Id,float Iq,float w1,float *Vd_ref)
{
    float Err_d = 0;
    float Pro_cur_d = 0;
    int AW_Flag = 0;
    float Int_cur_d = 0;
    float w1L = 0;
    float Cca_cur_d = 0;
    float V_ref = 0;


    // Error Computation

    Err_d = Id_ref - Id;                                                                                // compute error

    // Proportional part of the PI Controller

    Pro_cur_d = PI_Struct->Kp*Err_d;

    // Integral part of the PI controller
    // Depending on the status of the Anti-windup flag status, integral controller either keeps on integrating
    // or freezes its value to the previous value

    if(AW_Flag == 0)
    {
        Int_cur_d = PI_Struct->Int_pre_d + PI_Struct->Ts*Err_d;
    }

    else
    {
        Int_cur_d = PI_Struct->Int_pre_d;
    }

    // Compute cross-coupling and active resistance

    w1L = PI_Struct->Lsigma*w1;                                  //cross-coupling coefficient

    Cca_cur_d = PI_Struct->Ra*Id + w1L*Iq;                       // contribution from cross coupling and active resistance

    // Output of the PI controller

    V_ref = Pro_cur_d + PI_Struct->Ki*Int_cur_d - Cca_cur_d ;   // update reference voltage including active resistance and cross coupling cancellation

    // Saturation and anti wind-up flag

    if(V_ref > PI_Struct->Vd_ulim)
    {
        *Vd_ref = PI_Struct->Vd_ulim;
        AW_Flag = 1;
    }

    else if (V_ref < PI_Struct->Vd_llim)
    {
        *Vd_ref = PI_Struct->Vd_llim;
        AW_Flag = 1;
    }

    else
    {
        *Vd_ref = V_ref;
        AW_Flag = 0;
    }

    PI_Struct->Int_pre_d = Int_cur_d;

}

