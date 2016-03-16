/*
 * Beaglebone Open-Source Machine Drive
 * D-Axis current controller
 * KTH Project Work - 2018 
 */


/* ================================== INCLUDES ============================== */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <dq_axis_current_ctrl.h>


/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */


/* ================================== FUNCTION PROTOTYPES =================== */


/* ================================== INTERNAL GLOBALS ====================== */


/* ================================== FUNCTION DEFINITIONS ================== */

void PI_StructInit(PI_Typedef *PI_Struct,float RStator,float CurrCtrlBwidth,float Tsamp,
			float L_leak, float V_max, float V_min)
{
    PI_Struct->Kp = CurrCtrlBwidth*L_leak;
    PI_Struct->Ki = CurrCtrlBwidth*RStator;
    PI_Struct->Int_pre = 0;
    PI_Struct->Ts = Tsamp;
    PI_Struct->Ra = CurrCtrlBwidth*L_leak - RStator;
    PI_Struct->Lsigma = L_leak;
    PI_Struct->V_ulim = V_max;
    PI_Struct->V_llim = V_min;

}

void PI_Controller(PI_Typedef *PI_Struct,float I_ref,float Iprim,float Isec,float w1,float *V_ref, int Is_d_axis)
{
    float Err = 0;
    float Pro_cur = 0;
    int AW_Flag = 0;
    float Int_cur = 0;
    float w1L = 0;
    float Cca_cur = 0;
    float V_temp_ref = 0;


    // Error Computation
    Err = I_ref - Iprim; 

    // Proportional part of the PI Controller
    Pro_cur = PI_Struct->Kp*Err;

    // Integral part of the PI controller
    // Depending on the status of the Anti-windup flag,
    // integral controller either keeps on integrating or freezes its value
    if(AW_Flag == 0)
    {
        Int_cur = PI_Struct->Int_pre + PI_Struct->Ts*Err;
    } else {
        Int_cur = PI_Struct->Int_pre;
    }

    // Compute cross-coupling and active resistance
    w1L = PI_Struct->Lsigma*w1;				//cross-coupling coefficient
    
    // Contribution from cross coupling and active resistance
    if (Is_d_axis == 1)
    {
	    Cca_cur = PI_Struct->Ra*Iprim + w1L*Isec;
    } else {
	    Cca_cur = PI_Struct->Ra*Iprim - w1L*Isec;
    }

    // Output of the PI controller
    // Update reference voltage including active resistance and cross coupling cancellation
    V_temp_ref = Pro_cur + PI_Struct->Ki*Int_cur - Cca_cur ;  
    
    // Saturation and anti wind-up flag
    if(V_temp_ref > PI_Struct->V_ulim)
    {
        *V_ref = PI_Struct->V_ulim;
        AW_Flag = 1;
    } else if (V_temp_ref < PI_Struct->V_llim) {
        *V_ref = PI_Struct->V_llim;
        AW_Flag = 1;
    } else {
        *V_ref = V_temp_ref;
        AW_Flag = 0;
    }

    PI_Struct->Int_pre = Int_cur;

}

