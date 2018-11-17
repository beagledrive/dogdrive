#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <tralgo.h>


#define PI 3.14159265358979323846  // I have to check this

/*******  Reference Generator ******/
// This C script includes field weakening, flux estimation, speed regulation and rotor position estimation
// Flux estimation is done using current model and IFO
// Speed regulation incorporates active viscous damping and anti windup

typedef struct
{
    // Base value of voltage and angular frequency of the output voltage of the inverter

    float Vbase;              // Base voltage
    float K_reservoir;        // Reservoir constant
    float Wbase;              // Base angular frequency of the output voltage of the inverter
    float W1_pre;             // Previous value of w1

    // Machine parameters and sampling time period

    float R_R;                // Rotor resistance in inverse T-equivalent circuit of the induction motor
    float Lsigma;             // Leakage inductance in inverse T-equivalent circuit of the induction motor
    float L_M;                // Magnetizing inductance in inverse T_equivalent circuit of the induction motor
    float Ts;                 // Sampling time

    // Speed controller parameters

    float Kps_c;              // Constant part of proportional gain of the speed controller
    float Kis_c;			  // Constant part of integral gain of the speed controller
    float ba_c;               // Constant part of active viscous damping
    float int_fw_pre;	      // Previous value of the Integral output of the field weakening controller
    float psi_pre;            // Previous flux
    float int_s_pre;	      // Previous value of the Integral output of the speed controller
    float theta1_pre;         // Previous value of rotor position

    // Speed controller saturation limits

    float Imax;               // Maximum current
    float Imin;               // Minimum current
    float Inom;               // Nominal current

}RG_Typedef;

void RG_StructInit(RG_Typedef *RG_Struct,float BaseVolt,float ResCons,float BaseAngularspeed,
			float RotorRes,float LeakageInd,float MagnetizingInd,float Tsamp,
			float PropCoeffi,float IntCoffi,float b_active,float MaxCur,
			float MinCur, float NomCur);

void RG_Controller(RG_Typedef *RG_Struct,float Vd_ref,float Vq_ref,float Wr_ref,float Wr,
			float *Id_ref,float *Iq_ref,float *W1,float *theta1);

void RG_StructInit(RG_Typedef *RG_Struct,float BaseVolt,float ResCons,float BaseAngularspeed,
			float RotorRes,float LeakageInd,float MagnetizingInd,float Tsamp,
			float PropCoeffi,float IntCoffi,float b_active,float MaxCur,
			float MinCur, float NomCur)
{
    RG_Struct->Vbase = BaseVolt;
    RG_Struct->K_reservoir = ResCons;
    RG_Struct->Wbase = BaseAngularspeed;
    RG_Struct->W1_pre = 0;
    RG_Struct->R_R = RotorRes;
    RG_Struct->Lsigma = LeakageInd;
    RG_Struct->L_M = MagnetizingInd;
    RG_Struct->Ts = Tsamp;
    RG_Struct->Kps_c = PropCoeffi;
    RG_Struct->Kis_c = IntCoffi;
    RG_Struct->ba_c = b_active;
    RG_Struct->int_fw_pre = 0;
    RG_Struct->psi_pre = 0;
    RG_Struct->int_s_pre = 0;
    RG_Struct->theta1_pre = 0;
    RG_Struct->Imax = MaxCur;
    RG_Struct->Imin = MinCur;
    RG_Struct->Inom = NomCur;

}
void RG_Controller(RG_Typedef *RG_Struct,float Vd_ref,float Vq_ref,float Wr_ref,
			float Wr,float *Id_ref,float *Iq_ref,float *W1,float *theta1 )
{
    float Verr = 0;
    float W_max = 0;
    float Kfw_den = 0;
    float K_fw = 0;
    float Id_cur = 0;
    float psi_den = 0;
    float psi_cur = 0;
    float kps = 0;
    float kis = 0;
    float ba = 0;
    float es = 0;
    float pro_s = 0;
    float int_s_cur = 0;
    float AW_Flag = 0;
    float ar_s = 0;
    float Iq_ref_nom = 0;
    float zita = 0;
    float Iq_ref_max = 0;

    // Field weakening control - d-axis current reference generation
    // Field weakening control is integral control

    // Compute error in square of the voltage

    Verr = RG_Struct->Vbase*RG_Struct->Vbase*RG_Struct->K_reservoir*RG_Struct->K_reservoir - Vd_ref*Vd_ref - Vq_ref*Vq_ref;

    // Compute w_max = max (w_base, w1)

    if(RG_Struct->W1_pre >= RG_Struct->Wbase)
    {
        W_max = RG_Struct->W1_pre;
    }
    else
    {
        W_max = RG_Struct->Wbase;
    }

    // Integral gain of the field weakening control

    Kfw_den = RG_Struct->Lsigma*RG_Struct->Lsigma*RG_Struct->Vbase*W_max;

    K_fw = RG_Struct->Ts*RG_Struct->R_R/Kfw_den;

    Id_cur = RG_Struct->int_fw_pre + K_fw*Verr;

    if (Id_cur < RG_Struct->Imin)
    {
        Id_cur = RG_Struct->Imin;
    }

    if (Id_cur > RG_Struct->Inom)
    {
        Id_cur = RG_Struct->Inom;
    }

    *Id_ref = Id_cur;

    RG_Struct->int_fw_pre = *Id_ref;

    // Rotor flux estimation using current model and IFO

    psi_den = 1 + RG_Struct->Ts*RG_Struct->R_R/RG_Struct->L_M;

    psi_cur = RG_Struct->Ts*RG_Struct->R_R*(*Id_ref)/psi_den + RG_Struct->psi_pre/psi_den;

    RG_Struct->psi_pre = psi_cur;


    // Speed regulator proportional, integral and active resistance coefficients

    kps = RG_Struct->Kps_c/psi_cur;

    kis = RG_Struct->Kis_c/psi_cur;

    ba = RG_Struct->ba_c/psi_cur;

    // error in speed

    es = Wr_ref - Wr;

    // Proportional part of the speed regulator

    pro_s = kps*es;

    // Integral part of the speed regulator

    if(AW_Flag == 0)
    {
        int_s_cur = RG_Struct->int_s_pre + RG_Struct->Ts*es;
    }

    else
    {
        int_s_cur = RG_Struct->int_s_pre;
    }

    // Active resistance part of the speed regulator

    ar_s = ba*Wr;

    // Output of the speed regulator

    Iq_ref_nom = pro_s + kis*int_s_cur - ar_s;

    // Computation of maximum q-axis current reference

    zita = (RG_Struct->Lsigma + RG_Struct->L_M)/RG_Struct->Lsigma;

    Iq_ref_max = sqrt(RG_Struct->Imax*RG_Struct->Imax - (*Id_ref)*(*Id_ref) );

    if (Iq_ref_max >= zita*(*Id_ref))
    {
        Iq_ref_max = zita*(*Id_ref);
    }

    // Saturation and anti wind-up flag

    if (Iq_ref_nom >= Iq_ref_max)
    {
        *Iq_ref = Iq_ref_max;
        AW_Flag = 1;
    }
    else
    {
        *Iq_ref = Iq_ref_nom;
        AW_Flag = 0;
    }

    RG_Struct->int_s_pre = int_s_cur;

    // Rotor position estimation

    *W1 = Wr + RG_Struct->R_R*(*Iq_ref)/psi_cur;

    *theta1 = RG_Struct->theta1_pre + RG_Struct->Ts*(*W1);

    if(*theta1 > 2*PI)
    {
        *theta1 = 0;
    }

    if (*theta1 < 0)
    {
        *theta1 = 0;
    }

    RG_Struct->W1_pre = *W1;

    RG_Struct->theta1_pre = *theta1;
}

