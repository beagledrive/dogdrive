#include <stdio.h>
#include <stdlib.h>

/*******  Reference Generator ******/
// This C script includes field weakening, flux estimation and speed regulation
// Flux estimation is done using current model and IFO
// Speed regulation incorporates active viscous damping and anti windup

typedef struct
{
    // Base value of voltage and angular frequency of the output voltage of the inverter

    float Vbase;              // Base voltage
    float wbase;              // Base angular frequency of the output voltage of the inverter

    // Machine parameters and sampling time period

    float R_R;                // Rotor resistance in inverse T-equivalent circuit of the induction motor
    float Lsigma;             // Leakage inductance in inverse T-equivalent circuit of the induction motor
    float L_M;                // Magnetizing inductance in inverse T_equivalent circuit of the induction motor
    float Ts;                 // Sampling time

    // Speed controller parameters

    float Kps_c;              // Constant part of proportional gain of the speed controller
    float Kis_c;			  // Constant part of integral gain of the speed controller
    float ba_c;               // Constant part of active viscous damping
    float Id_pre;	          // Previous value of the Integral output of the field weakening controller
    float psi_pre;            // Previous flux
    float Iq_pre;	          // Previous value of the Integral output of the speed controller

    // Speed controller saturation limits

    float Imax;               // Maximum current
    float Imin;               // Minimum current
    float Inom;               // Nominal current

}RG_Typedef;

void RG_StructInit(RG_Typedef *RG_Struct,float BaseVolt,float BaseAngularspeed,float RotorRes,float LeakageInd,float MagnetizingInd,float Tsamp,float PropCoeffi,float IntCoffi,float b_active,float MaxCur,float MinCur, float NomCur);
void RG_Controller(RG_Typedef *RG_Struct,float Vd_ref,float Vq_ref,float Wr_ref,float Wr,float w1,float *Id_ref,float *Iq_ref);

void RG_StructInit(PI_Typedef *PI_Struct,float PropCoeffi,float IntCoffi,float Tsamp,float R_active,float L_leak)
{
    RG_Struct->Vbase = BaseVolt;
    RG_Struct->wbase = BaseAngularspeed;
    RG_Struct->R_R = RotorRes;
    RG_Struct->Lsigma = LeakageInd;
    RG_Struct->L_M = MagnetizingInd;
    RG_Struct->Ts = Tsamp;
    RG_Struct->Kps_c = PorpCoeffi;
    RG_Struct->Kis_c = IntCoffi;
    RG_Struct->ba_c = b_active;
    RG_Struct->Id_pre = 0;
    RG_Struct->psi_pre = 0;
    RG_Struct->Iq_pre = 0;
    RG_Struct->Imax = MaxCur;
    RG_Struct->Imin = MinCur;
    RG_Struct->Inom = NomCur;

}
void RG_Controller(RG_Typedef *RG_Struct,float Vd_ref,float Vq_ref,float Wr_ref,float Wr,float w1,float *Id_ref,float *Iq_ref)
{
    float Verr = 0;
    float w_max = 0;
    float Kfw_den = 0;
    float Kfw = 0;
    float Id_cur = 0;
    float psi_den = 0;
    float psi_cur = 0;
    float kps = 0;
    float kis = 0;
    float ba = 0;
    float es = 0;
    float Iq_cur = 0;

    // Field weakening control

    Verr = RG_Struct->Vbase*RG_Struct->Vbase - Vd_ref*Vd_ref - Vq_ref*Vq_ref;

    if(w1 >= RG_Struct->wbase)
    {
        w_max = w1;
    }
    else
    {
        w_max = wbase;
    }

    Kfw_den = RG_Struct->Lsigma*RG_Struct->Lsigma*RG_Struct->Vbase*w_max;

    Kfw = RG_Struct->Ts*RG_Struct->R_R/Kfw_den;

    Id_cur = Id_pre + Kfd*Verr;

    if (Id_cur <= RG_Struct->Imin)
    {
        Id_cur = Imin;
    }

    if (Id_cur >= RG_Struct->Inom)
    {
        Id_cur = Inom;
    }

    *Id_ref = Id_cur;

    RG_Struct->Id_pre = *Id_ref;

    // Rotor flux estimation using current model and IFO

    psi_den = 1 + RG_Struct->Ts*RG_Struct->R_R/RG_Struct->Lsigma;

    psi_cur = RG_Struct->Ts*RG_Struct->R_R*(*Id_ref)/psi_den + psi_pre/psi_den;

    psi_pre = psi_cur;

    // Speed regulator

    kps = RG_Struct->Kps_c/psi_cur;

    kis = RG_Struct->Kis_c/psi_cur;

    ba = RG_Struct->ba_c/psi_cur;

    es = Wr_ref - Wr;

    Iq_cur = RG_Struct->Iq_pre + RG_Struct->Ts*es;

    Iq_ref_nom = kps*es + kis*Iq_cur - ba*wr;

    zita = (RG_Struct->Lsigma + RG_Struct->L_M)/RG_Struct->Lsigma;

    Iq_ref_max = sqrt(RG_Struct->Imax*RG_Struct->Imax - (*Id_ref)*(*Id_ref) );

    if (Iq_ref_max >= zita*(*Id_ref))
    {
        Iq_ref_max = zita*(*Id_ref);
    }

    *Iq_ref = Iq_ref_nom;

    if (*Iq_ref >= Iq_ref_max)
    {
        *Iq_ref = Iq_ref_max;
    }

    if (*Iq_ref == Iq_ref_nom)
    {
        RG_Struct->Iq_pre = Iq_cur;
    }
    else
    {
       RG_Struct->Iq_pre = 0;
    }

}

