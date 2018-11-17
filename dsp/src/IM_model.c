#include <stdio.h>
#include <stdlib.h>

/*******  Induction Motor model  ******/

typedef struct
{
    float Rs;                 // Stator resistance in dynamic T-equivalent circuit for the induction motor
    float Rr;			      // Rotor resistance in dynamic T-equivalent circuit for the induction motor
    float Ll;                 // Leakage inductance in dynamic T-equivalent circuit for the induction motor
    float Lm;                 // Magnetizing inductance in dynamic T-equivalent circuit for the induction motor
    float Ts;                 // Sampling time
    float Np;                 // Number of pole pairs
    float K;                  // Space vector scaling constant
    float J;                  // Mechanical inertia constant
    float b;                  // damping constant

}IM_Typedef;

void IM_StructInit(IM_Typedef *IM_Struct,float Rstator,float Rrotor,float Lleak,float Lmag,float Tsamp,float Npp,float SpcVectScaling,float InrConst,float DampConst);
void IM_model(IM_Typedef *IM_Struct,float Valpha,float Vbeta,float TL,float *Ialpha,float *Ibeta,float *Wr);

void IM_StructInit(IM_Typedef *IM_Struct,float Rstator,float Rrotor,float Lleak,float Lmag,float Tsamp,float Npp,float SpcVectScaling,float InrConst,float DampConst)
{
    IM_Struct->Rs = Rstator;
    IM_Struct->Rr = Rrotor;
    IM_Struct->Ll = Lleak;
    IM_Struct->Lm = Lmag;
    IM_Struct->Ts = Tsamp;
    IM_Struct->Np = Npp;
    IM_Struct->K = SpcVectScaling;
    IM_Struct->J = InrConst;
    IM_Struct->b = DampConst;
}
void IM_model(IM_Typedef *IM_Struct,float Valpha,float Vbeta,float TL,float *Ialpha,float *Ibeta,float *Wr)
{
    // Declaration

    // parameters for inverse T-equivalent circuit for induction motor

    float Ls = 0;
    float Lr = 0;
    float R_S = 0;
    float R_R = 0;
    float L_M = 0;
    float L_sigma = 0;

    // parameters for current and flux dynamics

    A = 0;
    B = 0;
    Ialpha_cur = 0;
    Ialpha_pre = 0;
    Ibeta_cur = 0;
    Ibeta_pre = 0;
    Valpha_cur = 0;
    Vbeta_cur = 0;
    psi_alpha_cur = 0;
    psi_alpha_pre = 0;
    psi_beta_cur = 0;
    psi_beta_pre = 0;

    // parameters for torque and mechanical dynamics

    C = 0;
    Te_cur = 0;
    TL_cur = 0
    Wr_cur = 0;
    Wr_pre = 0;


    // Computation

    Valpha_cur = Valpha;
    Vbeta_cur = Vbeta;
    TL_cur = TL;

    // Parameters for inverse T-equivalent circuit for the induction motor

    Ls = IM_Struct->Lm + 0.5*IM_Struct->Ll;
    Lr = IM_Struct->Lm + 0.5*IM_Struct->Ll;
    R_S = IM_Struct->Rs;
    R_R = ((IM_Struct->Lm*IM_Struct->Lm)/(Lr*Lr))*IM_Struct->Rr;
    L_M = IM_Struct->Lm*IM_Struct->Lm/Lr;
    L_sigma = Ls - L_M;

    // Current dynamics for inverse T-equivalent circuit for the induction motor

    A = 1 + (IM_Struct->Ts/L_sigma)*(R_S + R_R);

    Ialpha_cur = (1/A)*(Ialpha_pre + (IM_Struct->Ts/float L_sigma)*Valpha_cur + ((IM_Struct->Ts*R_R)/(L_sigma*L_M))*psi_beta_pre + (IM_Struct->Ts*Wr_pre/L_sigma)*psi_beta_pre);

    Ibeta_cur = (1/A)*(Ibeta_pre + (IM_Struct->Ts/float L_sigma)*Vbeta_cur + ((IM_Struct->Ts*R_R)/(L_sigma*L_M))*psi_alpha_pre - (IM_Struct->Ts*Wr_pre/L_sigma)*psi_alpha_pre);

    // Flux dynamics for inverse T-equivalent circuit for the induction motor

    B = 1 + (IM_Struct->Ts*R_R/L_M);

    psi_alpha_cur = (1/B)*(psi_alpha_pre + (IM_Struct->Ts*R_R)*Ialpha_cur - (IM_Struct->Ts*Wr_pre)*psi_beta_pre);

    psi_beta_cur = (1/B)*(psi_beta_pre + (IM_Struct->Ts*R_R)*Ibeta_cur - (IM_Struct->Ts*Wr_pre)*psi_alpha_cur);

    // Developed electromagnetic torque

    Te_cur = ((3*IM_Struct->Np)/(2*IM_Struct->K*IM_Struct->K))*(psi_alpha_cur*Ibeta_cur - psi_beta_cur*Ialpha_cur);

    // Mechanical dynamics for inverse T-equivalent circuit for the induction motor

    C = 1 + (IM_Struct->Ts*IM_Struct->b/IM_Struct->J);

    Wr_cur = (1/C)*(Wr_pre +(IM_Struct->Ts/IM_Struct->J)*Te_cur -(IM_Struct->Ts/IM_Struct->J)*TL_cur);

    // Update state variables

    Ialpha_pre = Ialpha_cur;
    Ibeta_pre = Ibeta_cur;

    psi_alpha_pre = psi_alpha_cur;
    psi_beta_pre = psi_beta_cur;

    Wr_pre = Wr_cur;

    // Output

    *Ialpha = Ialpha_cur;
    *Ibeta = Ibeta_cur;
    *Wr = Wr_cur;

}

