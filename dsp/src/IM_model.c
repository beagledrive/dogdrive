/* Beaglebone Open-Source Machine Drive
 * Induction motor model used to test control algorithm
 * KTH Project Work - 2018 
 */


/* ================================== INCLUDES ============================== */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <IM_model.h>


/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */


/* ================================== FUNCTION PROTOTYPES =================== */


/* ================================== INTERNAL GLOBALS ====================== */


/* ================================== FUNCTION DEFINITIONS ================== */

void IM_StructInit(IM_Typedef *IM_Struct,float Rstator,float Rrotor,float Lleak,
			float Lmag,float Tsamp,float Npp,float SpcVectScaling,
			float InrConst,float DampConst)
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
    IM_Struct->psi_a_pre = 0;
    IM_Struct->psi_b_pre = 0;
}

void IM_model(IM_Typedef *IM_Struct,float Valpha,float Vbeta,float TL,
			float *Ia,float *Ib,float *Ic, float *Ialpha,
		        float *Ibeta, float *TM_cur, float *Wr)
{

    // Parameters for inverse T-equivalent circuit for induction motor
    float R_S = 0;
    float R_R = 0;
    float L_M = 0;
    float L_sigma = 0;

    // parameters for current and flux dynamics
    float A = 0;
    float B = 0;
    float Ialpha_cur = 0;
    float Ialpha_pre = 0;
    float Ibeta_cur = 0;
    float Ibeta_pre = 0;
    float Valpha_cur = 0;
    float Vbeta_cur = 0;
    float psi_alpha_cur = 0;
    float psi_alpha_pre = 0;
    float psi_beta_cur = 0;
    float psi_beta_pre = 0;

    // parameters for torque and mechanical dynamics
    float C = 0;
    float Te_cur = 0;
    float TL_cur = 0;
    float Wr_cur = 0;
    float Wr_pre = 0;


    Ialpha_pre = *Ialpha;
    Ibeta_pre = *Ibeta;
    Wr_pre = *Wr;
    psi_alpha_pre = IM_Struct->psi_a_pre;
    psi_beta_pre = IM_Struct->psi_b_pre;

    // Computation
    Valpha_cur = Valpha;
    Vbeta_cur = Vbeta;
    TL_cur = TL;

    // Parameters for inverse T-equivalent circuit for the induction motor
    R_S = IM_Struct->Rs;
    R_R = IM_Struct->Rr;
    L_M = IM_Struct->Lm;
    L_sigma = IM_Struct->Ll;

    // Current dynamics for inverse T-equivalent circuit for the induction motor
    A = 1 + (IM_Struct->Ts/L_sigma)*(R_S + R_R);

    Ialpha_cur = (1/A)*(Ialpha_pre + (IM_Struct->Ts/L_sigma)*Valpha_cur 
		    + ((IM_Struct->Ts*R_R)/(L_sigma*L_M))*psi_beta_pre 
		    + (IM_Struct->Ts*Wr_pre/L_sigma)*psi_beta_pre);

    Ibeta_cur = (1/A)*(Ibeta_pre + (IM_Struct->Ts/L_sigma)*Vbeta_cur 
		    + ((IM_Struct->Ts*R_R)/(L_sigma*L_M))*psi_alpha_pre 
		    - (IM_Struct->Ts*Wr_pre/L_sigma)*psi_alpha_pre);

    // Flux dynamics for inverse T-equivalent circuit for the induction motor
    B = 1 + (IM_Struct->Ts*R_R/L_M);

    psi_alpha_cur = (1/B)*(psi_alpha_pre + (IM_Struct->Ts*R_R)*Ialpha_cur 
		    - (IM_Struct->Ts*Wr_pre)*psi_beta_pre);

    psi_beta_cur = (1/B)*(psi_beta_pre + (IM_Struct->Ts*R_R)*Ibeta_cur 
		    - (IM_Struct->Ts*Wr_pre)*psi_alpha_cur);

    // Developed electromagnetic torque
    Te_cur = (psi_alpha_cur*Ibeta_cur - psi_beta_cur*Ialpha_cur);

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
    *Wr = Wr_cur;

    *Ia = Ialpha_cur ;
    *Ib = -0.5*Ialpha_cur+(sqrt(3)/2)*Ibeta_cur;
    *Ic = -0.5*Ialpha_cur-(sqrt(3)/2)*Ibeta_cur;

    *Ialpha = Ialpha_cur;
    *Ibeta = Ibeta_cur;
    IM_Struct->psi_a_pre = psi_alpha_cur;
    IM_Struct->psi_b_pre = psi_beta_cur;
    *TM_cur = Te_cur;
}

