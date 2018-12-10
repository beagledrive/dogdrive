/* Beaglebone Open-Source Machine Drive
 * PWM Space Vector Modulation function
 * KTH Project Work - 2018 
 */


/* ================================== INCLUDES ============================== */

#include <svpwm.h>

/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */


/* ================================== FUNCTION PROTOTYPES =================== */


/* ================================== INTERNAL GLOBALS ====================== */


/* ================================== FUNCTION DEFINITIONS ================== */

void SVPWM_StructInit(SVPWM_Typedef *SVPWM_Struct,float SampTime,float DClinkVolt,
			float TimeOffsetFact)
{
    SVPWM_Struct->Ts = SampTime;
    SVPWM_Struct->Udc = DClinkVolt;
    SVPWM_Struct->a = TimeOffsetFact;

}

void SVPWM_Algorithm(SVPWM_Typedef *SVPWM_Struct,float Ua_ref,float Ub_ref,float Uc_ref,
			uint32_t *Ta_rise,uint32_t *Ta_fall,uint32_t *Tb_rise,uint32_t *Tb_fall,
			uint32_t *Tc_rise, uint32_t *Tc_fall)
{
    float Tas = 0;
    float Tbs = 0;
    float Tcs = 0;
    float Tmax = 0;
    float Tmin = 0;
    float Teff = 0;
    float T0 = 0;
    float Toffset = 0;
    float Tga = 0;
    float Tgb = 0;
    float Tgc = 0;

    // Computation of the imaginary time
    Tas = (SVPWM_Struct->Ts/SVPWM_Struct->Udc)*Ua_ref;
    Tbs = (SVPWM_Struct->Ts/SVPWM_Struct->Udc)*Ub_ref;
    Tcs = (SVPWM_Struct->Ts/SVPWM_Struct->Udc)*Uc_ref;

    // Computing minimum time
    Tmin = Tas;

    if(Tmin > Tbs)
    {
        Tmin = Tbs;
    }

    if(Tmin > Tcs)
    {
        Tmin = Tcs;
    }

    // Computing maximum time
    Tmax = Tas;

    // Fixing needed

    if(Tmax < Tbs)
    {
	Tmax = Tbs;
    }

    if(Tmax < Tcs)
    {
        Tmax = Tcs;
    }

    // Computing effective time and zero time
    Teff = (Tmax - Tmin);

    T0 = (SVPWM_Struct->Ts - Teff) ;

    Toffset = -Tmin + SVPWM_Struct->a*T0;

    // Computing duration of actual gating pulse
    Tga = Tas + Toffset;
    Tgb = Tbs + Toffset;
    Tgc = Tcs + Toffset;


    // Computing rising edge and falling edge of the PWM
    // FIXME - Handle unexpected error case when cal. times are negative
    // FIXME - Handle units
    *Ta_rise = (uint32_t)(((SVPWM_Struct->Ts/2)-(Tga/2))*1000000000/5);
    *Ta_fall = (uint32_t)(((SVPWM_Struct->Ts/2)+(Tga/2))*1000000000/5);

    *Tb_rise = (uint32_t)((SVPWM_Struct->Ts/2)-(Tgb/2));
    *Tb_fall = (uint32_t)((SVPWM_Struct->Ts/2)+(Tgb/2));

    *Tc_rise = (uint32_t)((SVPWM_Struct->Ts/2)-(Tgc/2));
    *Tc_fall = (uint32_t)((SVPWM_Struct->Ts/2)+(Tgc/2));

}

