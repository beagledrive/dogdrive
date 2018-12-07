/*
 * Beaglebone Open-Source Machine Drive
 * SV-PWM header
 * KTH Project Work - 2018 
 */

#ifndef SV_PWM_H_		// Header wrapper name
#define SV_PWM_H_

#include <inttypes.h>

/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */

typedef struct
{
    float Ts;                  // Sampling time
    float Udc;                 // DC-link voltage
    float a;                   // Time-offset factor

} SVPWM_Typedef;


/* ================================== FUNCTION PROTOTYPES =================== */


/*
 * Initialize SV-PWM structure
 */
void SVPWM_StructInit(SVPWM_Typedef *SVPWM_Struct,float SampTime, float DClinkVolt,
			float TimeOffsetFact);

/*
 * SV-PWM generation function
 */
void SVPWM_Algorithm(SVPWM_Typedef *SVPWM_Struct,float Ua_ref,float Ub_ref,float Uc_ref,
			uint32_t *Ta_rise,uint32_t *Ta_fall,uint32_t *Tb_rise,uint32_t *Tb_fall,
			uint32_t *Tc_rise, uint32_t *Tc_fall);

#endif
