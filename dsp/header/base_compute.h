/*
 * Beaglebone Open-Source Machine Drive
 * Compute Base Functions
 * KTH Project Work - 2018 
 */

#ifndef BASE_COMPUTE_H_		// Header wrapper name
#define BASE_COMPUTE_H_


/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */


/* ================================== FUNCTION PROTOTYPES =================== */

/* 
 * Function to convert from nominal values to p.u. base
 */
void Base_Compute(float NOMINAL_PHASE_VOLTAGE,float NOMINAL_PHASE_CURRENT,
		  float POLE_PAIRS,float NOMINAL_STATOR_FREQUENCY,
                  float SV_SCALING_CONST, float NOMINAL_STATOR_RESISTANCE,
		  float NOMINAL_ROTOR_RESISTANCE, float NOMINAL_LEAKAGE_INDUCTANCE,
                  float NOMINAL_MAGNETIZING_INDUCTANCE, float RISE_TIME_CC,
	          float NOMINAL_INERTIA, float NOMINAL_DAMPING_CONSTANT,
		  float *STATOR_RESISTANCE, float *ROTOR_RESISTANCE,
                  float *LEAKAGE_INDUCTANCE, float *MAGNETIZING_INDUCTANCE,
		  float *MECH_INERTIA_CONST, float *DAMPING_CONST,
                  float *VOLTAGE_BASE, float *VOLTAGE_MAX, float *VOLTAGE_MIN,
		  float *MAX_CURRENT,
                  float *NOM_CURRENT, float *MIN_CURRENT, float *CURR_CTRL_BANDWIDTH,
		  float *SPD_CTRL_BANDWIDTH, float *BASE_ANGULAR_FREQUENCY);


#endif
