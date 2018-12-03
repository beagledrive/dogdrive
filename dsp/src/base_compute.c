#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <base_compute.h>

#define PI 3.14159265358979323846

/*******  Base Value Computation ******/

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
		  float *ANGULAR_FREQ_BASE, float *MAX_CURRENT,
                  float *NOM_CURRENT, float *MIN_CURRENT, float *CURR_CTRL_BANDWIDTH,
		  float *SPD_CTRL_BANDWIDTH, float *BASE_ANGULAR_FREQUENCY)
    {
        float Ubase, Ibase, Pbase,wbase, Zbase, Rbase, Lbase, Tbase, Jbase, bbase, tbase;

        Ubase = sqrt(2)*SV_SCALING_CONST*NOMINAL_PHASE_VOLTAGE;
        Ibase = sqrt(2)*SV_SCALING_CONST*NOMINAL_PHASE_CURRENT;
        wbase = 2*PI*NOMINAL_STATOR_FREQUENCY;
        Pbase = (3/(2*SV_SCALING_CONST*SV_SCALING_CONST))*Ubase*Ibase;
        Zbase = Ubase/Ibase;
        Rbase = Zbase;
        Lbase = Zbase/wbase;
        Tbase = POLE_PAIRS*Pbase/wbase;
        Jbase = POLE_PAIRS*Tbase/(wbase*wbase);
        bbase = POLE_PAIRS*Tbase/wbase;
        tbase = 1/wbase;

        *STATOR_RESISTANCE = NOMINAL_STATOR_RESISTANCE/Rbase;
        *ROTOR_RESISTANCE = NOMINAL_ROTOR_RESISTANCE/Rbase;
        *LEAKAGE_INDUCTANCE = NOMINAL_LEAKAGE_INDUCTANCE/Lbase;
        *MAGNETIZING_INDUCTANCE = NOMINAL_MAGNETIZING_INDUCTANCE/Lbase;
        *MECH_INERTIA_CONST = NOMINAL_INERTIA/Jbase;
        *DAMPING_CONST = NOMINAL_DAMPING_CONSTANT/bbase;
        *VOLTAGE_BASE = Ubase;
        *VOLTAGE_MAX = 1.1*Ubase;
        *VOLTAGE_MIN = 0.1*Ubase;
        *ANGULAR_FREQ_BASE = wbase;
        *MAX_CURRENT = 1.1*Ibase;
        *NOM_CURRENT = Ibase;
        *MIN_CURRENT = 0.1*Ibase;
        *CURR_CTRL_BANDWIDTH = 2.1972/(RISE_TIME_CC/tbase);
        *SPD_CTRL_BANDWIDTH = (*CURR_CTRL_BANDWIDTH)/20;
	*BASE_ANGULAR_FREQUENCY = wbase;
    }
