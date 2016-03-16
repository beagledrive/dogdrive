/*
 * Beaglebone Open-Source Machine Drive
 * Main Function
 * KTH Project Work - 2018 
 */


/* ================================== INCLUDES ============================== */

#include <stdio.h>
#include <stdlib.h>

#include <IM_model.h>
#include <Reference_generator.h>
#include <dq_transformation.h>
#include <dq_axis_current_ctrl.h>


/* ================================== MACROS ================================ */

/*
 * Motor Parameters
 * Later with HMI, should be made customizable through interface
 */
#define STATOR_RESISTANCE		(float)0.77	// [Ohm]
#define ROTOR_RESISTANCE		(float)1.18	// [Ohm]
#define LEAKAGE_INDUCTANCE		(float)9.84	// [mH]
#define MAGNETIZING_INDUCTANCE		(float)70.57	// [mH]
#define SAMPLING_TIME			(float)0.5	// [s]
#define POLE_PAIRS			(float)2	
#define SV_SCALING_CONST		(float)1	
#define MECH_INERTIA_CONST		(float)1
#define DAMPING_CONST			(float)1
#define VOLTAGE_MAX			(float)500	// [Volt]
#define VOLTAGE_BASE			(float)400	// [Volt]
#define VOLTAGE_MIN			(float)300	// [Volt]
#define RESERVOIR_CONST			(float)0.95	
#define ANGULAR_FREQ_BASE		(float)1500	// [RPM]
#define MAX_CURRENT			(float)1.05	// [A]
#define NOM_CURRENT			(float)1	// [A]
#define MIN_CURRENT			(float)0.1	// [A]
#define SPD_CTRL_BANDWIDTH		(float)500	// [Hz]			
#define CURR_CTRL_BANDWIDTH		(float)5000	// [Hz]i

/*
 * Define D-Q Transformation flags
 */
#define D_AXIS_CONTROL			(int)1
#define Q_AXIS_CONTROL			(int)0



/* ================================== TYPEDEFS ============================== */


/* ================================== FUNCTION PROTOTYPES =================== */


/* ================================== INTERNAL GLOBALS ====================== */


/* ================================== FUNCTION DEFINITIONS ================== */


int main(int argc, char *argv[])
{

	if (argc != 2) {
		fprintf(stderr, "Invalid arguments. Usage: ./build <num of iterations>, e.g.: ./build 3 \n");

		return 0;
	}

	int b_error = 0;

	// Induction motor model
	IM_Typedef Induction_Motor;
	// Referenge Generator structure
	RG_Typedef Reference_Generator;
	// PI controller structure for DQ axis control
	PI_Typedef PI_Control;

	// Initialize alpha/beta voltage and torque parameters
	float V_alpha = 0;
	float V_beta = 0;
	float Torque_load = 0;

	// Initialize phase currents
	float I_a = 0;
        float I_b = 0;
	float I_c = 0;	

	// Initialize phase voltages
	float V_a = 0;
	float V_b = 0;
	float V_c = 0;

	// Initialize Motor Speed
	float W_r = 0;

	// Initialize reference values for Vd Vq and Wr, overwritten by current controller
	float Vd_ref = 0;
	float Vq_ref = 0;
	float Wr_ref = 0;

	// Initialize reference D-Q currents, rotor position and rotor angular speed
	float Id_ref = 0;
	float Iq_ref = 0;
	float W1 = 0;
	float Theta1 = 0;

	// Initialize D-Q currents
	float Id = 0;
	float Iq = 0;

	// Initialize Induction motor
	IM_StructInit(&Induction_Motor, STATOR_RESISTANCE, ROTOR_RESISTANCE, LEAKAGE_INDUCTANCE, 
			MAGNETIZING_INDUCTANCE,SAMPLING_TIME, POLE_PAIRS, SV_SCALING_CONST,
			MECH_INERTIA_CONST,DAMPING_CONST);

	// Initialize Reference generator
	RG_StructInit(&Reference_Generator, VOLTAGE_BASE, RESERVOIR_CONST, ANGULAR_FREQ_BASE,
		   	ROTOR_RESISTANCE, LEAKAGE_INDUCTANCE, MAGNETIZING_INDUCTANCE,
		    	SAMPLING_TIME, MECH_INERTIA_CONST,POLE_PAIRS, SV_SCALING_CONST,
		       	DAMPING_CONST,SPD_CTRL_BANDWIDTH, MAX_CURRENT, MIN_CURRENT, NOM_CURRENT);

	// Initialize PI controller - check KP KI, RACTIVE VMIN and VMAX Parameters
	PI_StructInit(&PI_Control, MECH_INERTIA_CONST, POLE_PAIRS, SV_SCALING_CONST,
		       	SPD_CTRL_BANDWIDTH, SAMPLING_TIME, ROTOR_RESISTANCE,
		       	LEAKAGE_INDUCTANCE, VOLTAGE_MAX, VOLTAGE_MIN);

	// Iteration variable: number of cycles to run
	int i = 0;
	int lim = atoi(argv[1]);
	while(!b_error) 
	{
		// Simulate Induction Motor
		fprintf(stderr, "IM_model input: V_alpha = %f, V_beta = %f, Torque_Load = %f\n",
			       	V_alpha, V_beta, Torque_load);
		IM_model(&Induction_Motor, V_alpha, V_beta, Torque_load,
				&I_a, &I_b, &I_c, &W_r);
		fprintf(stderr, "IM_model output: I_a = %f, I_b = %f, I_c = %f, W_r = %f\n\n",
			       	I_a, I_b, I_c, W_r);

		// Run Reference Generator
		fprintf(stderr, "RG_control input: Vd_ref = %f, Vq_ref = %f, Wr_ref = %f\n",
			       	Vd_ref, Vq_ref, Wr_ref);
		RG_Controller(&Reference_Generator, Vd_ref, Vq_ref, Wr_ref,
			    	W_r, &Id_ref, &Iq_ref, &W1, &Theta1);
		fprintf(stderr, "RG_control output: Id_ref = %f, Iq_ref = %f, W1 = %f, Theta1 = %f\n\n",
			       	Id_ref, Iq_ref, W1, Theta1);
		
		// Run DQ_transformation algorithm
		fprintf(stderr, "DQ_transf input: I_a = %f, I_b = %f, I_c = %f, theta1 = %f\n",
				I_a, I_b, I_c, Theta1);
		DQ_Transformation(SV_SCALING_CONST, I_a, I_b, I_c, Theta1, &Id, &Iq);
		fprintf(stderr, "DQ_transf output: Id = %f, I_q = %f\n\n",
				Id, Iq);

		// Run D-axis control
		fprintf(stderr, "D-axis Control Input: I_ref(d) = %f, Iprim(d) = %f, Isec(q) = %f, w1 = %f\n",
				Id_ref, Id, Iq, W1);
		PI_Controller(&PI_Control, Id_ref, Id, Iq, W1, &Vd_ref, D_AXIS_CONTROL);
		fprintf(stderr, "D-axis Control Output: Vd_ref = %f\n", 
				Vd_ref);	

		// Run Q-axis control
		fprintf(stderr, "Q-axis Control Input: I_ref(q) = %f, Iprim(q) = %f, Isec(d) = %f, w1 = %f\n",
				Iq_ref, Iq, Id, W1);
		PI_Controller(&PI_Control, Iq_ref, Iq, Id, W1, &Vq_ref, Q_AXIS_CONTROL);
		fprintf(stderr, "Q-axis Control Output: Vq_ref = %f\n\n", 
				Vq_ref);

		// Run IDQ_transformation algorithm
		fprintf(stderr, "IDQ_transf input: Vd = %f, Vq = %f, theta1 = %f\n",
				Vd_ref, Vq_ref, Theta1);
		IDQ_Transformation(SV_SCALING_CONST, Vd_ref, Vq_ref, Theta1,
					&V_alpha, &V_beta, &V_a, &V_b, &V_c);
		fprintf(stderr, "IDQ_transf output: Valpha = %f, Vbeta = %f, Va = %f, Vb = %f, Vc = %f\n\n",
				V_alpha, V_beta, V_a, V_b, V_c);

		fprintf(stderr, "=========================================================================\n\n");
		
		// Control loop length for testing
		i++;
		if (i == lim) { b_error = 1; }	
	}

	return -1;
}
