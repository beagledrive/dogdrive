/*
 * Beaglebone Open-Source Machine Drive
 * Main Function
 * KTH Project Work - 2018 
 */


/* ================================== INCLUDES ============================== */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include <IM_model.h>
#include <Reference_generator.h>
#include <dq_transformation.h>
#include <dq_axis_current_ctrl.h>
#include <base_compute.h>


/* ================================== MACROS ================================ */

/*
 * Motor Parameters
 * Later with HMI, should be made customizable through interface
 */
#define NOMINAL_PHASE_VOLTAGE		(float)127	// [Volta]
#define NOMINAL_PHASE_CURRENT		(float)7.8	// [A]
#define POLE_PAIRS			(float)2	
#define NOMINAL_STATOR_FREQUENCY	(float)50	// [Hz]
#define SV_SCALING_CONST		(float)0.707
#define NOMINAL_STATOR_RESISTANCE	(float)0.77	// [Ohm]
#define NOMINAL_ROTOR_RESISTANCE	(float)1.03	// [Ohm]
#define NOMINAL_LEAKAGE_INDUCTANCE	(float)0.0095	// [H]
#define NOMINAL_MAGNETIZING_INDUCTANCE	(float)0.066	// [H]
#define RISE_TIME_CC			(float)0.001	// [s]
#define NOMINAL_INERTIA			(float)0.0192
#define NOMINAL_DAMPING_CONST		(float)0.0024
#define SAMPLING_TIME			(float)0.0001	// [s]
#define RESERVOIR_CONST			(float)0.95


/*
 * Torque Load in pu
 */
#define TORQUE_LOAD			(float)0.6	// [p.u.]

/*
 * Define D-Q Transformation flags
 */
#define D_AXIS_CONTROL			(int)1
#define Q_AXIS_CONTROL			(int)0



/* ================================== TYPEDEFS ============================== */


/* ================================== FUNCTION PROTOTYPES =================== */


/* ================================== INTERNAL GLOBALS ====================== */


/*
 * Noramlized P.U. values of motor parameters
 */
static float Stator_Resistance = 0;
static float Rotor_Resistance = 0;
static float Leakage_Inductance = 0;
static float Magnetizing_Inductance = 0;
static float Mech_Inertia_Const = 0;
static float Damping_Const = 0;
static float Voltage_Base = 0;
static float Voltage_Max = 0;
static float Voltage_Min = 0;
static float Angular_Freq_Base = 0;
static float Max_Current = 0;
static float Nom_Current = 0;
static float Min_Current = 0;
static float Curr_Ctrl_Bandwidth = 0;
static float Spd_Ctrl_Bandwidth = 0;
static float Base_Ang_Freq = 0;



/* ================================== FUNCTION DEFINITIONS ================== */


int main(int argc, char *argv[])
{

	if (argc != 2) {
		fprintf(stderr, "Invalid arguments. Usage: ./build <num of iterations>, e.g.: ./build 3 \n");

		return 0;
	}

	int b_error = 0;

	// Get TimeStamp
	struct timeval tv;

	// File to write CSV output
	const char *fileName = "output.csv";
	FILE *fd = fopen(fileName, "w");
	// clear content of file first, then open for writing
	if(fd == NULL)
	{
		fprintf(stderr, "Unable to open file! \n");
		b_error = 1;
	} else {
		fclose(fd);
	}

	// Open file for writing
	fd = fopen(fileName, "a");
	if(fd == NULL)
	{
		fprintf(stderr, "Unable to open file! \n");
		b_error = 1;
	}


	// Induction motor model
	IM_Typedef Induction_Motor;
	// Referenge Generator structure
	RG_Typedef Reference_Generator;
	// PI controller structure for DQ axis control
	PI_Typedef PI_Control;


	// Initialize alpha/beta voltage, current and torque parameters
	float V_alpha = 0;
	float V_beta = 0;
	float I_alpha = 0;
	float I_beta = 0;
	float TM_cur = 0;

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

	// Compute Normalized values
	Base_Compute(NOMINAL_PHASE_VOLTAGE, NOMINAL_PHASE_CURRENT, POLE_PAIRS,
			NOMINAL_STATOR_FREQUENCY, SV_SCALING_CONST, NOMINAL_STATOR_RESISTANCE,
		        NOMINAL_ROTOR_RESISTANCE, NOMINAL_LEAKAGE_INDUCTANCE, NOMINAL_MAGNETIZING_INDUCTANCE,
		        RISE_TIME_CC, NOMINAL_INERTIA, NOMINAL_DAMPING_CONST,
			&Stator_Resistance, &Rotor_Resistance, &Leakage_Inductance, &Magnetizing_Inductance,
			&Mech_Inertia_Const, &Damping_Const, &Voltage_Base, &Voltage_Max,
			&Voltage_Min, &Angular_Freq_Base, &Max_Current, &Nom_Current, &Min_Current,
			&Curr_Ctrl_Bandwidth, &Spd_Ctrl_Bandwidth, &Base_Ang_Freq);

	// Initialize Induction motor
	IM_StructInit(&Induction_Motor, Stator_Resistance, Rotor_Resistance, Leakage_Inductance, 
			Magnetizing_Inductance,SAMPLING_TIME, POLE_PAIRS, SV_SCALING_CONST,
			Mech_Inertia_Const,Damping_Const);

	// Initialize Reference generator
	RG_StructInit(&Reference_Generator, Voltage_Base, RESERVOIR_CONST, Base_Ang_Freq,
		   	Rotor_Resistance, Leakage_Inductance, Magnetizing_Inductance,
		    	SAMPLING_TIME, Mech_Inertia_Const,POLE_PAIRS, SV_SCALING_CONST,
		       	Damping_Const,Spd_Ctrl_Bandwidth, Max_Current, Min_Current, Nom_Current);

	// Initialize PI controller - check KP KI, RACTIVE VMIN and VMAX Parameters
	PI_StructInit(&PI_Control, Stator_Resistance, Curr_Ctrl_Bandwidth,SAMPLING_TIME,
		       	Leakage_Inductance, Voltage_Max, Voltage_Min);

	// Iteration variable: number of cycles to run
	int i = 0;
	int lim = atoi(argv[1]);
	while(!b_error) 
	{

		// Control loop length for testing
		if (i == lim) { b_error = 1; break; }
		i++;

		// Simulate Induction Motor
		//fprintf(stderr, "IM_model input: V_alpha = %f, V_beta = %f, Torque_Load = %f\n",
		//	       	V_alpha, V_beta, TORQUE_LOAD);
		IM_model(&Induction_Motor, V_alpha, V_beta, TORQUE_LOAD,
				&I_a, &I_b, &I_c, &I_alpha, &I_beta, &TM_cur, &W_r);
		//fprintf(stderr, "IM_model output: I_a = %f, I_b = %f, I_c = %f, W_r = %f\n\n",
		//	       	I_a, I_b, I_c, W_r);

		// Run Reference Generator
		//fprintf(stderr, "RG_control input: Vd_ref = %f, Vq_ref = %f, Wr_ref = %f\n",
		//	       	Vd_ref, Vq_ref, Wr_ref);
		RG_Controller(&Reference_Generator, Vd_ref, Vq_ref, Wr_ref,
			    	W_r, &Id_ref, &Iq_ref, &W1, &Theta1);
		//fprintf(stderr, "RG_control output: Id_ref = %f, Iq_ref = %f, W1 = %f, Theta1 = %f\n\n",
		//	       	Id_ref, Iq_ref, W1, Theta1);
		
		// Run DQ_transformation algorithm
		//fprintf(stderr, "DQ_transf input: I_a = %f, I_b = %f, I_c = %f, theta1 = %f\n",
		//		I_a, I_b, I_c, Theta1);
		DQ_Transformation(SV_SCALING_CONST, I_a, I_b, I_c, Theta1, &Id, &Iq);
		//fprintf(stderr, "DQ_transf output: Id = %f, I_q = %f\n\n",
		//		Id, Iq);

		// Run D-axis control
		//fprintf(stderr, "D-axis Control Input: I_ref(d) = %f, Iprim(d) = %f, Isec(q) = %f, w1 = %f\n",
		//		Id_ref, Id, Iq, W1);
		PI_Controller(&PI_Control, Id_ref, Id, Iq, W1, &Vd_ref, D_AXIS_CONTROL);
		//fprintf(stderr, "D-axis Control Output: Vd_ref = %f\n", 
		//		Vd_ref);	

		// Run Q-axis control
		//fprintf(stderr, "Q-axis Control Input: I_ref(q) = %f, Iprim(q) = %f, Isec(d) = %f, w1 = %f\n",
		//		Iq_ref, Iq, Id, W1);
		PI_Controller(&PI_Control, Iq_ref, Iq, Id, W1, &Vq_ref, Q_AXIS_CONTROL);
		//fprintf(stderr, "Q-axis Control Output: Vq_ref = %f\n\n", 
		//		Vq_ref);

		// Run IDQ_transformation algorithm
		//fprintf(stderr, "IDQ_transf input: Vd = %f, Vq = %f, theta1 = %f\n",
		//		Vd_ref, Vq_ref, Theta1);
		IDQ_Transformation(SV_SCALING_CONST, Vd_ref, Vq_ref, Theta1,
					&V_alpha, &V_beta, &V_a, &V_b, &V_c);
		//fprintf(stderr, "IDQ_transf output: Valpha = %f, Vbeta = %f, Va = %f, Vb = %f, Vc = %f\n\n",
		//		V_alpha, V_beta, V_a, V_b, V_c);

		//fprintf(stderr, "=========================================================================\n\n");


		gettimeofday(&tv,NULL);
   		fprintf(fd, "%f,%f,%f,%f,%f,%f,%f,%f,%ld,%ld\n",V_alpha,V_beta,I_alpha,I_beta,W_r,TM_cur,
				Induction_Motor.psi_a_pre,Induction_Motor.psi_b_pre, tv.tv_sec, tv.tv_usec);

	}

	if (fd) 
	{
		fclose(fd);
	}
	return -1;
}
