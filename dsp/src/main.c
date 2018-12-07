/*
 * Beaglebone Open-Source Machine Drive
 * Main Function
 * KTH Project Work - 2018 
 */


/* ================================== INCLUDES ============================== */

#include <stdlib.h>
#include <sys/time.h>
#include <sys/stat.h>

#include <IM_model.h>
#include <Reference_generator.h>
#include <dq_transformation.h>
#include <dq_axis_current_ctrl.h>
#include <base_compute.h>
#include <bw_bridge.h>
#include <svpwm.h>


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
#define TIME_OFFSET_FACTOR		(float)0.5

// fix this to be sensor input
#define VDC				(float)760


/*
 * Torque Load in pu
 */
#define TORQUE_LOAD			(float)0.6	// [p.u.]

/*
 * Define D-Q Transformation flags
 */
#define D_AXIS_CONTROL			(int)1
#define Q_AXIS_CONTROL			(int)0

/*
 * Needed for usleep() function
 */
#define _XOPEN_SOURCE 			500

/*
 * Memory map GPIO-based interrupt signal from FPGA to internal memory of DSP
 */
#define GPIO_START_ADDR			0x4804C000
#define GPIO_END_ADDR			0x4804DFFF
#define GPIO_SIZE 			(GPIO_END_ADDR - GPIO_START_ADDR)
#define GPIO_DATAIN			0x138
#define GPIO_PIN 			(1 << 17)	// PIN 17 on BB for interrupt

/*
 * Register location of commands to FPGA
 */
#define CONFIG_REG_L			(int)0
#define CONFIG_REG_U			(int)1
#define SW_ON_REG_L			(int)2
#define SW_ON_REG_U			(int)3
#define SW_OFF_REG_L			(int)4
#define SW_OFF_REG_U			(int)5
// Size of command register [uint16_t]
#define COMMAND_REG_SIZE		(int)6


/* ================================== TYPEDEFS ============================== */


/* ================================== FUNCTION PROTOTYPES =================== */


/*
 * Function to send PWM regulation signals from DSP to shared GPMC memory
 */
int dsp_data_to_gpmc(const uint32_t P1_sw_on, const uint32_t P1_sw_off,
			const uint32_t P2_sw_on, const uint32_t P2_sw_off,
			const uint32_t P3_sw_on, const uint32_t P3_sw_off);

/*
 * Run control sequence and space vector modulation on interrupt cycle
 */
int control_loop();


/* ================================== INTERNAL GLOBALS ====================== */


/*
 * Normalized P.U. values of motor parameters
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
/*********************************/

/*
 * Alpha/Beta voltage and current parameters
 * Current load torque
 * Phase currents
 * Phase Voltages
 * Motor Speed
 * Reference D-Q voltages and motor speed
 * Reference D-Q currents
 * Rotor Position and angular speed
 * D-Q Currents
 */
static float V_alpha = 0;
static float V_beta = 0;

#ifdef _DEBUG

static float I_alpha = 0;
static float I_beta = 0;
static float TM_cur = 0;

#endif

static float I_a = 0;
static float I_b = 0;
static float I_c = 0;

static float V_a = 0;
static float V_b = 0;
static float V_c = 0;

static float W_r = 0;

static float Vd_ref = 0;
static float Vq_ref = 0;
static float Wr_ref = 0;

static float Id_ref = 0;
static float Iq_ref = 0;

static float W1 = 0;
static float Theta1 = 0;

static float Id = 0;
static float Iq = 0;
/*********************************/

/*
 * Induction Motor model
 */
static IM_Typedef Induction_Motor;

/*
 * Reference Generator
 */
static RG_Typedef Reference_Generator;

/*
 * PI Controller
 */
static PI_Typedef PI_Control;

/*
 * SVPWM Struct
 */
static SVPWM_Typedef SV_PWM;


#ifdef _DEBUG

/*
 * File descriptor for debug CSV output
 */
static FILE* fd_csv;

#endif

/*
 * PWM signal rise and fall times for the 3 phases
 * measured in ns
 */
static uint32_t P1_rise;
static uint32_t P1_fall;
static uint32_t P2_rise;
static uint32_t P2_fall;
static uint32_t P3_rise;
static uint32_t P3_fall;
/*********************************/


/* ================================== FUNCTION DEFINITIONS ================== */


int control_loop()
{
	int b_error = 0;		// Error bit for possible future error handling

#ifdef _DEBUG

	// Get TimeStamp for debug output
	struct timeval tv;

	// Simulate Induction Motor
	IM_model(&Induction_Motor, V_alpha, V_beta, TORQUE_LOAD,
			&I_a, &I_b, &I_c, &I_alpha, &I_beta, &TM_cur, &W_r);

#endif

	// Run Reference Generator
	RG_Controller(&Reference_Generator, Vd_ref, Vq_ref, Wr_ref,
			W_r, &Id_ref, &Iq_ref, &W1, &Theta1);

	// Run DQ_transformation algorithm
	// FIXME - no sensor interfacing yet
	DQ_Transformation(SV_SCALING_CONST, I_a, I_b, I_c, Theta1, &Id, &Iq);


	// Run D-axis control
	PI_Controller(&PI_Control, Id_ref, Id, Iq, W1, &Vd_ref, D_AXIS_CONTROL);


	// Run Q-axis control
	PI_Controller(&PI_Control, Iq_ref, Iq, Id, W1, &Vq_ref, Q_AXIS_CONTROL);


	// Run IDQ_transformation algorithm
	IDQ_Transformation(SV_SCALING_CONST, Vd_ref, Vq_ref, Theta1,
			&V_alpha, &V_beta, &V_a, &V_b, &V_c);

	// Run Space-Vector PWM modulation
	SVPWM_Algorithm(&SV_PWM, V_a, V_b, V_c, &P1_rise, &P1_fall, &P2_rise, &P2_fall,
				&P3_rise, &P3_fall);
	
	// Send Timing information to FPGA
	if (dsp_data_to_gpmc(P1_rise, P1_fall, P2_rise, P2_fall, P3_rise, P3_fall))
	{
		fprintf(stderr, "Error with sending PWM timing data to FPGA\n");
		b_error = 1;
	}
	
#ifdef _DEBUG
	// Log output to DEBUG CSV
	gettimeofday(&tv,NULL);
	fprintf(fd_csv, "%f,%f,%f,%f,%f,%f,%f,%f,%ld,%ld,%f,%u,%u\n",V_alpha,V_beta,I_alpha,I_beta,W_r,TM_cur,
			Induction_Motor.psi_a_pre,Induction_Motor.psi_b_pre, tv.tv_sec, tv.tv_usec,
			V_a, P1_rise, P1_fall);

#endif


	return b_error;
}


int dsp_data_to_gpmc(const uint32_t P1_sw_on, const uint32_t P1_sw_off,
			const uint32_t P2_sw_on, const uint32_t P2_sw_off,
			const uint32_t P3_sw_on, const uint32_t P3_sw_off)
{
	int b_error = 0;		// Error Bit
	int i = 0;			// Increment loop variable

	struct bridge br;		// Set up communication bridge

	/*
	 * Setup configuration command bits for FPGA
	 * From LSB:
	 * reset, enable, polarity,
	 */
	uint16_t setup_conf_L = 0b00000010;
	uint16_t setup_conf_U = 0x0000;	

	/*
	 * Store switch on and switch off times
	 */
	uint16_t sw_on_UW = (uint16_t)(P1_sw_on >> 16);			// Phase 1 switch on at [ns] Upper
	uint16_t sw_on_LW = (uint16_t)(P1_sw_on & 0x0000FFFF);		// Phase 1 switch on at [ns] Lower
	uint16_t sw_off_UW = (uint16_t)(P1_sw_off >> 16);		// Phase 1 switch off at [ns] Upper
	uint16_t sw_off_LW = (uint16_t)(P1_sw_off & 0x0000FFFF);	// Phase 1 switch off at [ns] Lower


	// Command to be sent to the FPGA memory
	uint16_t P1_data[] = {sw_on_LW, sw_on_UW, sw_off_LW, sw_off_UW, setup_conf_L, setup_conf_U};
	// Map commands to FPGA register addresses
	uint16_t reg_addr[] = {SW_ON_REG_L, SW_ON_REG_U, SW_OFF_REG_L, SW_OFF_REG_U, CONFIG_REG_L, CONFIG_REG_U};

	// Initialize communication bridge
	if (bridge_init(&br, BW_BRIDGE_MEM_ADR, BW_BRIDGE_MEM_SIZE) < 0) 
	{
		fprintf(stderr, "Unable to initialize communication bridge! \n");
		b_error = 1;
	}
	
	//fprintf(stdout, "-----%ui-----%ui\n", P1_sw_on, P1_sw_off);
	if (!b_error)
	{
		while (i < COMMAND_REG_SIZE)
		{
			set_fpga_mem(&br, reg_addr[i]*sizeof(uint16_t), &P1_data[i], 1);
			i++;
		}
	}

	// Close communication bridge
	if (!b_error)
	{
		bridge_close(&br);
	}

	return b_error;
}


int main()
{
	int b_error = 0;

#ifdef _DEBUG
	// File name for DEBUG output CSV
	const char *fileName = "dbg_output.csv";

#endif

	volatile void *gpio_addr = NULL;		// GPIO address pointer
	volatile unsigned int *gpio_datain = NULL;	// GPIO address to read data
	
	// Latch GPIO PIN to detect only once per interrupt in case of fast operation
	int b_pin_latch = 0;

	// File descriptor for device memory
	int fd_mem = open("/dev/mem", O_RDWR);
	if (fd_mem == -1)
	{
		fprintf(stderr, "Unable to open device memory module\n");
		b_error = 1;
	}

	// Map memory location
	if (!b_error)
	{
		gpio_addr = mmap(0, GPIO_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, GPIO_START_ADDR);
		
		if (gpio_addr == MAP_FAILED)
		{
			fprintf(stderr, "Unable to map GPIO to memory\n");
			b_error = 1;
		} else  {
			gpio_datain = gpio_addr + GPIO_DATAIN;
		}
	}

#ifdef _DEBUG
	// Open DEBUG file for output
	if (!b_error)
	{
		// Clear content of file first, then open again for writing
		fd_csv = fopen(fileName, "w");

		if(fd_csv == NULL)
		{
			fprintf(stderr, "Unable to open file! \n");
			b_error = 1;
		} else {
			fclose(fd_csv);
		}

		// Open file again for logging output
		fd_csv = fopen(fileName, "a");
		if(fd_csv == NULL)
		{
			fprintf(stderr, "Unable to open file! \n");
			b_error = 1;
		} else {
			fprintf(stdout, "Starting application in debug mode, logging output to %s\n",
					fileName);
		}
	}

#endif

	/*
	 * Compute normalized values of motor parameters
	 * Initialize induction motor model
	 * Initialize reference generator
	 * Initialize PI controller
	 * Initialize SV-PWM
	 */
	if (!b_error)
	{
		Base_Compute(NOMINAL_PHASE_VOLTAGE, NOMINAL_PHASE_CURRENT, POLE_PAIRS,
				NOMINAL_STATOR_FREQUENCY, SV_SCALING_CONST, NOMINAL_STATOR_RESISTANCE,
				NOMINAL_ROTOR_RESISTANCE, NOMINAL_LEAKAGE_INDUCTANCE, NOMINAL_MAGNETIZING_INDUCTANCE,
				RISE_TIME_CC, NOMINAL_INERTIA, NOMINAL_DAMPING_CONST,
				&Stator_Resistance, &Rotor_Resistance, &Leakage_Inductance, &Magnetizing_Inductance,
				&Mech_Inertia_Const, &Damping_Const, &Voltage_Base, &Voltage_Max,
				&Voltage_Min, &Angular_Freq_Base, &Max_Current, &Nom_Current, &Min_Current,
				&Curr_Ctrl_Bandwidth, &Spd_Ctrl_Bandwidth, &Base_Ang_Freq);

		IM_StructInit(&Induction_Motor, Stator_Resistance, Rotor_Resistance, Leakage_Inductance, 
				Magnetizing_Inductance,SAMPLING_TIME, POLE_PAIRS, SV_SCALING_CONST,
				Mech_Inertia_Const,Damping_Const);

		RG_StructInit(&Reference_Generator, Voltage_Base, RESERVOIR_CONST, Base_Ang_Freq,
				Rotor_Resistance, Leakage_Inductance, Magnetizing_Inductance,
				SAMPLING_TIME, Mech_Inertia_Const,POLE_PAIRS, SV_SCALING_CONST,
				Damping_Const,Spd_Ctrl_Bandwidth, Max_Current, Min_Current, Nom_Current);

		PI_StructInit(&PI_Control, Stator_Resistance, Curr_Ctrl_Bandwidth,SAMPLING_TIME,
				Leakage_Inductance, Voltage_Max, Voltage_Min);
		// FIXME - VDC not sensor input
		SVPWM_StructInit(&SV_PWM, SAMPLING_TIME, VDC, TIME_OFFSET_FACTOR);
	}

	// Wait for signal from the FPGA
	if (!b_error)
	{
		while(1)
		{
			if ((*gpio_datain & GPIO_PIN) && !b_pin_latch)
			{
				b_pin_latch = 1;
				if (control_loop())
				{
					fprintf(stderr, "Error in control loop, breaking sequence, shutting down!\n");
					b_error = 1;
					break;
				}
			} else {
				b_pin_latch = 0;
			}
		}
	} else {
		fprintf(stderr, "Error during initialization, shutting down \n");
	}

	// Close all files and unmap memory segment
	if (fd_mem)
	{	
		close(fd_mem);
	}

	if (gpio_addr)
	{
		munmap((void*)gpio_addr, GPIO_SIZE);
	}

#ifdef _DEBUG
	if (fd_csv) 
	{
		fclose(fd_csv);
	}
#endif

	return b_error;
}
