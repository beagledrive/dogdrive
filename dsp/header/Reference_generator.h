/*
 * Beaglebone Open-Source Machine Drive
 * Reference generator header
 * KTH Project Work - 2018 
 */

#ifndef REF_GEN_H_		// Header wrapper name
#define REF_GEN_H_


/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */

/*
 * Reference Generator Struct
 */
typedef struct {

    // Base value of voltage and angular frequency of the output voltage of the inverter
    
    float Vbase;              // Base voltage
    float K_reservoir;        // Reservoir constant
    float Wbase;              // Base angular frequency of the output voltage of the inverter
    float W1_pre;             // Previous value of w1

    // Machine parameters and sampling time period

    float R_R;                // Rotor resistance in dynamic T-equivalent circuit of the induction motor
    float Lsigma;             // Leakage inductance in dynamic T-equivalent circuit of the induction motor
    float L_M;                // Magnetizing inductance in dynamic T_equivalent circuit of the induction motor
    float Ts;                 // Sampling time

    // Speed controller parameters

    float Kps_c;              // Constant part of proportional gain for the speed controller
    float Kis_c;	      // Constant part of integral gain for the speed controller
    float ba_c;               // Constant part of active viscous damping
    float int_fw_pre;	      // Previous value of the Integral output of the field weakening controller
    float psi_pre;            // Previous flux
    float int_s_pre;	      // Previous value of the Integral output of the speed controller
    float theta1_pre;         // Previous value of rotor position

    // Speed controller saturation limits

    float Imax;               // Maximum current
    float Imin;               // Minimum current
    float Inom;               // Nominal current

} RG_Typedef;


/* ================================== FUNCTION PROTOTYPES =================== */

/*
 * Initialize Reference Generator Struct
 */
void RG_StructInit(RG_Typedef *RG_Struct,float BaseVolt,float ResCons,float BaseAngularspeed,
			float RotorRes,float LeakageInd,float MagnetizingInd,float Tsamp,
			float InertiaConst,float PolePair, float SVScalingConst,
			float DampingConst,float SpdCtrlBwidth, float MaxCur,float MinCur, float NomCur);

/*
 * Run Reference Generator Control
 */
void RG_Controller(RG_Typedef *RG_Struct,float Vd_ref,float Vq_ref,float Wr_ref,float Wr,
			float *Id_ref,float *Iq_ref,float *W1,float *theta1);

#endif
