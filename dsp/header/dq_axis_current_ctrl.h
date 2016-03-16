/*
 * Beaglebone Open-Source Machine Drive
 * DQ axis current controller header
 * KTH Project Work - 2018 
 */

#ifndef DQ_AXIS_CUR_CTRL_H_		// Header wrapper name
#define DQ_AXIS_CUR_CTRL_H_


/* ================================== MACROS ================================ */


/* ================================== TYPEDEFS ============================== */

/*
 * PI controller for D-axis and Q-axis control
 */
typedef struct
{
    float Kp;                 // Proportional Gain
    float Ki;		      // Integral Gain
    float Int_pre;	      // Previous value of the Integral output
    float Ts;                 // Sampling time
    float Ra;                 // Active Resistance
    float Lsigma;             // Leakage Inductance
    float V_ulim;             // Upper limit of the axis voltage
    float V_llim;             // Lower limit of the axis voltage

} PI_Typedef;


/* ================================== FUNCTION PROTOTYPES =================== */

/*
 * Initialize PI controller struct
 */
void PI_StructInit(PI_Typedef *PI_Struct,float RStator,float CurrCtrlBwidth,float Tsamp,
			float L_leak, float V_max, float V_min);

/*
 * PI controller function
 * Parameters
 * * Is_d_axis - binary input, if 1, implement d-axis control, otherwise q-axis control
 * * Iprim - primary current matches axis control variant, e.g. if D-axis ctrl, Iprim = Id and vice versa
 * * Isec - secondary current is complementer to primary current
 */
void PI_Controller(PI_Typedef *PI_Struct,float I_ref,float Iprim,float Isec,
			float w1,float *V_ref, int Is_d_axis);

#endif
