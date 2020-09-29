/*
 * File : SIMULINKtoMSFS.c
 * S-Function interface for Microsoft Flight Simulator
 * Real-Time Workshop note:
 *   This file can be used as is (noninlined) with the Real-Time Workshop
 *   C rapid prototyping targets, or it can be inlined using the Target 
 *   Language Compiler technology and used with any target. 
 *   the TLC code to inline the S-function.
 *
 * See simulink/src/sfuntmpl_doc.c
 */


#define S_FUNCTION_NAME  SIMULINKtoMSFS
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include "universal.h"
#include <windows.h>
#include "FSUIPC_User.h"
//#include "FSinterface.h"

#include "IPCuser.h"
#include "Types.h"
//#include "IPCuser.c"
//Include standard headers
#include <math.h>

// S-function parameters
#define NUM_PARAMS (1)
#define TS_PARAM (ssGetSFcnParam(S,0))

// Macros to access the S-function parameter values
#define SAMPLE_TIME (mxGetPr(TS_PARAM)[0])

char *pszErrors[] =
	{	"Okay",
		"Attempt to Open when already Open",
		"Cannot link to FSUIPC or WideClient",
		"Failed to Register common message with Windows",
		"Failed to create Atom for mapping filename",
		"Failed to create a file mapping object",    // 5
		"Failed to open a view to the file map",
		"Incorrect version of FSUIPC, or not FSUIPC",
		"Sim is not version requested",
		"Call cannot execute, link not Open",     // 9
		"Call cannot execute: no requests accumulated",
		"IPC timed out all retries",
		"IPC sendmessage failed all retries",
		"IPC request contains bad data",
		"Maybe running on WideClient, but FS not running on Server, or wrong FSUIPC",
		"Read or Write request cannot be added, memory for Process is full",
	};


// FS offsets
#define TAS_OFFSET			0x02B8
#define IAS_OFFSET			0x02BC
#define VS_OFFSET			0x0842
#define LAT_OFFSET			0x0560
#define LON_OFFSET			0x0568
#define ALT_OFFSET			0x0570
#define PITCH_OFFSET		0x0578
#define BANK_OFFSET			0x057C
#define HEAD_OFFSET			0x0580
#define SLEWFLAG_OFFSET		0x05DC
#define ROLLRATE_OFFSET		0x05E4
#define YAWRATE_OFFSET		0x05E6
#define SLEWMODE_OFFSET		0x05F4


// FS data structure types
typedef struct 
{
	SInt32 Lo;
	SInt32 Hi;
} SFSVarType;

typedef struct 
{
	UInt32 Lo;
	SInt32 Hi;
} UFSVarType;

typedef struct 
{
	UFSVarType Lat;
	UFSVarType Lon;
	UFSVarType Alt;
} FSPositionType;

typedef struct
{
	SInt32 Pitch;
	SInt32 Bank;
	SInt32 Heading;
} FSAttitudeType;



/*================*
 * Build checking *
 *================*/
// Convert position from FS to simulation application
void PositionFSToSim(double *Pos, FSPositionType *FSPos)
{
	double LatLo, LatHi, LonLo, LonHi, AltLo, AltHi;

	LatLo = (double)FSPos->Lat.Lo*90.0/10001750.0/(65536.0*65536.0);
	LatHi = (double)FSPos->Lat.Hi*90.0/10001750.0;
	Pos[LAT] = LatHi + LatLo;

	LonLo = (double)FSPos->Lon.Lo*360.0/(65536.0*65536.0)/(65536.0*65536.0);
	LonHi = (double)FSPos->Lon.Hi*360.0/(65536.0*65536.0);
	Pos[LON] = LonHi + LonLo;

	AltLo = (double)FSPos->Alt.Lo/10000000000.0;
	AltHi = (double)FSPos->Alt.Hi;
	Pos[ALT] = AltHi + AltLo;
} // PositionFSToSim

// Convert position from simulation application to FS
void PositionSimToFS(FSPositionType *FSPos, double *Pos)
{
	double LatScaled, LonScaled;
	SInt64 Temp;

	LatScaled = Pos[LAT]*10001750.0/90.0;
	Temp = (SInt64)(LatScaled*65536*65536);
	FSPos->Lat.Hi = (SInt32)(Temp>>32);
	FSPos->Lat.Lo = (UInt32)(Temp);
	
	LonScaled = Pos[LON]*65536*65536/360.0;
	Temp = (SInt64)(LonScaled*65536*65536);	
	FSPos->Lon.Hi = (SInt32)(Temp>>32);
	FSPos->Lon.Lo = (UInt32)(Temp);


	FSPos->Alt.Hi = (SInt32)floor(Pos[ALT]);
	FSPos->Alt.Lo = (SInt32)((Pos[ALT]-(double)FSPos->Alt.Hi)*10000000000.0);
} // PositionSimToFS

// Convert attitude from FS to simulation application
void AttitudeFSToSim(double *Att, FSAttitudeType *FSAtt)
{
	Att[PITCH] = -(double)FSAtt->Pitch*360.0/(65536.0*65536.0);
	Att[ROLL] = -(double)FSAtt->Bank*360.0/(65536.0*65536.0);
	Att[YAW] = (double)FSAtt->Heading*360.0/(65536.0*65536.0);
} // AttitudeFSToSim

// Convert attitude from simulation application to FS
void AttitudeSimToFS(FSAttitudeType* FSAtt, double *Att)
{
	FSAtt->Pitch = (SInt32)(-Att[PITCH]*65536.0*65536.0/360.0);
	FSAtt->Bank = (SInt32)(-Att[ROLL]*65536.0*65536.0/360.0);
	FSAtt->Heading = (SInt32)(Att[YAW]*65536.0*65536.0/360.0);
} // AttitudeSimToFS

// Convert airspeed from FS to simulation application
void AirspeedFSToSim(double *Airsp, SInt32 FSAirsp)
{
	*Airsp = (double)FSAirsp*0.51479/128.0;
} // AirspeedFSToSim

// Convert airspeed from simulation application to FS
void AirspeedSimToFS(SInt32 *FSAirsp, double Airsp)
{
	*FSAirsp = (SInt32)(Airsp/0.51479*128.0);
} // AirspeedSimToFS

// Convert vertical speed from FS to simulation application
void VertSpeedFSToSim(double *VS, SInt16 FSVS)
{
	*VS = -(double)FSVS/60.0;
} // VertSpeedFSToSim

// Convert vertical speed from simulation application to FS
void VertSpeedSimToFS(SInt16 *FSVS, double VS)
{
	*FSVS = -(SInt16)(VS*60.0);
} // VertSpeedSimToFS


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 3)) return;

	// Position input
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 0, 1);

	// Attitude input
    ssSetInputPortWidth(S, 1, 3);
    ssSetInputPortRequiredContiguous(S, 1, true); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 1, 1);

	// Velocity input
	ssSetInputPortWidth(S, 2, 2);
    ssSetInputPortRequiredContiguous(S, 2, true); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}


#undef MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */


#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
	DWORD dwResult;
	unsigned short SlewFlag = 1;
	unsigned short SlewMode = 3;
	unsigned int Result = 0;
// 	// Initialize the FSUIPC
   	FSUIPC_Close();
	if(FSUIPC_Open(SIM_ANY, &dwResult)) {
		
		// Enable slew mode
		FSUIPC_Write(SLEWFLAG_OFFSET, 2, &SlewFlag, &Result);
		// Choose slew mode display
		FSUIPC_Write(SLEWMODE_OFFSET, 2, &SlewMode, &Result);
		// Send to Flight Simulator
		FSUIPC_Process(&Result);
	}
	else {
		ssSetErrorStatus(S,pszErrors[dwResult]);
		return;
	}

  }
#endif /*  MDL_START */





/* Function: mdlOutputs =======================================================
 * Abstract:
 *    y = 2*u
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    const real_T *posvec = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *attvec = (const real_T*) ssGetInputPortSignal(S,1);
	const real_T *velvec = (const real_T*) ssGetInputPortSignal(S,2);
    
	double Position[NPOS];
	double Attitude[NROT];
	double Airspeed;
	double VertSpeed;

	FSPositionType FSPos;
	FSAttitudeType FSAtt;
	int FSAirsp;
	short FSVS;

	unsigned int Result = 0;

	Position[LAT] = posvec[0];
	Position[LON] = posvec[1];
	Position[ALT] = posvec[2];
	
	Attitude[ROLL] = attvec[0];
	Attitude[PITCH] = attvec[1];
	Attitude[YAW] = attvec[2];

	Airspeed = velvec[0];
	VertSpeed = velvec[1];

	// Unit conversions
	PositionSimToFS(&FSPos, Position);
	AttitudeSimToFS(&FSAtt, Attitude);
	AirspeedSimToFS(&FSAirsp, Airspeed);
	VertSpeedSimToFS(&FSVS, VertSpeed);

	// Write to FS data structure
	FSUIPC_Write(LAT_OFFSET, sizeof(FSPos.Lat), &FSPos.Lat, &Result);
	FSUIPC_Write(LON_OFFSET, sizeof(FSPos.Lon), &FSPos.Lon, &Result);
	FSUIPC_Write(ALT_OFFSET, sizeof(FSPos.Alt), &FSPos.Alt, &Result);
	FSUIPC_Write(PITCH_OFFSET, sizeof(FSAtt.Pitch), &FSAtt.Pitch, &Result);
	FSUIPC_Write(BANK_OFFSET, sizeof(FSAtt.Bank), &FSAtt.Bank, &Result);
	FSUIPC_Write(HEAD_OFFSET, sizeof(FSAtt.Heading), &FSAtt.Heading, &Result);
	FSUIPC_Write(TAS_OFFSET, sizeof(FSAirsp), &FSAirsp, &Result);
	FSUIPC_Write(IAS_OFFSET, sizeof(FSAirsp), &FSAirsp, &Result);
	FSUIPC_Write(VS_OFFSET, sizeof(FSVS), &FSVS, &Result);

	// Send to Flight Simulator
	FSUIPC_Process(&Result);

}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
   FSUIPC_Close();   
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


