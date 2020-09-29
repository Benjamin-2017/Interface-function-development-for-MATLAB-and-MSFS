/*
 * File : MSFStoSIMULINK.c
 * Abstract:
 *       An example C-file S-function for multiplying an input by 2,
 *       y  = 2*u
 *
 * Real-Time Workshop note:
 *   This file can be used as is (noninlined) with the Real-Time Workshop
 *   C rapid prototyping targets, or it can be inlined using the Target 
 *   Language Compiler technology and used with any target. See 
 *     matlabroot/toolbox/simulink/blocks/tlc_c/timestwo.tlc   
 *   the TLC code to inline the S-function.
 *
 * See simulink/src/sfuntmpl_doc.c
 *
 * Copyright 1990-2009 The MathWorks, Inc.
 * $Revision: 1.1.6.2 $
 */


#define S_FUNCTION_NAME  MSFStoSIMULINK
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"


#include <windows.h>
#include "FSUIPC_User.h"
#include "FSinterface.h"

#include "IPCuser.h"
//#include "IPCuser.c"

// S-function parameters
#define NUM_PARAMS (1)
#define TS_PARAM (ssGetSFcnParam(S,0))

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





/*================*
 * Build checking *
 *================*/


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

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, DYNAMICALLY_SIZED);

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
// // 	FSUIPC_Close();
// 	if(FSUIPC_Open(SIM_ANY, &dwResult)) {
// 		
// 		// Enable slew mode
// 		FSUIPC_Write(SLEWFLAG_OFFSET, 2, &SlewFlag, &Result);
// 
// 		// Choose slew mode display
// 		FSUIPC_Write(SLEWMODE_OFFSET, 2, &SlewMode, &Result);
// 
// 		// Send to Flight Simulator
// 		FSUIPC_Process(&Result);
// 	}
// 	else {
// 		ssSetErrorStatus(S,pszErrors[dwResult]);
// 		return;
// 	}
// 	  
  }
#endif /*  MDL_START */





/* Function: mdlOutputs =======================================================
 * Abstract:
 *    y = 2*u
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    int_T             i;
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    int_T             width = ssGetOutputPortWidth(S,0);
	
	DWORD dwResult;
    unsigned short SlewFlag = 1;
	unsigned short SlewMode = 3;
	unsigned int Result = 0;
    char chTime[3];
    short VS;
    short AP_VS;
	
   //FSUIPC_Close();  
    FSUIPC_Open(SIM_ANY, &dwResult);
    FSUIPC_Read(0x238, 3, chTime, &dwResult);
    FSUIPC_Read(AP_VS_OFFSET, 2, &AP_VS, &dwResult);
    
    FSUIPC_Process(&dwResult); 
    
   
    
    
   //*y =chTime[2];  
   *y =AP_VS;  


   
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


