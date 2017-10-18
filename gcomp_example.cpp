/* Copyright 2003-2014 The MathWorks, Inc. */

// *******************************************************************
// **** To build this mex function use: mex sfun_cppcount_cpp.cpp ****
// *******************************************************************


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  gcomp_example

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "simstruc.h"
#include <XBotInterface/RobotInterface.h>


#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

// Function: mdlInitializeSizes ===============================================
// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, outputs, states, etc.).
static void mdlInitializeSizes(SimStruct *S)
{


        
    int model_size = 45;
    
    
    // No expected parameters
    ssSetNumSFcnParams(S, 0);

    // Parameter mismatch will be reported by Simulink
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    // Specify I/O
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, model_size);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, model_size);

    ssSetNumSampleTimes(S, 1);

    // Reserve place for C++ object
    ssSetNumPWork(S, 1);

    ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME);
    
    

}


// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}





// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you
//   have states that should be initialized once, this is the place
//   to do it.
#define MDL_START
static void mdlStart(SimStruct *S)
{
    // Store new C++ object in the pointers vector
    static auto robot = XBot::ModelInterface::getModel("/home/arturo/Code/advr-superbuild/configs/ADVR_shared/centauro/configs/config_centauro_full_body_wheels.yaml");
    ssGetPWork(S)[0] = (void *) robot.get();
    
}



#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
/* Function: mdlSetInputPortDimensionInfo ====================================
 * Abstract:
 *    This routine is called with the candidate dimensions for an input port
 *    with unknown dimensions. If the proposed dimensions are acceptable, the
 *    routine should go ahead and set the actual port dimensions.
 *    If they are unacceptable an error should be generated via
 *    ssSetErrorStatus.
 *    Note that any other input or output ports whose dimensions are
 *    implicitly defined by virtue of knowing the dimensions of the given port
 *    can also have their dimensions set.
 */
static void mdlSetInputPortDimensionInfo(SimStruct        *S,
                                        int_T            port,
                                        const DimsInfo_T *dimsInfo)
{
    
    // Retrieve C++ object from the pointers vector
    XBot::ModelInterface * robot = static_cast<XBot::ModelInterface *>(ssGetPWork(S)[0]);
    

    int_T  uNumDims   = dimsInfo->numDims;
    int_T  uWidth     = dimsInfo->width;
    int32_T  *uDims   = dimsInfo->dims;
    
    boolean_T  isOk = true;
    
    /* Set input port dimension */
    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
    
    /*
     * The block only accepts 2-D or higher signals. Check number of dimensions.
     * If the parameter and the input signal are non-scalar, their dimensions
     * must be the same.
     */
    isOk = (uNumDims == 1) && (uWidth == robot->getJointNum());

    
    if(!isOk){
        ssSetErrorStatus(S, "Invalid input port dimensions. The input signal must be"
        "equal to the model dof count!!");
        return;
    }

} /* end mdlSetInputPortDimensionInfo */

# define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
/* Function: mdlSetOutputPortDimensionInfo ===================================
 * Abstract:
 *    This routine is called with the candidate dimensions for an output port
 *    with unknown dimensions. If the proposed dimensions are acceptable, the
 *    routine should go ahead and set the actual port dimensions.
 *    If they are unacceptable an error should be generated via
 *    ssSetErrorStatus.
 *    Note that any other input or output ports whose dimensions are
 *    implicitly defined by virtue of knowing the dimensions of the given
 *    port can also have their dimensions set.
 */
static void mdlSetOutputPortDimensionInfo(SimStruct        *S,
int_T            port,
const DimsInfo_T *dimsInfo)
{
    // Retrieve C++ object from the pointers vector
    XBot::ModelInterface * robot = static_cast<XBot::ModelInterface *>(ssGetPWork(S)[0]);
    

    int_T  uNumDims   = dimsInfo->numDims;
    int_T  uWidth     = dimsInfo->width;
    int32_T  *uDims   = dimsInfo->dims;
    
    boolean_T  isOk = true;
    
    /* Set input port dimension */
    if(!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
    
    /*
     * The block only accepts 2-D or higher signals. Check number of dimensions.
     * If the parameter and the input signal are non-scalar, their dimensions
     * must be the same.
     */
    isOk = (uNumDims == 1) && (uWidth == robot->getJointNum());

    
    if(!isOk){
        ssSetErrorStatus(S, "Invalid input port dimensions. The input signal must be"
        "equal to the model dof count!!");
        return;
    }
} /* end mdlSetOutputPortDimensionInfo */

# define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
/* Function: mdlSetDefaultPortDimensionInfo ====================================
 *    This routine is called when Simulink is not able to find dimension
 *    candidates for ports with unknown dimensions. This function must set the
 *    dimensions of all ports with unknown dimensions.
 */
static void mdlSetDefaultPortDimensionInfo(SimStruct *S)
{
    int_T outWidth = ssGetOutputPortWidth(S, 0);
    /* Input port dimension must be unknown. Set it to scalar. */
    if(!ssSetInputPortMatrixDimensions(S, 0, 1, 1)) return;
    if(outWidth == DYNAMICALLY_SIZED){
        /* Output dimensions are unknown. Set it to scalar. */
        if(!ssSetOutputPortMatrixDimensions(S, 0, 1, 1)) return;
    }
} /* end mdlSetDefaultPortDimensionInfo */
#endif

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Retrieve C++ object from the pointers vector
    XBot::ModelInterface * robot = static_cast<XBot::ModelInterface *>(ssGetPWork(S)[0]);
    
    // Get data addresses of I/O
    InputRealPtrsType  u = ssGetInputPortRealSignalPtrs(S,0);
               real_T *y = ssGetOutputPortRealSignal(S, 0);
               
    int_T u_size =  ssGetInputPortWidth(S, 0);
    
    Eigen::Map<Eigen::VectorXd> q((double *)u, robot->getJointNum());
    
    std::cout << q.transpose() << std::endl;
    
    robot->setJointPosition(q);
    robot->update();
    
    Eigen::Map<Eigen::VectorXd> y_map((double *)y, robot->getJointNum());
    Eigen::VectorXd gcomp;
    robot->computeGravityCompensation(gcomp);
    
    y_map = gcomp;
    
    

    // Call AddTo method and return peak value
//     y[0] = q(0);
}

/* Define to indicate that this S-Function has the mdlG[S]etSimState methods */
#define MDL_SIM_STATE

/* Function: mdlGetSimState =====================================================
 * Abstract:
 *
 */
static mxArray* mdlGetSimState(SimStruct* S)
{
    // Retrieve C++ object from the pointers vector
    return mxCreateDoubleScalar(.1);
}
/* Function: mdlGetSimState =====================================================
 * Abstract:
 *
 */
static void mdlSetSimState(SimStruct* S, const mxArray* ma)
{
    // Retrieve C++ object from the pointers vector
//     DoubleAdder *da = /*static_cast<DoubleAdder*>(ssGetPWork(S)[0]);
//     da->SetPeak(mxGetP*/r(ma)[0]);
}

// Function: mdlTerminate =====================================================
// Abstract:
//   In this function, you should perform any actions that are necessary
//   at the termination of a simulation.  For example, if memory was
//   allocated in mdlStart, this is the place to free it.
static void mdlTerminate(SimStruct *S)
{
    // Retrieve and destroy C++ object
//     HelloWorld *da = static_cast<HelloWorld *>(ssGetPWork(S)[0]);
//     delete da;
}


// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
