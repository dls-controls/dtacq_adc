/*
 * BPM_calc.cpp
 *
 * Asyn driver for callbacks to standard asyn array interfaces for NDArray drivers.
 * This is commonly used for EPICS waveform records.
 *
 * Author: Mark Rivers
 *
 * Created April 25, 2008
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsMessageQueue.h>
#include <cantProceed.h>
#include <iocsh.h>
#include "NDArray.h"
#include <epicsExport.h>
#include <epicsTypes.h>
#include <asynStandardInterfaces.h>
#include "NDPluginDriver.h"
#define BPM_calcDataString "STD_ARRAY_DATA"

/* Converts NDArray callback data into standard asyn arrays
   (asynInt8Array, asynInt16Array, asynInt32Array, asynFloat32Array or
   asynFloat64Array); normally used for putting NDArray data in EPICS
   waveform records. It handles the data type conversion if the NDArray
   data type differs from the data type of the asyn interface. It
   flattens the NDArrays to a single dimension because asyn and EPICS
   do not support multi-dimensional arrays. */
class BPM_calc : public NDPluginDriver {
public:
    BPM_calc(const char *portName, int queueSize, int blockingCallbacks,
             const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
             size_t maxMemory, int priority, int stackSize);
    /* These methods override the virtual methods in the base class */
    void processCallbacks(NDArray *pArray);
protected:
    int BPM_calcData;
    #define FIRST_BPM_CALC_PARAM BPM_calcData
    #define LAST_BPM_CALC_PARAM BPM_calcData
};
#define NUM_BPM_CALC_PARAMS ((int)(&LAST_BPM_CALC_PARAM - &FIRST_BPM_CALC_PARAM + 1))
static const char *driverName="BPM_calc";

/* Callback function that is called by the NDArray driver with new NDArray data.
   It does callbacks with the array data to any registered asyn clients on any
   of the asynXXXArray interfaces.  It converts the array data to the type
   required for that interface.
   \param[in] pArray  The NDArray from the callback.
*/

/** This subroutine takes 4 diode inputs waveform scans of NELM_IN elements,
appends them to the relevant output to provide a rolling scope-like view,
calculates X and Y positions and intensity as waveforms and rolling averages.
<pre>
           A _
           |   B
--BEAM-->  | + |
           C _ |
               D
</pre>
Inputs:
* A = Top Left Input
* B = Top Right Input
* C = Bottom Left Input
* D = Bottom Right Input

Outputs:
* VALE = X Position Waveform ((A + B) - (C + D))
* VALF = Y Position Waveform ((A + C) - (B + D))
* VALG = Intensity Waveform (A + B + C + D)
**/

void BPM_calc::processCallbacks(NDArray *pArray)
{
    /* This function calls back any registered clients on the standard
       asyn array interfaces with the data in our private buffer.
       It is called with the mutex already locked.
    */
    unsigned int i;
    size_t dims[2];
    NDArrayInfo_t arrayInfo;
    asynStandardInterfaces *pInterfaces = &this->asynStdInterfaces;
    NDArray *pProcessed, *pOutput;
    short *pAData, *pBData, *pCData, *pDData;
    short *pEData, *pFData, *pGData;
    /* Call the base class method */
    NDPluginDriver::processCallbacks(pArray);
    pArray->getInfo(&arrayInfo);
 	/* Allocate 3 x n_elements NDArray */
    dims[0] = arrayInfo.xSize;
    dims[1] =  3;
    printf("x size: %d\ny size: %d\n", arrayInfo.xSize, arrayInfo.ySize);
    pProcessed = this->pNDArrayPool->alloc(2, dims, (NDDataType_t)NDInt16,
                                           0, NULL);
    pAData = (short *)pArray->pData;
    pBData = (short *)pArray->pData + arrayInfo.xSize;
    pCData = (short *)pArray->pData + 2*arrayInfo.xSize;
    pDData = (short *)pArray->pData + 3*arrayInfo.xSize;
    pEData = (short *)pProcessed->pData;
    pFData = (short *)pProcessed->pData + arrayInfo.xSize;
    pGData = (short *)pProcessed->pData + 2*arrayInfo.xSize;
    /* This function is called with the lock taken, and it must be set
       when we exit. The following code can be exected without the mutex
       because we are not accessing pPvt */
    this->unlock();
    /* Calculate output and put in new NDArray */
    for (i=0; i<arrayInfo.xSize; i++) {
        *pEData++ = (*pAData + *pBData) - (*pCData + *pDData);
        *pFData++ = (*pAData + *pCData) - (*pBData + *pDData);
        *pGData++ = *pAData++ + *pBData++ + *pCData++ + *pDData++;
    }
    setIntegerParam(NDArraySizeY, arrayInfo.xSize);
    setIntegerParam(NDArraySizeX, 3);
    this->lock();
    this->getAttributes(pProcessed->pAttributeList);
    this->unlock();
    this->doCallbacksGenericPointer(pProcessed, NDArrayData, 0);
    this->lock();
    pProcessed->release();
    this->callParamCallbacks();
}

/* Constructor for BPM_calc; all parameters are simply passed to
   NDPluginDriver::NDPluginDriver. This plugin cannot block
   (ASYN_CANBLOCK=0) and is not multi-device (ASYN_MULTIDEVICE=0).
   It allocates a maximum of 2 NDArray buffers for internal use.
   \param[in] portName The name of the asyn port driver to be created.
   \param[in] queueSize The number of NDArrays that the input queue for
                        this plugin can hold when
                        NDPluginDriverBlockingCallbacks=0.  Larger queues
                        can decrease the number of dropped arrays,
                        at the expense of more NDArray buffers being
                        allocated from the underlying driver's NDArrayPool.
   \param[in] blockingCallbacks Initial setting for the
                                NDPluginDriverBlockingCallbacks flag.
                                0=callbacks are queued and executed by the
                                callback thread; 1 callbacks execute in
                                the thread of the driver doing the callbacks.
   \param[in] NDArrayPort Name of asyn port driver for initial source
                          of NDArray callbacks.
   \param[in] NDArrayAddr asyn port driver address for initial
                          source of NDArray callbacks.
   \param[in] maxMemory The maximum amount of memory that the NDArrayPool
                        for this driver is allowed to allocate. Set this
                        to -1 to allow an unlimited amount of memory.
   \param[in] priority The thread priority for the asyn port driver
                       thread if ASYN_CANBLOCK is set in asynFlags.
   \param[in] stackSize The stack size for the asyn port driver
                        thread if ASYN_CANBLOCK is set in asynFlags.
*/

BPM_calc::BPM_calc(const char *portName, int queueSize, int blockingCallbacks,
                   const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
                   size_t maxMemory, int priority, int stackSize)
    /* Invoke the base class constructor */
    : NDPluginDriver(portName, queueSize, blockingCallbacks, NDArrayPort,
                     NDArrayAddr, 1, NUM_BPM_CALC_PARAMS, maxBuffers, maxMemory,
                     asynGenericPointerMask,
                     asynGenericPointerMask,
                     /* asynFlags is set to 0, because this plugin cannot
                        block and is not multi-device. It does autoconnect */
                     0, 1, priority, stackSize)
{
    createParam(BPM_calcDataString, asynParamGenericPointer, &BPM_calcData);
    /* Set the plugin type string */
    setStringParam(NDPluginDriverPluginType, "BPM_calc");
    /* Try to connect to the NDArray port */
    connectToArrayPort();
}

/** Configuration command */
extern "C" int BPM_calcConfigure(const char *portName, int queueSize,
                                 int blockingCallbacks, const char *NDArrayPort,
                                 int NDArrayAddr, int maxBuffers,
                                 size_t maxMemory,int priority, int stackSize)
{
    new BPM_calc(portName, queueSize, blockingCallbacks, NDArrayPort,
                 NDArrayAddr, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}


/* EPICS iocsh shell commands */
static const iocshArg initArg0 = {"portName", iocshArgString};
static const iocshArg initArg1 = {"frame queue size", iocshArgInt};
static const iocshArg initArg2 = {"blocking callbacks", iocshArgInt};
static const iocshArg initArg3 = {"NDArrayPort", iocshArgString};
static const iocshArg initArg4 = {"NDArrayAddr", iocshArgInt};
static const iocshArg initArg5 = {"maxBuffers", iocshArgInt};
static const iocshArg initArg6 = {"maxMemory", iocshArgInt};
static const iocshArg initArg7 = {"priority", iocshArgInt};
static const iocshArg initArg8 = {"stackSize", iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0, &initArg1, &initArg2,
                                            &initArg3, &initArg4, &initArg5,
                                            &initArg6, &initArg7, &initArg8};
static const iocshFuncDef initFuncDef = {"BPM_calcConfigure", 9, initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    BPM_calcConfigure(args[0].sval, args[1].ival, args[2].ival,
                      args[3].sval, args[4].ival, args[5].ival,
                      args[6].ival, args[7].ival, args[8].ival);
}

extern "C" void BPM_calc_register(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

extern "C" {
    epicsExportRegistrar(BPM_calc_register);
}
