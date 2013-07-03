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
#include "Window.h"
#include "fftpack.h"

#include <math.h>
#define FFT_calcDataString "STD_ARRAY_DATA"


/* Converts NDArray callback data into standard asyn arrays
   (asynInt8Array, asynInt16Array, asynInt32Array, asynFloat32Array or
   asynFloat64Array); normally used for putting NDArray data in EPICS
   waveform records. It handles the data type conversion if the NDArray
   data type differs from the data type of the asyn interface. It
   flattens the NDArrays to a single dimension because asyn and EPICS
   do not support multi-dimensional arrays. */
class FFT_calc : public NDPluginDriver {
public:
    FFT_calc(const char *portName, int queueSize, int blockingCallbacks,
             const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
             size_t maxMemory, int priority, int stackSize);
    /* These methods override the virtual methods in the base class */
    void processCallbacks(NDArray *pArray);
protected:
    int FFT_calcData;
    #define FIRST_FFT_CALC_PARAM FFT_calcData
    #define LAST_FFT_CALC_PARAM FFT_calcData
    int window;
    int windowParameter;
private:
    enum windowType {RECT, HANN, HAMMING, BLACKMAN, BLACKMANHARRIS,
                     KAISERBESSEL, GAUSSIAN};
};
#define NUM_FFT_CALC_PARAMS ((int)(&LAST_FFT_CALC_PARAM - &FIRST_FFT_CALC_PARAM + 3))
static const char *driverName="FFT_calc";


void FFT_calc::processCallbacks(NDArray *pArray)
{
    int i, windowIndex;
    double *pWaveform, *pWaveIndexer, *pFFT, *pWindow, *pOutput, winParam;
    short *pData;
    NDArray *pProcessed;
    NDArrayInfo_t arrayInfo;
    NDPluginDriver::processCallbacks(pArray);
    pArray->getInfo(&arrayInfo);
    pProcessed = this->pNDArrayPool->alloc(1, &arrayInfo.ySize,
                                           (NDDataType_t)NDFloat64, 0, NULL);
    pData = (short *)pArray->pData;
    this->unlock();
    pWaveform = (double *) malloc(arrayInfo.ySize * sizeof(double));
    /* fftpack workspace must at least 2n + 15 in length */
    pFFT = (double *) malloc((2*arrayInfo.ySize + 15) * sizeof(double));
    pWindow = (double *) malloc(arrayInfo.ySize * sizeof(double));
    if (pWaveform == NULL || pFFT == NULL || pWindow == NULL) {
        this->lock();
        free((void *) pWaveform);
        free((void *) pFFT);
        free((void *) pWindow);
        return;
    }
    pWaveIndexer = pWaveform;
    for (i=0; i<arrayInfo.ySize; i++)
        *pWaveIndexer++ = (double) *pData++;
    pWaveIndexer = pWaveform;
    getIntegerParam(window, &windowIndex);
    switch (windowIndex) {
    case RECT:
        WindowRect(pWindow, arrayInfo.ySize);
        break;
    case HANN:
        WindowHann(pWindow, arrayInfo.ySize);
        break;
    case HAMMING:
        WindowHamming(pWindow, arrayInfo.ySize);
        break;
    case BLACKMAN:
        getDoubleParam(windowParameter, &winParam);
        WindowBlackman(pWindow, arrayInfo.ySize, winParam);
        break;
    case BLACKMANHARRIS:
        WindowBlackmanHarris(pWindow, arrayInfo.ySize);
        break;
    case KAISERBESSEL:
        getDoubleParam(windowParameter, &winParam);
        WindowKaiserBessel(pWindow, arrayInfo.ySize, winParam);
        break;
    case GAUSSIAN:
        getDoubleParam(windowParameter, &winParam);
        WindowGauss(pWindow, arrayInfo.ySize, winParam);
        break;
    }
    rffti(arrayInfo.ySize, pFFT);
    /* Apply the window */
    for (i=0; i<arrayInfo.ySize; i++) {
        *pWaveIndexer++ *= pWindow[i];
    }
    pWaveIndexer = pWaveform;
    rfftf(arrayInfo.ySize, pWaveform, pFFT);
    pOutput = (double *) pProcessed->pData;
    *pOutput++ = *pFFT;
    for (i=1; i<arrayInfo.ySize; i++) {
        const double imag = pFFT[2*i - 1], real = pFFT[2*i];
        *pOutput++ = sqrt(imag * imag + real * real);
    }
    this->lock();
    free(pWaveform);
    free((void *) pFFT);
    free((void *) pWindow);
    this->getAttributes(pProcessed->pAttributeList);
    this->unlock();
    this->doCallbacksGenericPointer(pProcessed, NDArrayData, 0);
    this->lock();
    pProcessed->release();
    this->callParamCallbacks();
}

/* Constructor for FFT_calc; all parameters are simply passed to
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

FFT_calc::FFT_calc(const char *portName, int queueSize, int blockingCallbacks,
                   const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
                   size_t maxMemory, int priority, int stackSize)
    /* Invoke the base class constructor */
    : NDPluginDriver(portName, queueSize, blockingCallbacks, NDArrayPort,
                     NDArrayAddr, 1, NUM_FFT_CALC_PARAMS, maxBuffers, maxMemory,
                     asynGenericPointerMask, asynGenericPointerMask,
                     /* asynFlags is set to 0, because this plugin cannot
                        block and is not multi-device. It does autoconnect */
                     0, 1, priority, stackSize)
{
    createParam(FFT_calcDataString, asynParamGenericPointer, &FFT_calcData);
    createParam("Window", asynParamInt32, &window);
    createParam("WindowParameter", asynParamFloat64, &windowParameter);
    /*setIntegerParam(window, 2);
    int wind;
    getIntegerParam(window, &wind);
    printf("Start Window: %d\n", wind);*/
    /* Set the plugin type string */
    setStringParam(NDPluginDriverPluginType, "FFT_calc");
    /* Try to connect to the NDArray port */
    connectToArrayPort();
}

/** Configuration command */
extern "C" int FFT_calcConfigure(const char *portName, int queueSize,
                                 int blockingCallbacks, const char *NDArrayPort,
                                 int NDArrayAddr, int maxBuffers,
                                 size_t maxMemory, int priority, int stackSize)
{
    new FFT_calc(portName, queueSize, blockingCallbacks, NDArrayPort,
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
static const iocshFuncDef initFuncDef = {"FFT_calcConfigure", 9, initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    FFT_calcConfigure(args[0].sval, args[1].ival, args[2].ival,
                      args[3].sval, args[4].ival, args[5].ival,
                      args[6].ival, args[7].ival, args[8].ival);
}

extern "C" void FFT_calc_register(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
}

extern "C" {
    epicsExportRegistrar(FFT_calc_register);
}
