/* dtacq_adc.cpp
 *
 * This is a driver for a simulated area detector.
 *
 * Author: Mark Rivers
 *         University of Chicago
 *
 * Created:  March 20, 2008
 *
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <asynOctetSyncIO.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>

#include "ADDriver.h"
#include <epicsExport.h>

static const char *driverName = "dtacq_adc";
class dtacq_adc : public ADDriver {
public:
    dtacq_adc(const char *portName, char *ipPortName, int nChannels,
              int nSamples, int maxBuffers,
              size_t maxMemory, int priority, int stackSize);
    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual void setShutter(int open);
    virtual void report(FILE *fp, int details);
    void simTask();

protected:
    int SimGainX;
    #define FIRST_SIM_DETECTOR_PARAM SimGainX
    int SimGainY;
    int SimGainRed;
    int SimGainGreen;
    int SimGainBlue;
    int SimNoise;
    int SimResetImage;
    int SimMode;
    int SimPeakStartX;
    int SimPeakStartY;
    int SimPeakWidthX;
    int SimPeakWidthY;
    int SimPeakNumX;
    int SimPeakNumY;
    int SimPeakStepX;
    int SimPeakStepY;
    int SimPeakHeightVariation;
    #define LAST_SIM_DETECTOR_PARAM SimPeakHeightVariation

private:
    /* These are the methods that are new to this class */
    /*template <typename epicsType> int computeArray(int sizeX, int sizeY);
    template <typename epicsType> int computeLinearRampArray(int sizeX, int sizeY);
    template <typename epicsType> int computePeaksArray(int sizeX, int sizeY);*/
    int computeArray(int n_samples, int n_channels);
    int readArray(int n_samples, int n_channels);
    int computeImage();
    /* Our data */
    epicsEventId startEventId;
    epicsEventId stopEventId;
    NDArray *pRaw;
    asynUser *pasynUserIP;
};

typedef enum {
    SimModeLinearRamp,
    SimModePeaks,
}SimModes_t;

#define SimGainXString          "SIM_GAIN_X"
#define SimGainYString          "SIM_GAIN_Y"
#define SimGainRedString        "SIM_GAIN_RED"
#define SimGainGreenString      "SIM_GAIN_GREEN"
#define SimGainBlueString       "SIM_GAIN_BLUE"
#define SimNoiseString          "SIM_NOISE"
#define SimResetImageString     "RESET_IMAGE"
#define SimModeString           "SIM_MODE"
#define SimPeakStartXString     "SIM_PEAK_START_X"
#define SimPeakStartYString     "SIM_PEAK_START_Y"
#define SimPeakWidthXString     "SIM_PEAK_WIDTH_X"
#define SimPeakWidthYString     "SIM_PEAK_WIDTH_Y"
#define SimPeakNumXString       "SIM_PEAK_NUM_X"
#define SimPeakNumYString       "SIM_PEAK_NUM_Y"
#define SimPeakStepXString      "SIM_PEAK_STEP_X"
#define SimPeakStepYString      "SIM_PEAK_STEP_Y"
#define SimPeakHeightVariationString  "SIM_PEAK_HEIGHT_VARIATION"
#define NUM_SIM_DETECTOR_PARAMS ((int)(&LAST_SIM_DETECTOR_PARAM - &FIRST_SIM_DETECTOR_PARAM + 1))

int dtacq_adc::readArray(int n_samples, int n_channels)
{
    int status = asynSuccess;
    size_t nread = 0;
    int eomReason, total_read = 0;
    while (total_read < n_samples * n_channels) {
        status = pasynOctetSyncIO->read(pasynUserIP,
                                        (char *) this->pRaw->pData+total_read,
                                        n_samples*n_channels*2 - nread, 5.0, &nread,
                                        &eomReason);
        total_read += nread;
    }
    if (status != asynSuccess) {
        printf("N read: %u %d %d\n", nread, eomReason, status);
    }
    return status;
}

int dtacq_adc::computeArray(int n_samples, int n_channels)
{
    int i, j;
    char * pData = (char *) this->pRaw->pData;
    for (i=0; i<n_channels; i++) {
        for (j=0; j<n_samples; j++) {
          (*pData++) = (char) char(127*sin(0.01*j) + 128);
        }
    }
    return 1;
}

/* Controls the shutter */
void dtacq_adc::setShutter(int open)
{
    int shutterMode;

    getIntegerParam(ADShutterMode, &shutterMode);
    if (shutterMode == ADShutterModeDetector) {
        /* Simulate a shutter by just changing the status readback */
        setIntegerParam(ADShutterStatus, open);
    } else {
        /* For no shutter or EPICS shutter call the base class method */
        ADDriver::setShutter(open);
    }
}

/* Computes the new image data */
int dtacq_adc::computeImage()
{
    int status = asynSuccess;
    NDDataType_t dataType;
    int itemp;
    int binX, binY, minX, minY, sizeX, sizeY, reverseX, reverseY;
    int xDim=0, yDim=1, colorDim=-1;
    int resetImage;
    int maxSizeX, maxSizeY;
    int colorMode;
    int ndims=0;
    NDDimension_t dimsOut[3];
    size_t dims[3];
    NDArrayInfo_t arrayInfo;
    NDArray *pImage;
    const char* functionName = "computeImage";
    /* NOTE: The caller of this function must have taken the mutex */
    status |= getIntegerParam(ADBinX,         &binX);
    status |= getIntegerParam(ADBinY,         &binY);
    status |= getIntegerParam(ADMinX,         &minX);
    status |= getIntegerParam(ADMinY,         &minY);
    status |= getIntegerParam(ADSizeX,        &sizeX);
    status |= getIntegerParam(ADSizeY,        &sizeY);
    status |= getIntegerParam(ADReverseX,     &reverseX);
    status |= getIntegerParam(ADReverseY,     &reverseY);
    status |= getIntegerParam(ADMaxSizeX,     &maxSizeX);
    status |= getIntegerParam(ADMaxSizeY,     &maxSizeY);
    status |= getIntegerParam(NDColorMode,    &colorMode);
    status |= getIntegerParam(NDDataType,     &itemp);
    dataType = (NDDataType_t)itemp;
    status |= getIntegerParam(SimResetImage,  &resetImage);
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: error getting parameters\n",
                          driverName, functionName);
    /* Make sure parameters are consistent, fix them if they are not */
    if (binX < 1) {
        binX = 1;
        status |= setIntegerParam(ADBinX, binX);
    }
    if (binY < 1) {
        binY = 1;
        status |= setIntegerParam(ADBinY, binY);
    }
    if (minX < 0) {
        minX = 0;
        status |= setIntegerParam(ADMinX, minX);
    }
    if (minY < 0) {
        minY = 0;
        status |= setIntegerParam(ADMinY, minY);
    }
    if (minX > maxSizeX-1) {
        minX = maxSizeX-1;
        status |= setIntegerParam(ADMinX, minX);
    }
    if (minY > maxSizeY-1) {
        minY = maxSizeY-1;
        status |= setIntegerParam(ADMinY, minY);
    }
    if (minX+sizeX > maxSizeX) {
        sizeX = maxSizeX-minX;
        status |= setIntegerParam(ADSizeX, sizeX);
    }
    if (minY+sizeY > maxSizeY) {
        sizeY = maxSizeY-minY;
        status |= setIntegerParam(ADSizeY, sizeY);
    }
    ndims = 2;
    xDim = 0;
    yDim = 1;
    if (resetImage) {
    /* Free the previous raw buffer */
        if (this->pRaw) this->pRaw->release();
        /* Allocate the raw buffer we use to compute images. */
        dims[xDim] = maxSizeX;
        dims[yDim] = maxSizeY;
        this->pRaw = this->pNDArrayPool->alloc(ndims, dims, NDInt16, 0, NULL);
        if (!this->pRaw) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s:%s: error allocating raw buffer\n",
                      driverName, functionName);
            return(status);
        }
    }
    status |= readArray(maxSizeX, maxSizeY);
    /* Extract the region of interest with binning.
       If the entire image is being used (no ROI or binning) that's OK because
       convertImage detects that case and is very efficient */
    this->pRaw->initDimension(&dimsOut[xDim], sizeX);
    this->pRaw->initDimension(&dimsOut[yDim], sizeY);
    if (ndims > 2) this->pRaw->initDimension(&dimsOut[colorDim], 3);
    dimsOut[xDim].binning = binX;
    dimsOut[xDim].offset  = minX;
    dimsOut[xDim].reverse = reverseX;
    dimsOut[yDim].binning = binY;
    dimsOut[yDim].offset  = minY;
    dimsOut[yDim].reverse = reverseY;
    /* We save the most recent image buffer so it can be used in the
       read() function. Now release it before getting a new version. */
    if (this->pArrays[0]) this->pArrays[0]->release();
    status = this->pNDArrayPool->convert(this->pRaw,
                                         &this->pArrays[0],
                                         dataType,
                                         dimsOut);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error allocating buffer in convert()\n",
                    driverName, functionName);
        return(status);
    }
    pImage = this->pArrays[0];
    pImage->getInfo(&arrayInfo);
    /* Multiply by a scale factor to convert -32767..32768 into -10V..10V
     * Also divide by the binning factor so we get a sensible scale for averaging */
    double mult = 10.0 / 32768 / binX / binY;
    double * pData = (double *) pImage->pData;
    for (unsigned int i=0; i<arrayInfo.nElements; i++) {
    	pData[i] *= mult;
    }

    status = asynSuccess;
    status |= setIntegerParam(NDArraySize,  (int)arrayInfo.totalBytes);
    status |= setIntegerParam(NDArraySizeX, (int)pImage->dims[xDim].size);
    status |= setIntegerParam(NDArraySizeY, (int)pImage->dims[yDim].size);
    status |= setIntegerParam(SimResetImage, 0);
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s:%s: error setting parameters\n",
                          driverName, functionName);
    return(status);
}

static void simTaskC(void *drvPvt)
{
    dtacq_adc *pPvt = (dtacq_adc *)drvPvt;
    pPvt->simTask();
}

/* This thread calls computeImage to compute new image data and does the
   callbacks to send it to higher layers. It implements the logic for single,
   multiple or continuous acquisition. */
void dtacq_adc::simTask()
{
    int status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int arrayCallbacks;
    int acquire=0;
    NDArray *pImage;
    double acquireTime, acquirePeriod, delay;
    epicsTimeStamp startTime, endTime;
    double elapsedTime;
    const char *functionName = "simTask";
    this->lock();
    /* Loop forever */
    while (1) {
        /* If we are not acquiring then wait for a semaphore that
           is given when acquisition is started */
        if (!acquire) {
            /* Release the lock while we wait for an event that
               says acquire has started, then lock again */
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: waiting for acquire to start\n",
                      driverName, functionName);
            this->unlock();
            status = epicsEventWait(this->startEventId);
            this->lock();
            acquire = 1;
            setStringParam(ADStatusMessage, "Acquiring data");
            setIntegerParam(ADNumImagesCounter, 0);
        }

        /* We are acquiring. */
        /* Get the current time */
        epicsTimeGetCurrent(&startTime);
        getIntegerParam(ADImageMode, &imageMode);
        /* Get the exposure parameters */
        getDoubleParam(ADAcquireTime, &acquireTime);
        getDoubleParam(ADAcquirePeriod, &acquirePeriod);
        setIntegerParam(ADStatus, ADStatusAcquire);
        /* Open the shutter */
        setShutter(ADShutterOpen);
        /* Call the callbacks to update any changes */
        callParamCallbacks();
        /* Simulate being busy during the exposure time. Use
           epicsEventWaitWithTimeout so that manually stopping the
           acquisition will work */
        if (acquireTime > 0.0) {
            this->unlock();
            status = epicsEventWaitWithTimeout(this->stopEventId, acquireTime);
            this->lock();
        } else {
            status = epicsEventTryWait(this->stopEventId);
        }
        if (status == epicsEventWaitOK) {
            acquire = 0;
            if (imageMode == ADImageContinuous) {
                setIntegerParam(ADStatus, ADStatusIdle);
            } else {
                setIntegerParam(ADStatus, ADStatusAborted);
            }
            callParamCallbacks();
        }


        /* Update the image */
        status = computeImage();

        if (status) continue;

        /* Close the shutter */
        setShutter(ADShutterClosed);

        if (!acquire) continue;

        setIntegerParam(ADStatus, ADStatusReadout);
        /* Call the callbacks to update any changes */
        callParamCallbacks();

        pImage = this->pArrays[0];

        /* Get the current parameters */
        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
        imageCounter++;
        numImagesCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);

        /* Put the frame number and time stamp into the buffer */
        pImage->uniqueId = imageCounter;
        pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;

        /* Get any attributes that have been defined for this driver */
        this->getAttributes(pImage->pAttributeList);

        if (arrayCallbacks) {
            /* Call the NDArray callback */
            /* Must release the lock here, or we can get into a deadlock, because we can
               block on the plugin lock, and the plugin can be calling us */
            this->unlock();
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: calling imageData callback\n", driverName, functionName);
            doCallbacksGenericPointer(pImage, NDArrayData, 0);
            this->lock();
        }

        /* See if acquisition is done */
        if ((imageMode == ADImageSingle) ||
            ((imageMode == ADImageMultiple) &&
             (numImagesCounter >= numImages))) {
            /* First do callback on ADStatus. */
            setStringParam(ADStatusMessage, "Waiting for acquisition");
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();
            acquire = 0;
            setIntegerParam(ADAcquire, acquire);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: acquisition completed\n", driverName, functionName);
        }
        /* Call the callbacks to update any changes */
        callParamCallbacks();
        /* If we are acquiring then sleep for the acquire period minus elapsed time. */
        if (acquire) {
            epicsTimeGetCurrent(&endTime);
            elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
            delay = acquirePeriod - elapsedTime;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                      "%s:%s: delay=%f\n",
                      driverName, functionName, delay);
            if (delay >= 0.0) {
                /* We set the status to waiting to indicate we are in the period delay */
                setIntegerParam(ADStatus, ADStatusWaiting);
                callParamCallbacks();
                this->unlock();
                status = epicsEventWaitWithTimeout(this->stopEventId, delay);
                this->lock();
                if (status == epicsEventWaitOK) {
                    acquire = 0;
                    if (imageMode == ADImageContinuous) {
                        setIntegerParam(ADStatus, ADStatusIdle);
                    } else {
                        setIntegerParam(ADStatus, ADStatusAborted);
                    }
                    callParamCallbacks();
                }
            }
        }
    }
}


/* Called when asyn clients call pasynInt32->write().
   This function performs actions for some parameters, including ADAcquire, ADColorMode, etc.
   For all parameters it sets the value in the parameter library and calls any registered callbacks..
   \param[in] pasynUser pasynUser structure that encodes the reason and address.
   \param[in] value Value to write. */
asynStatus dtacq_adc::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int adstatus;
    int acquiring;
    int imageMode;
    asynStatus status = asynSuccess;
    /* Ensure that ADStatus is set correctly before we set ADAcquire.*/
    getIntegerParam(ADStatus, &adstatus);
    getIntegerParam(ADAcquire, &acquiring);
    if (function == ADAcquire) {
        if (value && !acquiring) {
            setStringParam(ADStatusMessage, "Acquiring data");
            getIntegerParam(ADImageMode, &imageMode);
        }
        if (!value && acquiring) {
            setStringParam(ADStatusMessage, "Acquisition stopped");
            if (imageMode == ADImageContinuous) {
                setIntegerParam(ADStatus, ADStatusIdle);
            } else {
                setIntegerParam(ADStatus, ADStatusAborted);
            }
            setIntegerParam(ADStatus, ADStatusAcquire);
        }
    }
    callParamCallbacks();
    /* Set the parameter and readback in the parameter library. This may be
       overwritten when we read back the status at the end, but that's OK */
    status = setIntegerParam(function, value);
    /* For a real detector this is where the parameter is sent to the hardware */
    if (function == ADAcquire) {
        if (value && !acquiring) {
            /* Send an event to wake up the simulation task. It won't actually
               start generating new images until we release the lock below */
            epicsEventSignal(this->startEventId);
        }
        if (!value && acquiring) {
            /* This was a command to stop acquisition */
            /* Send the stop event */
            epicsEventSignal(this->stopEventId);
        }
    } else if ((function == NDDataType) ||
               (function == NDColorMode) ||
               (function == SimMode)) {
        status = setIntegerParam(SimResetImage, 1);
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_SIM_DETECTOR_PARAM) status = ADDriver::writeInt32(pasynUser, value);
    }
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:writeInt32 error, status=%d function=%d, value=%d\n",
              driverName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:writeInt32: function=%d, value=%d\n",
              driverName, function, value);
    return status;
}

/* Called when asyn clients call pasynFloat64->write().
   This function performs actions for some parameters, including ADAcquireTime, ADGain, etc.
   For all parameters it sets the value in the parameter library and calls any registered callbacks..
   \param[in] pasynUser pasynUser structure that encodes the reason and address.
   \param[in] value Value to write.
*/
asynStatus dtacq_adc::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(function, value);
    /* Changing any of the following parameters requires recomputing the base image */
    if ((function == ADAcquireTime) ||
        (function == ADGain) ||
        (function == SimGainX) ||
        (function == SimGainY) ||
        (function == SimGainRed) ||
        (function == SimGainGreen) ||
        (function == SimGainBlue)) {
            status = setIntegerParam(SimResetImage, 1);
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_SIM_DETECTOR_PARAM) status = ADDriver::writeFloat64(pasynUser, value);
    }
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:writeFloat64 error, status=%d function=%d, value=%f\n",
              driverName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:writeFloat64: function=%d, value=%f\n",
              driverName, function, value);
    return status;
}

/* Report status of the driver.
   Prints details about the driver if details>0.
   It then calls the ADDriver::report() method.
   \param[in] fp File pointed passed by caller where the output is written to.
   \param[in] details If >0 then driver details are printed.
*/
void dtacq_adc::report(FILE *fp, int details)
{

    fprintf(fp, "Simulation detector %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

/* Constructor for dtacq_adc; most parameters are simply passed to
   ADDriver::ADDriver. After calling the base class constructor this method
   creates a thread to compute the simulated detector data, and sets
   reasonable default values for parameters defined in this class,
   asynNDArrayDriver and ADDriver.
   \param[in] portName The name of the asyn port driver to be created.
   \param[in] maxSizeX The maximum X dimension of the images that this
                       driver can create.
   \param[in] maxSizeY The maximum Y dimension of the images that this
                       driver can create.
   \param[in] dataType The initial data type (NDDataType_t) of the images
                       that this driver will create.
   \param[in] maxBuffers The maximum number of NDArray buffers that the
                         NDArrayPool for this driver is allowed to allocate.
                         Set this to -1 to allow an unlimited number of buffers.
   \param[in] maxMemory The maximum amount of memory that the NDArrayPool
                        for this driver is allowed to allocate. Set this to
                        -1 to allow an unlimited amount of memory.
   \param[in] priority The thread priority for the asyn port driver thread
                       if ASYN_CANBLOCK is set in asynFlags.
   \param[in] stackSize The stack size for the asyn port driver thread if
                        ASYN_CANBLOCK is set in asynFlags.
*/
dtacq_adc::dtacq_adc(const char *portName, char *ipPortName, int nChannels,
                     int nSamples, int maxBuffers,
                     size_t maxMemory, int priority, int stackSize)
    : ADDriver(portName, 1, NUM_SIM_DETECTOR_PARAMS, maxBuffers, maxMemory,
               0, 0, /* No interfaces beyond those set in ADDriver.cpp */
               0, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize),
      pRaw(NULL)
{
    int status = asynSuccess;
    const char *functionName = "dtacq_adc";
    /* Create the epicsEvents for signaling to the simulate task
       when acquisition starts and stops */
    this->startEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId) {
        printf("%s:%s epicsEventCreate failure for start event\n",
            driverName, functionName);
        return;
    }
    this->stopEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->stopEventId) {
        printf("%s:%s epicsEventCreate failure for stop event\n",
            driverName, functionName);
        return;
    }
    createParam(SimGainXString, asynParamFloat64, &SimGainX);
    createParam(SimGainYString, asynParamFloat64, &SimGainY);
    createParam(SimGainRedString, asynParamFloat64, &SimGainRed);
    createParam(SimGainGreenString, asynParamFloat64, &SimGainGreen);
    createParam(SimGainBlueString, asynParamFloat64, &SimGainBlue);
    createParam(SimNoiseString, asynParamInt32, &SimNoise);
    createParam(SimResetImageString, asynParamInt32, &SimResetImage);
    createParam(SimModeString, asynParamInt32, &SimMode);
    createParam(SimPeakNumXString, asynParamInt32, &SimPeakNumX);
    createParam(SimPeakNumYString, asynParamInt32, &SimPeakNumY);
    createParam(SimPeakStepXString, asynParamInt32, &SimPeakStepX);
    createParam(SimPeakStepYString, asynParamInt32, &SimPeakStepY);
    createParam(SimPeakStartXString, asynParamInt32, &SimPeakStartX);
    createParam(SimPeakStartYString, asynParamInt32, &SimPeakStartY);
    createParam(SimPeakWidthXString, asynParamInt32, &SimPeakWidthX);
    createParam(SimPeakWidthYString, asynParamInt32, &SimPeakWidthY);
    createParam(SimPeakHeightVariationString, asynParamInt32,
                &SimPeakHeightVariation);
    /* Set some default values for parameters */
    status =  setStringParam (ADManufacturer, "Simulated detector");
    status |= setStringParam (ADModel, "Basic simulator");
    status |= setIntegerParam(ADMaxSizeX, nChannels);
    status |= setIntegerParam(ADMaxSizeY, nSamples);
    status |= setIntegerParam(ADSizeX, nChannels);
    status |= setIntegerParam(ADSizeY, nSamples);
    status |= setIntegerParam(NDArraySizeX, nChannels);
    status |= setIntegerParam(NDArraySizeY, nSamples);
    status |= setIntegerParam(NDArraySize, 0);
    status |= setIntegerParam(NDDataType, NDFloat64);
    status |= setIntegerParam(ADImageMode, ADImageContinuous);
    status |= setDoubleParam (ADAcquireTime, .001);
    status |= setDoubleParam (ADAcquirePeriod, .005);
    status |= setIntegerParam(ADNumImages, 100);
    status |= setIntegerParam(SimNoise, 3);
    status |= setIntegerParam(SimResetImage, 1);
    status |= setDoubleParam (SimGainX, 1);
    status |= setDoubleParam (SimGainY, 1);
    status |= setDoubleParam (SimGainRed, 1);
    status |= setDoubleParam (SimGainGreen, 1);
    status |= setDoubleParam (SimGainBlue, 1);
    status |= setIntegerParam(SimMode, 0);
    status |= setIntegerParam(SimPeakStartX, 1);
    status |= setIntegerParam(SimPeakStartY, 1);
    status |= setIntegerParam(SimPeakWidthX, 10);
    status |= setIntegerParam(SimPeakWidthY, 20);
    status |= setIntegerParam(SimPeakNumX, 1);
    status |= setIntegerParam(SimPeakNumY, 1);
    status |= setIntegerParam(SimPeakStepX, 1);
    status |= setIntegerParam(SimPeakStepY, 1);
    status |= setIntegerParam(SimPeakHeightVariation, 3);
    if (status) {
        printf("%s: unable to set camera parameters\n", functionName);
        return;
    }
    /* Create the thread that updates the images */
    status = (epicsThreadCreate("SimDetTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)simTaskC,
                                this) == NULL);
    if (status) {
        printf("%s:%s epicsThreadCreate failure for image task\n",
            driverName, functionName);
        return;
    }
    /* Connect to the ip port */
    pasynOctetSyncIO->connect(ipPortName, 0, &this->pasynUserIP, NULL);
}
/* Configuration command, called directly or from iocsh */
extern "C" int dtacq_adcConfig(const char *portName, char *ipPortName, int nChannels, int nSamples,
                                 int maxBuffers, int maxMemory, int priority, int stackSize)
{
    new dtacq_adc(portName, ipPortName, nChannels, nSamples,
                    (maxBuffers < 0) ? 0 : maxBuffers,
                    (maxMemory < 0) ? 0 : maxMemory,
                    priority, stackSize);
    return(asynSuccess);
}
/* Code for iocsh registration */
static const iocshArg dtacq_adcConfigArg0 = {"Port name", iocshArgString};
static const iocshArg dtacq_adcConfigArg1 = {"IP Port asyn name",
                                             iocshArgString};
static const iocshArg dtacq_adcConfigArg2 = {"N Channels", iocshArgInt};
static const iocshArg dtacq_adcConfigArg3 = {"N Samples / frame", iocshArgInt};
static const iocshArg dtacq_adcConfigArg4 = {"maxBuffers", iocshArgInt};
static const iocshArg dtacq_adcConfigArg5 = {"maxMemory", iocshArgInt};
static const iocshArg dtacq_adcConfigArg6 = {"priority", iocshArgInt};
static const iocshArg dtacq_adcConfigArg7 = {"stackSize", iocshArgInt};
static const iocshArg * const dtacq_adcConfigArgs[] =  {&dtacq_adcConfigArg0,
                                                        &dtacq_adcConfigArg1,
                                                        &dtacq_adcConfigArg2,
                                                        &dtacq_adcConfigArg3,
                                                        &dtacq_adcConfigArg4,
                                                        &dtacq_adcConfigArg5,
                                                        &dtacq_adcConfigArg6,
                                                        &dtacq_adcConfigArg7};
static const iocshFuncDef configdtacq_adc = {"dtacq_adcConfig", 8,
                                             dtacq_adcConfigArgs};
static void configdtacq_adcCallFunc(const iocshArgBuf *args)
{
    dtacq_adcConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
                    args[4].ival, args[5].ival, args[6].ival, args[7].ival);
}

static void dtacq_adcRegister(void)
{
    iocshRegister(&configdtacq_adc, configdtacq_adcCallFunc);
}

extern "C" {
epicsExportRegistrar(dtacq_adcRegister);
}
