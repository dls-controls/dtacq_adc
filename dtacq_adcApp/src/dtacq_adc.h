#include "ADDriver.h"

const size_t bufferSize = 128;
#define STRINGLEN 128


#define DtacqADCInvertString         "INVERT"
#define DtacqAggregationSitesString  "AGGR_SITES"
#define DtacqMasterSiteString        "MASTER_SITE"
#define DtacqGainString              "RANGE"
#define DtacqChannelsString          "CHANNELS"
#define DtacqEnableScratchpadString  "USE_SAMPLE_COUNT"
#define DtacqBadFramesString         "BAD_ARRAY"

typedef enum DtacqModuleType {
  ACQ420=1,
  ACQ425=5,
  ACQ437=6
} DtacqModuleType;

static const char *driverName = "dtacq_adc";
class dtacq_adc : public ADDriver {
public:
    dtacq_adc(const char *portName, const char *dataPortName, const char *controlPortName,
              int nChannels, int moduleType, int nSamples, int maxBuffers, size_t maxMemory,
              const char *dataHostInfo, int priority, int stackSize);
    virtual int postInitConfig();
    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual void report(FILE *fp, int details);
    void dtacqTask();
    /* Parameters specific to dtacq_adc (areaDetector) */
    // ###TODO: Inversion is not currently implemented
    int DtacqAdcInvert;
#define DTACQ_FIRST_PARAMETER DtacqAdcInvert
    int DtacqAggregationSites;
    int DtacqMasterSite;
    int DtacqGain;
    int DtacqChannels;
    int DtacqEnableScratchpad;
    int DtacqBadFrames;
    //
#define DTACQ_NUM_PARAMETERS ((int) (&DtacqBadFrames - &DTACQ_FIRST_PARAMETER + 1))

private:
    /* Frame handling functions */
    int readArray(int n_samples, int n_channels, int nBytes);
    int computeImage();
    /* Connection handling and device communication functions */
    asynStatus getSiteInformation();
    asynStatus getDeviceParameter(const char *parameter, char *readBuffer,
                                  int bufferLen, const char *site=NULL);
    asynStatus setDeviceParameter(const char *parameter, const char *value, const char *site=NULL);
    void closeSocket();
    /* Data processing functions */
    asynStatus calculateConversionFactor(int gainSelection, double *factor);
    asynStatus calculateDataSize();
    asynStatus applyScaling(NDArray *pFrame, int nChannels, int skipElements);
    asynStatus applyBitMask(NDArray *pFrame, int nChannels, int skipElements);
    int nElements(NDArray *pFrame);
    /* Events */
    epicsEvent *acquireStartEvent;
    epicsEvent *acquireStopEvent;
    /* Raw frame (read from device data port) */
    NDArray *pRaw;
    /* Device communication parameters*/
    char dataPortName[STRINGLEN], dataHostInfo[STRINGLEN];
    asynUser *commonDataIPPort, *octetDataIPPort;
    asynUser *controlIPPort;
    /* Gain control parameters and value scaling */
    std::map<int, std::vector<double> > ranges;
    int moduleType;
    double count2volt;
    /* Mask to zero out the site/channel information in 24bit data */
    static const int bitMask = 0xffffff00;
    /* Override values for ACQ420FMC gain options */
    static const int ngvals = 4;
    char *gnames[ngvals];
    int gvals[ngvals];
    int gseverities[ngvals];
    bool cleanSampleSeen;
    // 64 bit so that ADC will overflow before we do (since it stores this as a 32 bit int)
    uint64_t sampleCount;
};
