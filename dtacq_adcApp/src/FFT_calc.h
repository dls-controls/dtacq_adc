#include "NDPluginDriver.h"

class FFT_calc : public NDPluginDriver {
  public:
    FFT_calc(const char *portName, int queueSize, int blockingCallbacks,
             const char *NDArrayPort, int NDArrayAddr, int maxBuffers,
             size_t maxMemory, int priority, int stackSize);
    void processCallbacks(NDArray *pArray);

  private:
    enum window {RECT, HANN, HAMMING, BLACKMAN, BLACKMANHARRIS,
                 KAISERBESSEL, GAUSSIAN}
    int FFT_calcWindow;
