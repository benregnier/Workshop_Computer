#ifndef BRAIDS_DRIVERS_CVOUT_H_
#define BRAIDS_DRIVERS_CVOUT_H_

#include "pico/stdlib.h"
#include "braids/drivers/calibration.h"

#define PIN_CV2_OUT         22
#define PIN_CV1_OUT         23

namespace braids {

class CvOut {
  public:
    void Init();
    
    // Set PWM value for a channel using floating-point value (0.0 - 1.0)
    void SetFloat(int channel, float fvalue);

    // Set PWM value for a channel using 19-bit integer value (0 - 524288)
    void Set(int channel, uint32_t value);

  private:

    Calibration calibration;
    volatile uint32_t val[2];
    uint32_t dmaBuffer[16];
    uint32_t* dmaBufferAddress;
};
}

#endif