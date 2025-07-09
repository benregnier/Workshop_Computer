#include "braids/drivers/cv_out.h"
#include "braids/drivers/calibration.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

namespace braids {
  
void CvOut::Init() {
  dmaBufferAddress = &(dmaBuffer[0]);
  for (int i = 0; i < 16; i++) dmaBuffer[i] = 0;

  // Setup CV PWM
  gpio_set_function(PIN_CV1_OUT, GPIO_FUNC_PWM);
  gpio_set_function(PIN_CV2_OUT, GPIO_FUNC_PWM);

  // Create PWM config struct
  pwm_config config = pwm_get_default_config();
  pwm_config_set_wrap(&config, 2047); // 11-bit PWM

  // Initialize PWM for both CV outputs
  pwm_init(pwm_gpio_to_slice_num(PIN_CV1_OUT), &config, true);
  pwm_init(pwm_gpio_to_slice_num(PIN_CV2_OUT), &config, true);

  // Setup chained DMAs for continuous operation
  int slice_num = pwm_gpio_to_slice_num(PIN_CV1_OUT);

  int pwm_dma_chan = dma_claim_unused_channel(true);
  int control_dma_chan = dma_claim_unused_channel(true);

  // Data transfer DMA configuration
  dma_channel_config pwm_dma_chan_config = dma_channel_get_default_config(pwm_dma_chan);
  channel_config_set_transfer_data_size(&pwm_dma_chan_config, DMA_SIZE_32);
  channel_config_set_read_increment(&pwm_dma_chan_config, true);
  channel_config_set_write_increment(&pwm_dma_chan_config, false);
  channel_config_set_dreq(&pwm_dma_chan_config, DREQ_PWM_WRAP0 + slice_num);
  channel_config_set_chain_to(&pwm_dma_chan_config, control_dma_chan);

  dma_channel_configure(
    pwm_dma_chan,
    &pwm_dma_chan_config,
    &pwm_hw->slice[slice_num].cc, // destination
    dmaBuffer,                    // source
    16,                           // number of values
    false                         // don't start now
  );

  // Control DMA configuration
  dma_channel_config control_dma_chan_config = dma_channel_get_default_config(control_dma_chan);
  channel_config_set_transfer_data_size(&control_dma_chan_config, DMA_SIZE_32);
  channel_config_set_read_increment(&control_dma_chan_config, false);
  channel_config_set_write_increment(&control_dma_chan_config, false);
  channel_config_set_chain_to(&control_dma_chan_config, pwm_dma_chan);

  dma_channel_configure(
    control_dma_chan,
    &control_dma_chan_config,
    &dma_hw->ch[pwm_dma_chan].read_addr,
    &dmaBufferAddress,
    1,
    false);

  // Initialize CV outputs to 0V
  Set(0, 262144);
  Set(1, 262144);

  // Start DMA
  dma_start_channel_mask(1u << control_dma_chan);

  // Set initial PWM levels
  pwm_set_gpio_level(PIN_CV1_OUT, 1024);
  pwm_set_gpio_level(PIN_CV2_OUT, 1024);

  calibration.init();
}

void CvOut::SetFloat(int channel, float fvalue) {
  // uint32_t value;
  // if (fvalue <= 0.0f)
  //   value = 0;
  // else
  //   value = fvalue * 524288;
  // if (value > 524288) value = 524288;

  uint32_t value = calibration.voltageToCalibratedOut(channel, fvalue);
  Set(channel, value);
}

void CvOut::Set(int channel, uint32_t value) {
  int32_t error = 127;
  for (int i = 0; i < 16; i++) {
    uint32_t truncated_val = (value - error) & 0x00FFFF00;
    error += truncated_val - value; 
    if (channel == 0)
      dmaBuffer[i] = (dmaBuffer[i] & 0x0000FFFF) | (truncated_val << 8);
    else
      dmaBuffer[i] = (dmaBuffer[i] & 0xFFFF0000) | (truncated_val >> 8);
  }
}

}  // namespace braids
