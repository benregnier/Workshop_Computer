#include "braids/drivers/switch.h"

namespace braids {

void Switch::Init() {
  switch_state_ = 0xff;
}

void Switch::Debounce(uint16_t switch_val) {
  // wait for value to settle on startup
  if(!initialized && switch_val > SWITCH_DOWN_BOUNDRY) {
    initialized = true;
  }
  
  if(initialized) {
    switch_state_ = (switch_state_ << 1) | !(switch_val < SWITCH_DOWN_BOUNDRY);
  }
}

}  // namespace braids
