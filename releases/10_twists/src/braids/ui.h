#ifndef BRAIDS_UI_H_
#define BRAIDS_UI_H_

#include "braids/drivers/display.h"
#include "braids/drivers/switch.h"

#include "braids/settings.h"

namespace braids {

class Ui {
 public:
  Ui() { }
  ~Ui() { }
  
  void Init();
  void Poll(uint16_t switch_val);

 private:
  void IncrementShape();
  void UpdateDisplay();
  
  Display display_;
  Switch switch_;

  uint8_t selected_shape = 0;

  DISALLOW_COPY_AND_ASSIGN(Ui);
};

}  // namespace braids

#endif // BRAIDS_UI_H_
