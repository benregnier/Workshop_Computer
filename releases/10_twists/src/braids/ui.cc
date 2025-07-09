#include "braids/ui.h"

namespace braids {

using namespace stmlib;

void Ui::Init() {
  switch_.Init();
  display_.Init();
  UpdateDisplay();
}

void Ui::Poll(uint16_t switch_val) {
  switch_.Debounce(switch_val);
  if (switch_.just_pressed()) {
    IncrementShape();
  }
}

void Ui::UpdateDisplay() {
  display_.SetBits(1 << settings.GetValue(SETTING_SELECTED_AVAILABLE_SHAPE));
}

void Ui::IncrementShape() {
  settings.IncrementSelectedShape();
  UpdateDisplay();
}

}  // namespace braids
