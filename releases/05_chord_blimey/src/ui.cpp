#include "ui.h"
#include "pico/rand.h"
#include <math.h>

void UI::init(Computer* computer) {
    _computer = computer;

    coin_weight[0] = UINT32_MAX;
    coin_weight[1] = UINT32_MAX;
}

int UI::getArpLength() { 
    return fix_length_on ? fix_length : -1;
}

float UI::getRootVolts() {
    return root_volts;
}

uint UI::getNoteLengthMS() {
    // invert knob value and curve it so we have more control over the faster end
    return pow(2, (4095 - _computer->getPotZValue()) / 300) + 20;
}

uint UI::getChord() {
    float knoby_volts = _computer->getPotYValue() * MUX_BY_VOLT;
    float cv2_volts = _computer->getCVInVolts(2);

    float total_volts = cv2_volts + knoby_volts;
    
    if(total_volts > 1.0f) {
        return 11;
    } else if(total_volts < 0.0f) {
        return 0;
    } else {
        return floor(total_volts / VOLT_SEMITONE);
    }
}

void UI::update() {
    float knobx_volts = _computer->getPotXValue() * MUX_BY_VOLT;
    root_volts = _computer->getCVInVolts(1) + knobx_volts;
}

void UI::checkSwitch() {
    // switch 
    // up = play full length of chord
    // mid = limit number of notes
    // down = toggle number of notes
    ComputerSwitchState switch_state = _computer->getSwitchState();
    if(switch_state != prev_switch_state) {
        if(switch_state == ComputerSwitchState::DOWN) {
            fix_length++;
            if(fix_length > 6) {
                fix_length = 1;
            }
        } 
        else {
            fix_length_on = switch_state == ComputerSwitchState::MID;
        }

        if(switch_state == ComputerSwitchState::DOWN || switch_state == ComputerSwitchState::MID) {
            last_switch_change = to_ms_since_boot(get_absolute_time());
            _computer->setLEDs(0x3f >> (6 - fix_length));
        } else {
            last_switch_change = 0;
        }

        prev_switch_state = switch_state;
    }
}

void UI::spinRandomOuts() {
    bool any_changed = false;
    for(int i=0; i<2; i++) {
        uint32_t change_value = get_rand_32();
        if(coin_weight[i] < change_value) {
            coin_weight[i] = UINT32_MAX;

            // output a random value from 0v to 1v
            uint r = get_rand_32() * 0.000000002561137081;
            uint output_val = 1024 - (r * 14.22222222) + 0.5;

            _computer->setAudioOut(i + 1, output_val);
            any_changed = true;
            
        }
        else {
            // increase probability of changing next time
            coin_weight[i] -= 0xfffffff;
        }
    }

    // add delay so outputs settle
    if(any_changed) {
        sleep_us(500);
    }
}

