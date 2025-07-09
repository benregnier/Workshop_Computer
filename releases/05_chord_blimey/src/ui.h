#ifndef UI_H
#define UI_H

#include <stdio.h>
#include "computer.h"

#define VOLT_SEMITONE           0.0833333333333333 // 1 / 12
#define MUX_BY_VOLT             0.0002442002442 // 1 / 4095

class UI {
    public:
        void init(Computer* computer);
        int getArpLength();
        float getRootVolts();
        uint getNoteLengthMS();
        uint getChord();
        void spinRandomOuts();

        void update();
        void checkSwitch();
        uint32_t last_switch_change = 0;

    private:
        float root_volts = 0.0f;
        uint32_t coin_weight[2];

        uint8_t fix_length = 6;
        bool fix_length_on = false;
        ComputerSwitchState prev_switch_state;

        Computer* _computer;
};

#endif