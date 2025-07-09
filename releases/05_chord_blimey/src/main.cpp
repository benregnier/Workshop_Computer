/*
    Chord Blimey!
    A simple arpeggiator for Music Thing Workshop System Computer
    Tom Waters / Random Works Modular 2024

    Send a trigger into Pulse In 1 and get an arpeggio from CV Out & Pulse Out 1
    
    Pulse Out 2 fires when the last note has finished
    It also fires at startup so you can patch it to Pulse In 1 for looping arpeggios

    CV Out 2 outputs the root note
    The big Knob controls the speed
    The X Knob controls the root note
    The Y Knob controls the chord

    CV 1 In controls the root note (added to X Knob)
    CV 2 In controls the chord 0v - 1v (added to Y Knob)

    Audio Out 1 & 2 output a random voltage 0v - 1v for patching to CV Ins
    At the end of a chord a coin is tossed to decide if the output should change
    If starts off unlikely to change and gets more likely with each toss

    LEDs show the current note in the chord
*/

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "computer.h"
#include "ui.h"

#define TRIGGER_LENGTH 10

Computer computer;
UI ui;

int chords[12][7] = {
    {0, 4, 7, -1},              // M
    {0, 4, 7, 11, -1},          // M7
    {0, 4, 7, 11, 14, -1},      // M9
    {0, 4, 7, 11, 14, 17, -1},  // M11
    {0, 5, 7, -1},              // SUS4
    {0, 4, 8, -1},              // AUG
    {0, 3, 6, -1},              // DIM
    {0, 4, 7, 10, -1},          // DOM7
    {0, 3, 7, 10, 14, 17, -1},  // m11
    {0, 3, 7, 10, 14, -1},      // m9
    {0, 3, 7, 10, -1},          // m7
    {0, 3, 7, -1}               // m
};

bool pulse_in_got[2] = {false, false};
void pulsein_callback(uint gpio, uint32_t events) {
    pulse_in_got[gpio - PIN_PULSE1_IN] = true;
}

// current play state
bool chord_play = false;
uint chord = 0;
uint chord_note = 0;
uint32_t last_note_time = 0;

int arp_count = 0;
int arp_length = -1;

int main() {
    stdio_init_all();
    computer.init();
    computer.calibrateIfSwitchDown();

    computer.setPulseCallback(1, pulsein_callback);
    ui.init(&computer);

    // output trigger on pulse 2 to start looping if patched
    computer.setTimedPulse(2, TRIGGER_LENGTH);

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        computer.poll();
        ui.checkSwitch();
        arp_length = ui.getArpLength();

        // if we get a pulse start a new arp
        if(pulse_in_got[0]) {
            pulse_in_got[0] = false;
            chord_note = 0;
            arp_count = 0;

            ui.spinRandomOuts();
            
            chord = ui.getChord();
            chord_play = true;
            last_note_time == 0;
        }

        // if time for a new note
        uint note_length = ui.getNoteLengthMS();
        if(chord_play && (now - last_note_time >= note_length || last_note_time == 0)) {

            if(chords[chord][chord_note] == -1 || (arp_length >= 0 && arp_count >= arp_length)) {
                // next pulse after last note -> output pulse 2
                computer.setTimedPulse(2, TRIGGER_LENGTH);
                chord_play = false;
                continue;
            }
            else
            {
                // set next note time
                last_note_time = now;
                ui.update();

                float chord_root_volts = ui.getRootVolts();
                if(arp_count > chord_note) {
                    chord_root_volts += 1.0;
                }
                
                float chord_note_volts = chord_root_volts + ((float)chords[chord][chord_note] * VOLT_SEMITONE);
                computer.setCVOutVolts(1, chord_note_volts);
                computer.setCVOutVolts(2, chord_root_volts);

                computer.setTimedPulse(1, TRIGGER_LENGTH);
                if(ui.last_switch_change == 0 || now - ui.last_switch_change >= 3000) {
                    computer.setLEDs(1 << chord_note);
                }
                chord_note++;
                arp_count++;

                if(chords[chord][chord_note] == -1 && arp_count < arp_length) {
                    chord_note = 0;
                }
            }
        }
    }

    return 0;
}
