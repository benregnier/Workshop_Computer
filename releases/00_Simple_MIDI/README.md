## Music Thing Modular 
## Workshop System 

## Simple Midi Card with Calibration v0.5 

Written by Tom Whitwell
In Herne Hill, London, October 2024 

### How to Calibrate 
- Hold down the momentary toggle switch on startup 
- Release the switch 5 seconds after startup 
- The centre left LED will start to flash
- Patch the top oscillator in the Workshop System to an output so you can hear it 
- Make sure FM is fully off  
- Use a tuner to the oscillator to C3
- Patch the left CV output to the top oscillator 'Pitch' input 
- If the oscillator changes tune (it probably will, a little bit) you need to calibrate 0V
- Flip the toggle switch up
- The led will start to flash more slowly 
- Now you can use the big knob and the X knob as coarse/fine controls to set the position of 0v. Fiddle around until C3 is back in tune
- When you're ready, flip the toggle switch down. 
- Now tap the toggle switch, and the LED will move to the top left LED 
- This indicates it is sending +2V from the left hand output 
- Flip the switch up, and tweak the knobs until the oscillator gives a C5 in tune. 
- Push the switch back to the middle to save that setting, then tap down again to set -2V to C1
- Tap the switch again, and the LED will move to the right column, in the middle. 
- This represents the 0V output for the right hand CV output. 
- IMPORTANT: Repatch at this point, so the right CV output controls the Bottom oscillator. 
- (The oscillators aren't calibrated, so each output will be calibrated to a specific oscillator's quirks!) 
- You can tap through the voltages to check everything is working as it should, flipping up to change any voltages you're not happy with. 
- Once you're finished, press reset to return to normal MIDI mode. 

### Using the Eeprom calibration data in other cards 
- This card includes several useful classes 
- However, I (Tom) wrote it, so it's not very well organised, and can certainly be improved. 
- CV.h is Chris Johnson's delta/sigma code, which pushes 19bit precision from 11 bit PWM 
- DACChannel.h uses the calibration data to create  calibration constants. ChatGPT wrote most of this, so I don't really understand it, but it seems to work. 
- calibration.h organises the reading and writing of data to the Eeprom, with error checking. This was a Tom + ChatGPT collaboration, hopefully a good basis for incorporating the calibration code in other cards. 
    - The memory map for calibration makes space for 10x calibration points for both channels, but this only uses 3. 
    - There are certainly ways to automate the calibration process to make it more precise and easier to use. 
- Click.h is a little short tap/long tap library I wrote for Startup 

### PAGE 0 0x50 Memory Map for 2 x Precision PWM Voltage Outputs (Channels 0 and 1)

| Offset | Bytes | Contents                                                                                     |
|--------|-------|----------------------------------------------------------------------------------------------|
| 0      | 2     | Magic number = 2001 - if number is present, EEPROM has been initialized                      |
| 2      | 1     | Version number 0-255                                                                         |
| 3      | 1     | Padding                                                                                      |
| 4      | 1     | Channel 0 - Number of entries 0-9                                                            |
| 5      | 40    | 10 x 4 byte blocks: 1 x 4 bit voltage + 4 bits space \| 1 x 24 bit setting = 32 bits = 4 bytes |
| 45     | 1     | Channel 1 - Number of entries 0-9                                                            |
| 46     | 40    | 10 x 4 byte blocks: 1 x 4 bit voltage + 4 bits space \| 1 x 24 bit setting = 32 bits = 4 bytes |
| 86     | 2     | CRC Check over previous data                                                                 |
| 88     |       | END                                                                                          |



### Includes 
Responsive Analog Read by Damien Clarke
https://github.com/dxinteractive/ResponsiveAnalogRead
Adafruit Tiny USB Library 
https://github.com/adafruit/Adafruit_TinyUSB_Arduino 
Earle Pilhower's arduino-pico 
https://github.com/earlephilhower/arduino-pico  

### Compilation options: 
Board: Generic RP2040 
Flash Size: 16MB (No FS) 
CPU Speed: 133MHz 
Boot Stage 2: W25Q16JV QSPI/4 (for the newer 2mb cards) 
USB Stack: Adafruit TinyUSB 

### Installation 

- To install the UF2: Push the boot select button (remove the main knob on Computer), it's on the right 
- Connect the front USB to the computer 
- Hold down boot and tap reset (the button at the bottom by the card slot 
- A folder should appear on your desktop called RPI RP2 
- Drag simplemidi.uf2 into that folder 
- The system should reboot and the folder disappear 
- Now check your DAW for midi interfaces - you should find one called Music Thing Workshop System MIDI or something similar to that 