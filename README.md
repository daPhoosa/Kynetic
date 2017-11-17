# KYNETIC
Motion control software for 3D printing with the Teensy 3.5 - EXPERIMENTAL

## Features In development
* Smooth jerk free accelerations using a custom trajectory planner
* Multi-block look ahead for smooth continuous motion
* Segment junction smoothing to mitigate surface ringing
* Extruder look ahead for high speed printing

## SOFTWARE

Arduino IDE 1.8.5       // https://www.arduino.cc/en/Main/Software

Teensyduino             // https://www.pjrc.com/teensy/teensyduino.html

###Required Libraries:

<FrequencyTimer2.h>     // https://github.com/PaulStoffregen/FrequencyTimer2     --included with teensyduino

<SmoothMove.h>          // https://github.com/daPhoosa/SmoothMove                --instal to libraries diectory

<uStepper.h>            // https://github.com/daPhoosa/uStepper                  --instal to libraries diectory

<SdFat.h>               // https://github.com/greiman/SdFat                      --instal to libraries diectory

<PollTimer.h>           // https://github.com/daPhoosa/PollTimer                 --instal to libraries diectory

<uButton.h>             // https://github.com/daPhoosa/uButton                   --instal to libraries diectory

