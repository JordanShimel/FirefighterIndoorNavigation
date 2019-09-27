# FireFighterRemoteUnit

Code for remote unit for Fire Fighter senior project
1 - Overview
This module will be responsible for capturing stereo video from the Intel RealSense D435i camera, converting it into a ROSBag, and transmitting it over Wi-Fi to the base unit.  It will run on an UPBoard using Ubuntu 16.04, with an attached Wi-Fi module.  This unit will have a point-to-point Wi-Fi connection to the base unit.

2 - Requirements
-Ubuntu 16.04
-Intel RealSense SDK

3 - Roadmap
0.1 - Initial video capture tests, no conversion or transmission
0.2 - Transmission of unconverted video
0.3 - Conversion and transmission of video as ROSBag
0.4 - UI for configuration via UPBoard
0.5 - Remote configuration from base unit
