# FlexSat-Hardware

![Happy FlexSat](images/happy-flexsat.gif)

## Hardware Setup:

* Obtain a battery (Turnigy 4S 14.8V 5000mAh, XT90 connector) for each motor

  * There should be another motor assembly in the lab for experiments with the two motor setup.
* Attach assembly to the mocap cage

  * Hold the aluminum plate perpendicular to a column in the mocap cage
  * Loosely fix the pipe clamps around the pipes
  * Tighten the pipe clamps with a Philips Head screwdriver
  * Make sure the final assembly is level, the final fit should be very snug

## Systems Overview:


### Motor Assembly

![Motor Assembly](images/motor-assembly.png)

    

* adjusting IMU to center line
* taking motors apart

### Wiring Assembly 

![Happy FlexSat](images/wiring-assembly.png)

* parts to make sure plugged in

## Initializing an Experiment:

* plug motor into power
* plug in computer to ODrive motor controller

  * https://gui.odriverobotics.com/
  * More details for debugging: https://docs.odriverobotics.com/v/latest/interfaces/gui.html
* test to make sure the motor actually works (calibrating for integrated encoder)
* should be calibrated already, but test velocity (TORQUE?) control mode
* remove USB from motor controller
* plug in computer to both microcontroller

  * check if pull data from IMUs
  * file name?
* analyze data to make sure actually getting stuff

## Running an Experiment:

* plug into where
  * computer to arduino1
  * computer to arduino2
* data collection
* run arduino file
* run matlab file
  * delay
* run experiment
  * data shows up in MATLAB struct
  * copy data / labelling scheme

## Debugging:

* something with the motor step
  * motor calibration isn't good
* something at the arduino step
* something at the MATALB step


communication back and forth was hard

regex for messags

one-way comms for arduino and matlab
