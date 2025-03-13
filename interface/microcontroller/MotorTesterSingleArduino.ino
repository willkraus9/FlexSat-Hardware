#include <Arduino.h>
#include "ODriveCAN.h"
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
// Documentation for this example can be found here:
// https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html
// See https://github.com/arduino/ArduinoCore-API/blob/master/api/HardwareCAN.h
// and https://github.com/arduino/ArduinoCore-renesas/tree/main/libraries/Arduino_CAN

// TIMING ===========================================================================================================
int n = 0;
unsigned long ttimee = 0;
int motorValue = 0;
const int DATA_SIZE = 300; // NB: unsure if need this, from when IMU used 

// RW Setup for pseudorandom signal
float rampValue; // value sent to motor during ramp 
float rampTime; // time start ramping value

float randomValue; // value sent to motor during random actuation 
float randomTime; // time start random actuation

float velValue; // actual velocity command for motor

// RW Setup for chirp signal
const int f0 = 0.1; // minimum frequency
const int f1 = 10.0; // maximum frequency, changed to focus on lower frequencies
const float T = 180.0; //duration of signal (180 seconds)
const float Fs = 50.0; // frequency for data collection
const unsigned long samplePeriod = 1000000 / Fs;  // Sample period (microseconds)

unsigned long lastSampleTime = 0;

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000
// ODrive node_id for odrv0
#define ODRV0_NODE_ID 0
#define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
HardwareCAN& can_intf = CAN;
bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

void setup() {
  Serial.begin(115200);
  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  Serial.println("Starting ODriveCAN demo");

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Serial.println("Waiting for ODrive...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }

  Serial.println("found ODrive");

  // request bus voltage and current (1sec timeout)
  Serial.println("attempting to read bus voltage and current");
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1)) {
    Serial.println("vbus request failed!");
    while (true); // spin indefinitely
  }

  Serial.print("DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  Serial.println("Enabling closed loop control...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }
  Serial.println("ODrive running!");
}
void loop() {
  unsigned long currentTime = micros(); //only read micros() once  
  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
  rampValue = (currentTime / 1000000) - rampTime;
    // should go from 0 to middle of randomValue's range (manually set)
    // currentTime in seconds - rampTime (0 at 10, 1 at 11, 2 at 12, etc.; do until reach end)
  rampTime = 30;
  randomValue = random(10,21); // hard limit is (-25, 25) rotations per second; 
    // hardware bug: anything above 15 rad/s may cause the IMUs to stop sending data
    // NB: random() excludes the second value for the range!
  randomTime = rampTime + 16; // ensures in between random values 
  if (currentTime / 1000000 < rampTime) {// (30 seconds aka 30,000,000 microseconds) (this is for getting a standard bias term)
    velValue = 0;
    } else if (currentTime / 1000000 > rampTime && currentTime / 1000000 < randomTime){
    velValue = 0; // no ramp value since torque control!
    } else if (currentTime / 1000000 > randomTime){
      float t = (currentTime / 1000000.0);  // Convert time to seconds
      float k = (f1 - f0) / T;
      float chirpFreq = f0 + k * t;
      float chirpSignal = 0.5 * (sin(2 * PI * chirpFreq * t));
      velValue = chirpSignal*1.5; // should be -0.1 to 0.1 --> chirpSignal*2
    }
    odrv0.setTorque(velValue);
    Serial.println(velValue);
}
// Troubleshooting: 
// The IMUs aren't connecting: unplug the Arduino, double tap the RESET button, and run the code again. Check imu_bus_tester to see if the IMUs are being read.
// This script runs and nothing gets printed to the command line: this is an issue with the Arduino. Double tap the RESET button, power cycle the Arduino, and run the script again. 