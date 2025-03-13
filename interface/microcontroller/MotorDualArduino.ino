// CAN ===========================================================================================================
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

// CAN ===========================================================================================================
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
// TIMING ===========================================================================================================
// RW Setup for chirp signal
const int f0 = 0.1;
const int f1 = 50.0;
const float T = 180.0; //duration of signal (180 seconds)
const float Fs = 50.0; // frequency for data colleciton 
const unsigned long samplePeriod = 1000000 / Fs;  // Sample period (microseconds)

unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
  // CAN SETUP 
  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }

  // request bus voltage and current (1sec timeout)
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1)) {
    Serial.println("vbus request failed!");
    while (true); // spin indefinitely
  }

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
}
void loop() {
  /* 
  Conditional statements: 
  \--> Timing (has to be # Hz)
      \--> Receive CAN feedback
          \--> Receive motor command from MATLAB
  */
  // Setup parsing motor values from MATLAB
  unsigned long currentTime = micros(); //only read micros() once  
  static String inputString = ""; // A string to hold incoming data
  static bool stringComplete = false; // Whether the string is complete
  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages

  if (odrv0_user_data.received_feedback) {
    Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
    odrv0_user_data.received_feedback = false;
    // CAN FEEDBACK
    char inChar = (char)Serial.read(); // Get the new byte
    inputString += inChar; // Add it to the inputString
    // If the incoming character is '>', set stringComplete to true
    // RECEIVE MATLAB COMMAND
    if (inChar == '>') {
      stringComplete = true;
      // Process the motor command string
      if (stringComplete) {
        int startIndex = inputString.indexOf('<');
        int endIndex = inputString.indexOf('>');
        // Ensure the string is valid
        if (startIndex >= 0 && endIndex > startIndex) {
          String command = inputString.substring(startIndex + 1, endIndex);
          float motorValue = command.toFloat(); // Convert the command
          // odrv0.setTorque(motorValue);
          odrv0.setVelocity(motorValue);
          // Clear the inputString and reset stringComplete
          inputString = "";
          stringComplete = false;
        }
      }
      // sanity check: does the motor work with an alternate CAN configuration
    n = (n+1) % DATA_SIZE;
    ttimee = currentTime;
    lastSampleTime = currentTime;
    }
  }
}
