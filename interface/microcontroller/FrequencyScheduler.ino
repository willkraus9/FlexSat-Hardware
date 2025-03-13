#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(19, 0x29); 

//NB: adding z accel data causes skipping of samples!
float accel1_x;
float accel1_y;
float accel1_z;

float gyro1_x;
float gyro1_y;
float gyro1_z;

float accel2_x;
float accel2_y;
float accel2_z;

float gyro2_x;
float gyro2_y;
float gyro2_z;

const int DATA_SIZE = 300;
struct SensorData{
  float accel1_x;
  float accel1_y; 
  float accel1_z;

  float gyro1_x;
  float gyro1_y;
  float gyro1_z;

  float accel2_x;
  float accel2_y;
  float accel2_z;

  float gyro2_x;
  float gyro2_y;
  float gyro2_z;
  unsigned long time;
};

SensorData sensor_data[DATA_SIZE];

int n;
// bool finish;
unsigned long ttimee;

void setup()
{
  Serial.begin(115200);
  n = 0;
  // finish = 0;
  ttimee = 0;
  Serial.println("Orientation Sensor Test");
  Serial.println("");
  Wire.begin();     // Initialize primary I2C bus

if(!bno1.begin()) {
  Serial.println("Ooops, no BNO055 detected on primary I2C bus ... Check your wiring or I2C ADDR!");
  while(1);
} else {
  Serial.println("BNO055 on primary I2C bus initialized successfully!");
}
  bno1.setExtCrystalUse(true);

  if(!bno2.begin()) {
  Serial.println("Ooops, no BNO055 detected on secondary I2C bus ... Check your wiring or I2C ADDR!");
  while(1);
} else {
  Serial.println("BNO055 on secondary I2C bus initialized successfully!");
}

  bno2.setExtCrystalUse(true);

  delay(1000);
}

void loop(){
  unsigned long currentTime = micros(); //only read micros() once
  if (currentTime - ttimee >= 20000){ // was 10,000 microsecond sampling for 100 Hz; reduce to 50 Hz
    sensors_event_t event1;
    bno1.getEvent(&event1);
    imu::Vector<3> accel1 = bno1.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro1 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    sensors_event_t event2;
    bno2.getEvent(&event2);
    imu::Vector<3> accel2 = bno2.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro2 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    accel1_x = accel1.x();
    accel1_y = accel1.y(); 
    accel1_z = accel1.z();

    gyro1_x = gyro1.x();
    gyro1_y = gyro1.y();
    gyro1_z = gyro1.z();

    accel2_x = accel2.x();
    accel2_y = accel2.y();
    accel2_z = accel2.z();

    gyro2_x = gyro2.x();
    gyro2_y = gyro2.y();
    gyro2_z = gyro2.z();

    sensor_data[n].time = currentTime / 1000;
    sensor_data[n].accel1_x = accel1_x;
    sensor_data[n].accel1_y = accel1_y;
    sensor_data[n].accel1_z = accel1_z;

    sensor_data[n].gyro1_x = gyro1_x;
    sensor_data[n].gyro1_y = gyro1_y;
    sensor_data[n].gyro1_z = gyro1_z;

    sensor_data[n].accel2_x = accel2_x;
    sensor_data[n].accel2_y = accel2_y;
    sensor_data[n].accel2_z = accel2_z;

    sensor_data[n].gyro2_x = gyro2_x;
    sensor_data[n].gyro2_y = gyro2_y;
    sensor_data[n].gyro2_z = gyro2_z;

    // print data
    Serial.print(sensor_data[n].time);
    Serial.print(", ");
    Serial.print(sensor_data[n].accel1_x);
    Serial.print(", ");
    Serial.print(sensor_data[n].accel1_y);
    Serial.print(", ");
    Serial.print(sensor_data[n].accel1_z);
    Serial.print(", ");

    Serial.print(sensor_data[n].gyro1_x);
    Serial.print(", ");
    Serial.print(sensor_data[n].gyro1_y);
    Serial.print(", ");
    Serial.print(sensor_data[n].gyro1_z);
    Serial.print(", ");

    Serial.print(sensor_data[n].accel2_x);
    Serial.print(", ");
    Serial.print(sensor_data[n].accel2_y);
    Serial.print(", ");
    Serial.print(sensor_data[n].accel2_z);
    Serial.print(", ");

    Serial.print(sensor_data[n].gyro2_x);
    Serial.print(", ");
    Serial.print(sensor_data[n].gyro2_y);
    Serial.print(", ");
    Serial.print(sensor_data[n].gyro2_z);
    Serial.println();
    
    n = (n+1) % DATA_SIZE;
    ttimee = currentTime;
  }
}