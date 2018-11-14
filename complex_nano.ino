//Gulyás Péter; NK-HAFSIW
//DeepPilot Project
//BNO055 ASSN calibration then orientation, encoder, servo read
//Created: 2018-November-14

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/* Connections
		Connect SCL/LRX - pin-12 to analog 5
		Connect SDA/ATX - pin-11 to analog 4
		Connect VCC - pin-20 to 3-5V DC
		Connect GND - pin-19 to common ground
		D2 - Encoder A
		D8 - Encoder B
*/

// Set the delay between fresh samples --> adjusting datarate
//#define BNO055_SAMPLERATE_DELAY_MS (45)   // ~20Hz datarate
//#define BNO055_SAMPLERATE_DELAY_MS (95)   // ~10Hz datarate
//#define BNO055_SAMPLERATE_DELAY_MS (195)   // ~5Hz datarate
#define BNO055_SAMPLERATE_DELAY_MS (13)   // ~40Hz datarate

#define SrvFb A0  
#define ENC_A 2
#define ENC_B 8

//Define variables
float QX = 0, QY = 0, QZ = 0, QW = 0;
float GX = 0, GY = 0, GZ = 0;
float LX = 0, LY = 0, LZ = 0;
String mystring;

struct MotFb
{
  double velo;
  double dist;
  int rpm;
};
typedef struct MotFb Motor_FB;

int AngFbInt;               //-------------------------------------                    
String VeloFeedback;                    //FEEDBACK//
String RpmFeedback;
String DistFeedback;
//String str;
String buf1;
String buf2;
String buf3;            //-------------------------------------

const int interrupt0 = 0; // (PIN 2)  //ENCODER READING//

long sum_pulse_num = 0;
int pulse_num;                    //Calculations//
int rpm = 0;

// FUNCTION LIST -------------------------------------------START------

void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
void Enc_Calc();
void Pulse_counter();
int ServFb();

// FUNCTION LIST --------------------------------------------END-------

// Identify sensor with special ID
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Setup function to check calibration.--------------------START-------
// If needed: doing calibration, and saving to EEPROM -----------------

void setup(void)
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("BNO055 Orientation sensor "); Serial.println("");

  // Sensor initialization
  if (!bno.begin())
  {
    // Error message
    Serial.print("BNO055 not detected ... Check your I2C ADDR and wiring!");
    while (1);
  }

  displaySensorDetails();
  delay(1500);

  int eeAddress = 0;
  long bnoID;
  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  //  Look for unique sensor ID in EEPROM.
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
    Serial.println("Calibration data not found in EEPROM. Calibration nedded!");
    delay(500);

    bno.setExtCrystalUse(false);
    sensors_event_t event;
    bno.getEvent(&event);
    Serial.println("Calibrating Sensor: ");
    while (!bno.isFullyCalibrated())
    {
      displayCalStatus();
      Serial.println("");
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }

    Serial.println("Calibration completed!");
    Serial.println("-------------/-------");
    Serial.println("------------/--------");
    Serial.println("-----------/---------");
    Serial.println("----------/----------");
    Serial.println("----'\'----/-----------");
    Serial.println("-----'\'--/------------");
    Serial.println("------'\'/-------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCal;
    bno.getSensorOffsets(newCal);
    displaySensorOffsets(newCal);
    Serial.println("---------------------");
    Serial.println("Storing calibration data to EEPROM.");
    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;
    EEPROM.put(eeAddress, bnoID);
    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCal);
    Serial.println("Calibration array stored to EEPROM.");
    delay(500);

  }
  else
  {
    //Serial.println("Calibration data found in EEPROM. Using it as Offset array.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    //displaySensorOffsets(calibrationData);
    bno.setSensorOffsets(calibrationData);

  }

  //displaySensorStatus();

  /*Serial.println("Orientation data transfer starts in 3 sec");
  delay(1000);
  Serial.println("Orientation data transfer starts in 2 sec");
  delay(1000);*/
  //Serial.println("Orientation data transfer starts in 1 sec");
  delay(1000);

}

// Setup function to check calibration.--------------------END---------

// Serial orientation data transfer to host --------------START--------
void loop()
{
  // Get new data reading from BNO055 chip
  sensors_event_t event;
  bno.getEvent(&event);

  //Serial data transfer - Start -
  // Timestamp
  //Serial.print(millis());
  //Serial.print(",");
  // Display Heading-Roll-Pitch as orientation
  /*Serial.print(event.orientation.x, 2);
  Serial.print(",");
  Serial.print(event.orientation.y, 2);
  Serial.print(",");
  Serial.print(event.orientation.z, 2);*/
  
  /* Display Quaternion data */
  imu::Quaternion quat = bno.getQuat();
  QX = (quat.x());
  QY = (quat.y());
  QZ = (quat.z());
  QW = (quat.w());
  mystring = String(QX, 6);
  Serial.print(mystring);
  Serial.print(" ");
  mystring = String(QY, 6);
  Serial.print(mystring);
  Serial.print(" ");
  mystring = String(QZ, 6);
  Serial.print(mystring);
  Serial.print(" ");
  mystring = String(QW, 6);
  Serial.print(mystring);
  Serial.print(" ");
  
  /* Display the Gyroscope data */
  imu::Vector<3> eulergyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  GX = eulergyro.x();
  GY = eulergyro.y();
  GZ = eulergyro.z();
  mystring = String(GX, 3);
  Serial.print(mystring);
  Serial.print(" ");
  mystring = String(GY, 3);
  Serial.print(mystring);
  Serial.print(" ");
  mystring = String(GZ, 3);
  Serial.print(mystring);
  Serial.print(" ");
  
  /* Display the Linearaccelerometer data */
  imu::Vector<3> eulerlin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); 
  LX = eulerlin.x();
  LY = eulerlin.y();
  LZ = eulerlin.z();
  mystring = String(LX, 3);
  Serial.print(mystring);
  Serial.print(" ");
  mystring = String(LY, 3);
  Serial.print(mystring);
  Serial.print(" ");
  mystring = String(LZ, 3);
  Serial.print(mystring);
  Serial.print(" ");

  /* Display the Encoder and servo data */
  Enc_Calc();
  AngFbInt = ServFb();
  Serial.print(AngFbInt);
  Serial.print(" ");
  Serial.print(buf1);
  Serial.print(" ");
  Serial.print(buf2);
  Serial.print(" ");
  Serial.print(buf3);
  
  // New line for next sample
  Serial.println("");
  // Serial data transfer - End -

  // Wait for specified time.
  delay(BNO055_SAMPLERATE_DELAY_MS);

}
// Serial orientation data transfer to host --------------END----------

// Functions used by the program ------------------------START---------

// Display basic information about the sensor -------------------------
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

// Display sensor status ----------------------------------------------
void displaySensorStatus(void)
{
  // Get the system status values
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  Serial.println("------------------------------------");
  Serial.print("Sys Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("Sys Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("------------------------------------");
  delay(500);
}

// Display BNO055 calib status ----------------------------------------
void displayCalStatus(void)
{
  // Get calibration status (0 to 3) 0:Not calibrated; 3:Fully caibrated
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // The data ignored until the system calibration is > 0
  Serial.print("\t");
  if (!system)
  {
    Serial.print("!");
  }
  //Display the individual values
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" Gyro:");
  Serial.print(gyro, DEC);
  Serial.print(" Acc:");
  Serial.print(accel, DEC);
  Serial.print(" Mag:");
  Serial.print(mag, DEC);
}

// Display the raw calibration offset and radius data -----------------
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
}

// Calculating dinamic values from encoder reading -----------------
void Enc_Calc(){
  Motor_FB FB_1;                // 48 Encoder pulse = 1 round
  rpm = pulse_num * 60 / 24;    // RPM calculating ratio for 1 ch
  //Serial.print(rpm);
  //Serial.print("\t");
  FB_1.rpm = rpm;
  FB_1.velo = rpm / 9.5492965964254;
  //Serial.print("Speed = ");
  //Serial.print(FB_1.velo, 4);
  //Serial.print(" rad/s");
  //Serial.print("\t"); 
  sum_pulse_num = sum_pulse_num + pulse_num;
  FB_1.dist = sum_pulse_num / 1185.36 * 0.282743;
  //Serial.print("Distance: ");
  //Serial.print(FB_1.dist, 4);
  //Serial.println(" m");                           
  //Serial.print("\t");
  VeloFeedback = String(FB_1.velo);
  RpmFeedback = String(FB_1.rpm);
  DistFeedback = String(FB_1.dist);
  buf1 = VeloFeedback;
  buf2 = RpmFeedback;
  buf3 = DistFeedback;
  pulse_num = 0x00;
}

// Counting encoder pulses - Only falling edge is considerd -----------------
void Pulse_counter()
{
 if(digitalRead(ENC_B) == HIGH) pulse_num++;          
 else pulse_num--;
}

// Servo feedback and mapping, from analog readig -----------------
int ServFb(){
  int val = analogRead(SrvFb);              //Analog value reading to know the position
  //Serial.print(val);
  int valMap = map(val, 132, 413, -10, 10); //Remap the analog interval to get the angle of the car 
  return valMap;
}

// Functions used by the program ------------------END-----------------
