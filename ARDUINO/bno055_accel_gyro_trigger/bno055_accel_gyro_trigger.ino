#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (5)
const int triggerPin = 25;
int counter = 0;
int frametype = 1;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

void setup(void)
{
  Serial.begin(115200);
  //Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  pinMode(triggerPin, OUTPUT);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(5000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  //Serial.print("Current Temperature: ");
  //Serial.print(temp);
  //Serial.println(" C");
  //Serial.println("");

  bno.setExtCrystalUse(true);

  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  //imu::Quaternion quat = bno.getQuat();

  counter++;
  if (counter == 10){
    frametype = 2;
    digitalWrite(triggerPin,HIGH);
    digitalWrite(triggerPin,LOW);
    counter = 0;
  } else {
      frametype = 1;
    }
  sensors_event_t angVelocityData, accelerometerData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print(frametype);
  Serial.print(","); 
  Serial.print(angVelocityData.gyro.x);
  Serial.print(",");
  Serial.print(angVelocityData.gyro.y);
  Serial.print(",");
  Serial.print(angVelocityData.gyro.z);
  Serial.print(",");
  Serial.print(accelerometerData.acceleration.x);
  Serial.print(",");
  Serial.print(accelerometerData.acceleration.y);
  Serial.print(",");
  Serial.println(accelerometerData.acceleration.z);
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
