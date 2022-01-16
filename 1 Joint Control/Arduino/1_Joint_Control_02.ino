// PAYLOAD UPDATE

/*  

  DC MOTOR ANGLE CONTROL USING IMU
 
  Hardware modifications:
  The PS1 jumper must be closed.
  The PS0 jumper must be open. PS0/WAKE is connected and the WAK pin is used to bring the IC out of sleep.
  The I2C pull up jumper must be cleared/open.
  The ADR jumper must be cleared/open. 

  Hardware Connections:
  Don't hook the BNO080 to a normal 5V Uno! Either use the Qwiic system or use a
  microcontroller that runs at 3.3V.
  Arduino 13 = BNO080 SCK
  12 = SO
  11 = SI
  10 = !CS
  9 = WAK
  8 = !INT
  7 = !RST
  3.3V = 3V3
  GND = GND
  
*/

#include <SPI.h>

#include "SparkFun_BNO080_Arduino_Library.h" 
BNO080 myIMU;

// THESE PINS CAN BE ANY GPIO
byte imuCSPin = 10;
byte imuWAKPin = 9;
byte imuINTPin = 8;
byte imuRSTPin = 7;

// JOINT PIN
# define joint 5

// DIRECTION PIN
#define dir_pin 2


// DEFINE TARGET ANGLE  
float target = 125;

// DEFINE PID CONSTANTS
float kp = 10;
float kd = 0;
float ki = 0;

float error = 0;
float prev_error = 0;
float e_int = 0;
float e_dot = 0;

// SAMPLING TIME
float dt = 0.001;

// CONTROL SIGNAL
float cs = 0;

float pwm = 0;

int dir;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("BNO080 SPI Read Example"));

  pinMode(dir_pin, OUTPUT);
  
  // PIPE DEGUB MESSAGES TO SERIAL PORT
  myIMU.enableDebugging(Serial); 

  if(myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin) == false)
  {
    Serial.println(F("BNO080 over SPI not detected. Are you sure you have all 6 connections? Freezing..."));
    while(1);
  }

  // IMU CONNECTED OVER SPI
  
  // UPDATE DATA EVERY 50 ms
  myIMU.enableRotationVector(50); 

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form roll, pitch, yaw"));

}

void loop()
{
  delay(10);
 
  // LOOK FOR REPORTS FROM IMU
  if (myIMU.dataAvailable() == true)
  {
    // CONVERT EULER ANGLES TO DEGREES
    float roll = (myIMU.getRoll()) * 180.0 / PI;   
    float pitch = (myIMU.getPitch()) * 180.0 / PI; 
    float yaw = (myIMU.getYaw()) * 180.0 / PI;     

    Serial.print(roll, 1);
    Serial.print(F(","));
    Serial.print(pitch, 1);
    Serial.print(F(","));
    Serial.print(yaw, 1);

    // DEFINE ERROR
    error = target - roll; 

    // INTEGRAL ERROR
    e_int = e_int + (error * dt);

    // DERIAVTE ERROR
    e_dot = (error - prev_error) / dt;

    // CONTROL SIGNAL -> PWM
    pwm = abs(kp*error + ki*e_int + kd*e_dot);
    
    // CAP PWM
    if(pwm > 150)
    {
      pwm = 150;
    }
      
    // HOLD PAYLOAD
     if(abs(error)<3)
    {
      pwm = 0;
      analogWrite(joint, abs(pwm));
    }
    
    // GIVE MOTOR REQUIRED PWM
    analogWrite(joint, abs(pwm));
    
    // REVERSE DIRECTION OF MOTOR WHEN CURRENT POSITION > DESIRED POSITION
    if(error > 0)
    {
      dir = 1;
      digitalWrite(dir_pin, dir);
    }

    if(error < 0)
    {
      dir = 0;
      digitalWrite(dir_pin, dir);
    }

    Serial.println();
    
    Serial.print("DIRECTION PIN = ");
    Serial.print(digitalRead(dir_pin));

    Serial.println();
    
    Serial.print("PWM = ");
    Serial.print(pwm);
    
    Serial.println();
  }

}
