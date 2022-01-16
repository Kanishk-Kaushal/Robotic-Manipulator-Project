/*
  Using the BNO080 IMU

  Example : Euler Angles
  By: Paul Clark
  Date: April 28th, 2020

  Based on: Example1-RotationVector
  By: Nathan Seidle
  SparkFun Electronics
  Date: July 27th, 2018
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14686

  This example shows how to output the Euler angles: roll, pitch and yaw.
  The yaw (compass heading) is tilt-compensated, which is nice.
  https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/issues/5#issuecomment-306509440

  This example shows how to use the SPI interface on the BNO080. It's fairly involved
  and requires 7 comm wires (plus 2 power), soldering the PS1 jumper, and clearing
  the I2C jumper. We recommend using the Qwiic I2C interface, but if you need speed
  SPI is the way to go.

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

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

//These pins can be any GPIO
byte imuCSPin = 10;
byte imuWAKPin = 9;
byte imuINTPin = 8;
byte imuRSTPin = 7;
double T1;
double T2;
int IMU_1 = 0;
int IMU_2 = 0;
double Prev_E_1 = 0;
double Prev_E_2 = 0;
double Error_1 = 0;
double Error_2 = 0;
double dt = 0.1;
int PWM_1 = 0;
double PWM_2 = 0;
int flag = 0;

#define pi 3.14156

// INVERSE KINEMATICS | PARAMETERS -> DESIRED

void Inverse_Kinematics(float X_0, float y)
{
  if(X_0 < 0 && y > 0)
  {
    flag = 1;
  }
  
  // LINK LENGTHS

  float a1 = 5; // LINK LENTGH 1
  float a2 = 2; // LINK LENGTH 2

  // WORKSPACE PROBLEM

  if(((y*y)+(X_0*X_0)) > ((a1+a2)*(a1+a2)))
  {

        X_0 = (a1+a2)*(X_0)/(sqrt((X_0*X_0)+(y*y)));
        y = (a1+a2)*(y)/(sqrt((X_0*X_0)+(y*y)));
  }

  else
  {
         X_0 += 0;
         y += 0;
  }


  // INVERSE KINEMATICS

  double r = sqrt(pow(X_0, 2) + pow(y, 2));
 // Serial.println("R= ");
 // Serial.println(r);
  double phi1;
  double phi2;
  double phi3;

  // ELBOW UP FOR ACTUAL ARM

  if(X_0 < 0 )
  {
     phi1 = -acos((pow(a1, 2) + pow(r, 2) - pow(a2, 2)) / (2*a1*r));

                 phi2 = atan(y / (X_0 + 0.0000001)) + pi;
                 //phi2 = pi - atan(y / (X_0 + 0.0000001)) ;
                 phi3 = -acos((pow(a1, 2) + pow(a2, 2) - pow(r, 2)) / (2*a1*a2));
  }

  else
  {
    phi1 = acos((pow(a1, 2) + pow(r, 2) - pow(a2, 2)) / (2*a1*r));

                phi2 = atan(y / (X_0 + 0.0000001));
                phi3 = acos((pow(a1, 2) + pow(a2, 2) - pow(r, 2)) / (2*a1*a2));
  }
 // Serial.println("phi1 :");
 // Serial.print(phi1);
  // NOTE: LINE 252 & 259 -> X_0 + 0.0000001 -> 0.0000001 IS ADDED TO PREVENT ERROR IN CODE WHEN X_0 = 0

  // JOINT ANGLES

   T1 = (phi1 + phi2)*(180/pi);
   T2 = (phi3 - pi)*(180/pi);

  // Serial.println("T1 = ");
  // Serial.println(T1);

  // Serial.println("T2 = ");
  // Serial.println(T2);
   


}
/*
void PID(IMU_1, IMU_2)
{
 

  // IMU VALUES
  double Curr_pos_1 = IMU_1;
  double Curr_Pos_2 = abs((IMU_1 - IMU_2));

  // ERROR
  Error_1 = T1 - Curr_pos_1;
  Error_2 = T2 - Curr_Pos_2;

  Serial.println("Error_1 = %f ", Error_1);
  Serial.println("Current Pos = %f ", Curr_pos_1);
  Serial.println("T1 = %f", T1);

  Prev_E_1 += Error_1;
  Prev_E_2 += Error_2;

  // dE/dT
  double E_dot_1 = (Error_1 - Prev_E_1)/dt;
  double E_dot_2 = (Error_2 - Prev_E_2)/dt;

  // E.dT
  double E_int_1 = (Error_1)*dt + Prev_E_1;
  double E_int_2 = (Error_2)*dt + Prev_E_2;

  // PID CONSTANTS | SUBJECT TO CHANGE
  double Kp = 0.51;
  double Kd = 0.05;
  double Kc = 0.007;

  // OUPUT PWM SIGNALS
  PWM_1 =  ((Kp*Error_1) + (Kc*E_int_1) + (Kd*E_dot_1));
  PWM_2 =  (Kp*Error_2) + (Kc*E_int_2) + (Kd*E_dot_2);

  analogWrite(5, PWM_1);

  //Serial.println("PWM_1 = %f ", PWM_1);

  



}
*/


void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("BNO080 SPI Read Example"));

  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);

 // digitalWrite(1, LOW);

  myIMU.enableDebugging(Serial); //Pipe debug messages to Serial port

  if(myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin) == false)
  {
    Serial.println(F("BNO080 over SPI not detected. Are you sure you have all 6 connections? Freezing..."));
    while(1)
      ;
  }

  //You can also call begin with SPI clock speed and SPI port hardware
  //myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, 1000000);
  //myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, 1000000, SPI1);

  //The IMU is now connected over SPI
  //Please see the other examples for library functions that you can call

  myIMU.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form roll, pitch, yaw"));
}

void loop()
{
  delay(10); //You can do many other things. We spend most of our time printing and delaying.

  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees

    Serial.print("ROLL = ");
    Serial.println(roll);

    Serial.print("PITCH = ");
    Serial.println(pitch);

    Serial.print("YAW = ");
    Serial.println(yaw);
    
//    Serial.print(pitch, 1);
//    Serial.print(F(","));
//    Serial.print(yaw, 1);

    Serial.println();

    Inverse_Kinematics(5,1);
    //PID(pitch, 0);

    // IMU VALUES
  float Curr_pos_1 = roll;

   if( Curr_pos_1<10)
  {
    digitalWrite(2, LOW);
   // analogWrite(5, abs(PWM_1));

  Serial.println(digitalRead(1));
  }

   if(Curr_pos_1>165)
  {
    
    digitalWrite(2, HIGH);
  }

  
 
  IMU_2 = 0;
  
  int Curr_Pos_2 = abs((IMU_1 - IMU_2));

  // ERROR
  Error_1 = 60 - Curr_pos_1;
  Error_2 = T2 - Curr_Pos_2;

  if(Error_1 < 0.5)
  {
    digitalWrite(2, HIGH);
  }

   if(Error_1 > 0.5)
  {
    digitalWrite(2, LOW);
  }


 
  /*
  Serial.println("Error_1 = ");
  Serial.println(Error_1);
  Serial.println("Current Pos =  ");
  Serial.println(Curr_pos_1);
  Serial.println("T1 = " );
  Serial.println(T1);
  */
  
  Prev_E_1 += Error_1;
  Prev_E_2 += Error_2;

  // dE/dT
  double E_dot_1 = -(Error_1 - Prev_E_1)/dt;
  double E_dot_2 = (Error_2 - Prev_E_2)/dt;

  // E.dT
  double E_int_1 = (Error_1)*dt + Prev_E_1;
  double E_int_2 = (Error_2)*dt + Prev_E_2;

  // PID CONSTANTS | SUBJECT TO CHANGE
  double Kp = 1;
  double Kd = 0.0005;
  double Kc = 0.005;

  // OUPUT PWM SIGNALS
  PWM_1 =  ((Kp*Error_1) + (Kc*E_int_1) + (Kd*E_dot_1));
  PWM_2 =  (Kp*Error_2) + (Kc*E_int_2) + (Kd*E_dot_2);

  /*
  if(Error_1>-5 && Error_1<5)
  {
     if(abs(PWM_1) > 25)
  {
    PWM_1 = 25;
  }
  }
  
  */
  
  if(abs(PWM_1) > 25)
  {
    PWM_1 = 25;
  }
  
  
  
  analogWrite(5, abs(PWM_1));

  Serial.println("PWM_1 = ");
  Serial.println( abs(PWM_1));

    delay(10);
  }

}
