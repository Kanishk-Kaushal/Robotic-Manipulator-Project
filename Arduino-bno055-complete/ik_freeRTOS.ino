// Header files start
#include <Arduino_FreeRTOS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
// Header files end

// Definitions start
#define BNO055_SAMPLE_DELAY  100
#define pi 3.141527
// Definitions end

// Pins configuration start
#define dirPin1 4
#define pwmPin1 5
#define dirPin2 6
#define pwmPin2 7
// Pins configuration end

// Sensor initialisation start
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x29);
// Sensor initialisation end

// Function prototype start
void pin_setup(void);
void setBNO0551(void);
void setBNO0552(void);
static void jnt1FK(void* pvParameters);
static void jnt2FK(void* pvParameters);
// Function prototype end

// Setup function start
void setup()
{
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.println("serial communication initiated");
  // Initialise BNO055
  setBNO0551();
  setBNO0552();
  // GPIO setup
  pin_setup();
  // xTaskCreate(MyTask_pointer, "task_name", 100, Parameter, Priority, TaskHandle);
  xTaskCreate(jnt1FK, "FKjnt1", 1000, NULL, 3, NULL);
  xTaskCreate(jnt2FK, "FKjnt2", 1000, NULL, 3, NULL);
}
// Setup function end

// Loop function start
void loop()
{
  // void loop() is supposed to be empty as each task executes on interrupt after specified time.
}
// Loop function end

// Function definition start
void pin_setup(void)
{
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  Serial.println("pin setup complete");
}
// Tasks definition start
static void jnt1FK( void* pvParameters)
{
  // Task jnt1FK's local variables start
  int iter;
  // Number of waypoints
  const int numWP = 1;
  // Link lengths
  double l1 = 44.5;
  double l2 = 38.5;
  // Desired end-effector position
  double eefX[numWP]={l2};
  double eefY[numWP]={l1};
  double temp, tar_jnt_ang1, tar_jnt_ang2; // target joint angles
  double err1, err2; // errors|setpoint-currentval
  double int_err1, int_err2; // integral errors
  double der_err1, der_err2; // derivatal error
  double prev_err1, prev_err2 = 0; // previous errors
  double dt = 0.0002; // sampling time
  double kp = 3; // pid constant of proportionality
  double ki = 1; // pid constant of integration
  double kd = 0; // pid constant of derivation
  double pwm_cv1; // pwm control variable
  double curr_jnt_ang1, curr_jnt_ang2; // current system state
  double Final_pos = 0;
  double delta = 0;
  // Task jnt1FK's local variables end
  /*
  // Equally Spaced Vector Formation
  Final_pos = sqrt(sq(l1+l2) + sq(l1-l2));
  delta = (Final_pos)/numWP;
  for(int n=0; n<numWP; n++)
  {
    eefX[n]= n*delta;
    eefY[n] = abs(l1-l2);
  }
  */
  
  iter = 0;
  while (1) // tasks should never return or exit
  {
    // Inverse kinematics algorithm
    temp = acos((((eefX[iter]) * (eefX[iter]) + (eefY[iter]) * (eefY[iter]) - (l1 * l1 + l2 * l2))) / (2 * l1 * l2));
    tar_jnt_ang1 = (atan((l2 * sin(temp)) / (l2 * cos(temp) + l1)) + atan(eefY[iter] / eefX[iter]));
    tar_jnt_ang2 = temp;
    curr_jnt_ang1 = radians(yBNO0551()); // current joint angle 1
    curr_jnt_ang2 = curr_jnt_ang1 - radians(yBNO0552()); // current joint angle 2
    err1 = degrees(tar_jnt_ang1) - degrees(curr_jnt_ang1); // error in joint angle 1
    err2 = degrees(tar_jnt_ang2) - degrees(curr_jnt_ang2); // error in joint angle 2
    int_err1 = int_err1 + (err1 * dt); // integral error in joint angle 1
    der_err1 = (err1 - prev_err1) / dt; // derivatal error in joint angle 1
    pwm_cv1 = abs(kp * err1 + ki * int_err1 + kd * der_err1); // control varial for pwm in joint 1
    if (pwm_cv1 > 90) // capping pwm for a safer control at joint 1
      pwm_cv1 = 90;
    if (err1 > 0) // reversing polarity of motor once error reaches its minimum and oscillates after at joint 1
      digitalWrite(dirPin1, LOW);
    else
      digitalWrite(dirPin1, HIGH);
     
    if(err1 < 1 && err2 < 1)
    {
      iter++;
      delay(200);
      if(iter >= numWP)
        iter = (numWP - 1);
      
    }
     analogWrite(pwmPin1, pwm_cv1); // actuating link 1
  }
}
static void jnt2FK(void* pvParameters)
{
  int iter;
  // Number of waypoints
  const int numWP = 1;
  // Link lengths
  double l1 = 44.5;
  double l2 = 38.5;
  // Desired end-effector position
  double eefX[numWP] = {l2};
  double eefY[numWP] = {l1};
  double temp, tar_jnt_ang1, tar_jnt_ang2;
  double err1, err2;
  double int_err1, int_err2;
  double der_err1, der_err2;
  double prev_err1, prev_err2;
  double dt = 0.0002;
  double kp = 3;
  double ki = 1;
  double kd = 0;
  double curr_jnt_ang1, curr_jnt_ang2;
  double pwm_cv2;
  double Final_pos = 0;
  double delta = 0;
  /*
  // Equally Spaced Vector Formation
  Final_pos = sqrt(sq(l1+l2) + sq(l1-l2));
  delta = (Final_pos)/numWP;
  for(int n=0; n<numWP; n++)
  {
    eefX[n]= n*delta;
    eefY[n] = abs(l1-l2);
  }
  */
  iter = 0;
  while (1) // tasks should never return or exit
  {
    // Inverse kinematics algorithm
    temp = acos((((eefX[iter]) * (eefX[iter]) + (eefY[iter]) * (eefY[iter]) - (l1 * l1 + l2 * l2))) / (2 * l1 * l2));
    tar_jnt_ang1 = (atan((l2 * sin(temp)) / (l2 * cos(temp) + l1)) + atan(eefY[iter] / eefX[iter]));
    tar_jnt_ang2 = temp;
    curr_jnt_ang1 = radians(yBNO0551()); // current joint angle 1
    curr_jnt_ang2 = curr_jnt_ang1 - radians(yBNO0552()); // current joint angle 2
    err1 = degrees(tar_jnt_ang1) - degrees(curr_jnt_ang1); // error in joint angle 1
    err2 = degrees(tar_jnt_ang2) - degrees(curr_jnt_ang2); // error in joint angle 2
    int_err2 = int_err2 + (err2 * dt); // integral error in joint angle 2
    der_err2 = (err2 - prev_err2) / dt; // derivatal error in joint angle 2
    pwm_cv2 = abs(kp * err2 + ki * int_err2 + kd * der_err2); // control varial for pwm in joint 2
    if (pwm_cv2 > 90) // capping pwm for a safer control at joint 2
      pwm_cv2 = 90;
    if (err2 > 0) // reversing polarity of motor once error reaches its minimum and oscillates after at joint 2
      digitalWrite(dirPin2, HIGH);
    else
      digitalWrite(dirPin2, LOW);
   
     if(err1 < 1 && err2 < 1)
    {
      iter++;
      delay(200);
     if(iter >= numWP-1)
        iter = numWP-1;
    
    }
     analogWrite(pwmPin2, pwm_cv2); // actuating link 2
   
  }
}
// Tasks definition end
void setBNO0551(void)
{
  if (!bno1.begin())
  {
    Serial.print("bno055-1 not detected");
    while (1);
  }
  Serial.println("bno055-1 detected");
}
void setBNO0552(void)
{
  if (!bno2.begin())
  {
    Serial.print("bno055-2 not detected");
    while (1);
  }
  Serial.println("bno055-2 detcted");
}
double yBNO0551(void)
{
  sensors_event_t orientationData;
  bno1.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  double y = orientationData.orientation.z;

  return (y);
}
double yBNO0552(void)
{
  sensors_event_t orientationData;
  bno2.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  double y = orientationData.orientation.z;

  return (y);
}
// Function declaration end
