/***********************************************************************************
 *  }--\     RobotGeek Snapper Robotic Arm     /--{
 *      |         Analog Control Code         |
 *   __/                                       \__
 *  |__|                                       |__|
 *
 *
 *  The following sketch will move each joint of the arm simultaneously using interpolation and group-move capabilities of the ServoEx library. 
 *
 *  Snapper Arm Getting Started Guide
 *  http://learn.trossenrobotics.com/33-robotgeek-getting-started-guides/robotgeek-snapper-robot-arm/63-robotgeek-snapper-arm-getting-started-guide
 *
 *
 *  WIRING
 *    Servos
 *      Digital I/O 3 - Base Rotation - Robot Geek Servo 
 *      Digital I/O 5 - Shoulder Joint - Robot Geek Servo 
 *      Digital I/O 6 - Elbow Joint - Robot Geek Servo 
 *      Digital I/O 9 - Wrist Joint - Robot Geek Servo 
 *      Digital I/O 10 - Gripper Servo - 9g Servo 
 *
 *  
 *    Use an external power supply and set both PWM jumpers to 'VIN'
 *
 *  CONTROL
 *    Autonomous (see code below)
 *
 *
 *  NOTES
 *
 *    SERVO POSITIONS
 *    ADD INFO ABOUT SERVORX LIBRARY
 *      For RobotGeek servos, 600ms corresponds to fully counter-clockwise while
 *      2400ms corresponds to fully clock-wise. 1500ms represents the servo being centered 
 *
 *      For the 9g servo, 900ms corresponds to fully counter-clockwise while
 *      2100ms corresponds to fully clock-wise. 1500ms represents the servo being centered 
 *
 *
 *  This code is a Work In Progress and is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 ***********************************************************************************/


#include <ServoEx.h>

// instantiate array to hold servo objects (0-4 for 5 servos total)
ServoEx    ArmServo[5];

//////////////////////////////////////////////////////////////////////////////
// SERVO CONFIG  //
//////////////////////////////////////////////////////////////////////////////
// Declare servo names for objects.
enum {
  BAS_SERVO=0, SHL_SERVO, ELB_SERVO, WRI_SERVO, GRI_SERVO};

#define ROBOT_GEEK_9G_GRIPPER 1
#define ROBOT_GEEK_PARALLEL_GRIPPER 2

//The 9G gripper is the gripper with the small blue 9g servo
//The Parralle gripper has a full robotgeek servo and paralle rails
//Uncomment one of the following lines depending on which gripper you are using.
//#define GRIPPER_TYPE ROBOT_GEEK_9G_GRIPPER
#define GRIPPER_TYPE ROBOT_GEEK_PARALLEL_GRIPPER

#ifndef GRIPPER_TYPE
   #error YOU HAVE TO SELECT THE GRIPPER YOU ARE USING! Uncomment the correct line above for your gripper
#endif


// Servo position limitations - limits in microseconds
#define BASE_MIN      600     //full counterclockwise for RobotGeek 180 degree servo
#define BASE_MAX      2400    //full clockwise for RobotGeek 180 degree servo
#define SHOULDER_MIN  600
#define SHOULDER_MAX  2400
#define ELBOW_MIN     600
#define ELBOW_MAX     2400
#define WRIST_MIN     600
#define WRIST_MAX     2400 

//mins and maxes depending on gripper type
#if GRIPPER_TYPE == ROBOT_GEEK_9G_GRIPPER
  #define GRIPPER_MIN   900    //full counterclockwise for 9g servo
  #define GRIPPER_MAX   2100   //full clockwise for 9g servo
#elif GRIPPER_TYPE == ROBOT_GEEK_PARALLEL_GRIPPER
  #define GRIPPER_MIN   750    //full counterclockwise for 9g servo
  #define GRIPPER_MAX   2400   //full clockwise for 9g servo
#endif

// Define servo offsets in +/- uS. Adjust if your arm is not centering properly.
#define BAS_SERVO_ERROR 0 //(+ is CW, - is CCW)
#define SHL_SERVO_ERROR 0 //(+ is forward, - is backward)
#define ELB_SERVO_ERROR 0 //(+ is up, - is down)
#define WRI_SERVO_ERROR 0 //(+ is up, - is down)
#define GRI_SERVO_ERROR 0 //(+ is tighten grip, - is loosen grip) 

 //present positions of the servos 
int Base     =1500;    //holds the present position of the Base servo, starts at 1500 (centered)
int Shoulder =1500;    //holds the present position of the Shoulder servo, starts at 1500 (centered)
int Elbow    =1500;    //holds the present position of the Elbow servo, starts at 1500 (centered)
int Wrist    =1500;    //holds the present position of the wrist servo, starts at 1500 (centered)
int Gripper  =1500;    //holds the present position of the gripper servo, starts at 1500 (centered)

//////////////////////////////////////////////////////////////////////////////

int time  =1000;

 

void setup(){
Serial.begin(9600);
Serial.println("sss setup");
  // Attach servo and set limits
  ArmServo[BAS_SERVO].attach(3, BASE_MIN, BASE_MAX);
  ArmServo[SHL_SERVO].attach(5, SHOULDER_MIN, SHOULDER_MAX);
  ArmServo[ELB_SERVO].attach(6, ELBOW_MIN, ELBOW_MAX);
  ArmServo[WRI_SERVO].attach(9, WRIST_MIN, WRIST_MAX);
  ArmServo[GRI_SERVO].attach(10, GRIPPER_MIN, GRIPPER_MAX);
 
 
  // 
  SetServo(time);
  delay(time);
Serial.println("fins setup");
}
 
void loop(){
 
//position1
Base     =1000;
Shoulder =1300;
Elbow    =1400;
Wrist    =1800;
Gripper  =1700;
time     =2000;

//call SetServo Function, which organizes group servo move. 'time' is passed to it as the amount of time it takes for servos to reach goal position, based on longest servo range
  SetServo(time);
// delay so we don't move on to next step before servos complete movement. There is another, slightly more advanced way of doing this that we will cover later.  
  delay(time);
//optional delay added after the servo move, used to pause between movements.
  delay(1000);




//position2
Base     =1800;
Shoulder =1600;
Elbow    =1700;
Wrist    =1300;
Gripper  =1300;
time     =3000;

//call SetServo Function, which organizes group servo move. 'time' is passed to it as the amount of time it takes for servos to reach goal position, based on longest servo range
  SetServo(time);
// delay so we don't move on to next step before servos complete movement. There is another, slightly more advanced way of doing this that we will cover later.  
  delay(time);
//optional delay added after the servo move, used to pause between movements.  
  delay(1000);


//position3
Base     =1800;
Shoulder =1600;
Elbow    =1700;
Wrist    =1300;
Gripper  =1800;
time     =500;

//call SetServo Function, which organizes group servo move. 'time' is passed to it as the amount of time it takes for servos to reach goal position, based on longest servo range
  SetServo(time);
// delay so we don't move on to next step before servos complete movement. There is another, slightly more advanced way of doing this that we will cover later.  
  delay(time);
//optional delay added after the servo move, used to pause between movements.  
  delay(2000);
  
  
//position4
Base     =1800;
Shoulder =1600;
Elbow    =1700;
Wrist    =1300;
Gripper  =1300;
time     =500;

//call SetServo Function, which organizes group servo move. 'time' is passed to it as the amount of time it takes for servos to reach goal position, based on longest servo range
  SetServo(time);
// delay so we don't move on to next step before servos complete movement. There is another, slightly more advanced way of doing this that we will cover later.  
  delay(time);
//optional delay added after the servo move, used to pause between movements.  
  delay(1000);
  
  
//position5
Base     =1500;
Shoulder =1500;
Elbow    =1500;
Wrist    =1500;
Gripper  =1500;
time     =1000;

//call SetServo Function, which organizes group servo move. 'time' is passed to it as the amount of time it takes for servos to reach goal position, based on longest servo range
  SetServo(time);
// delay so we don't move on to next step before servos complete movement. There is another, slightly more advanced way of doing this that we will cover later.  
  delay(time);
//optional delay added after the servo move, used to pause between movements.  
  delay(1000);
  
 }
 


//===================================================================================================
// SetServo: Writes Servo Positions using ServoGroupMove from ServoEX library
//===================================================================================================
void SetServo(unsigned int DeltaTime)
{

  ServoGroupMove.start();   //start group move
  ArmServo[BAS_SERVO].writeMicroseconds(Base + BAS_SERVO_ERROR); //set position to 1st servo, account for error adjustments in defines
  ArmServo[SHL_SERVO].writeMicroseconds(Shoulder + SHL_SERVO_ERROR); // "" ""
  ArmServo[ELB_SERVO].writeMicroseconds(Elbow + ELB_SERVO_ERROR);
  ArmServo[WRI_SERVO].writeMicroseconds(Wrist + WRI_SERVO_ERROR);
  ArmServo[GRI_SERVO].writeMicroseconds(Gripper + GRI_SERVO_ERROR);
  ServoGroupMove.commit(DeltaTime); //commit the group move, pass along an interpolation time value
}


//===================================================================================================
// SetServoCenter: Move servos to center position using ServoGroupMove from ServoEX library
//===================================================================================================
void SetServoCenter(unsigned int DeltaTime)
{
  ServoGroupMove.start();
  ArmServo[BAS_SERVO].writeMicroseconds(1500 + BAS_SERVO_ERROR);
  ArmServo[SHL_SERVO].writeMicroseconds(1500 + SHL_SERVO_ERROR);
  ArmServo[ELB_SERVO].writeMicroseconds(1500 + ELB_SERVO_ERROR);
  ArmServo[WRI_SERVO].writeMicroseconds(1500 + WRI_SERVO_ERROR);
  ArmServo[GRI_SERVO].writeMicroseconds(1500 + GRI_SERVO_ERROR);
  ServoGroupMove.commit(DeltaTime);
}
