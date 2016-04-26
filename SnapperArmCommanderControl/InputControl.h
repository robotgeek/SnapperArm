#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H

#include "Kinematics.h"
#include "GlobalArm.h"
#include <Commander.h>

extern ServoEx    ArmServo[5];
extern Commander command ;

//=============================================================================
// Global Variables...
//=============================================================================
boolean  g_fArmActive = false;   // Is the arm logically on?

uint8_t  g_bIKStatus = IKS_SUCCESS;   // Status of last call to DoArmIK;;

//=============================================================================
// DIGITAL INPUT CONFIG...
//=============================================================================

// use #define to set the I/O numbers, since these will never change - this saves us memory while the Arduino is running
#define BUTTON1 2
#define BUTTON2 4

//variables to hold the current status of the button.(LOW == unpressed, HIGH = pressed)
int buttonState1 = LOW;         
int buttonState2 = LOW;            
boolean   loopbreak = LOW;

//////////////////////////////////////////////////////////////////////////////
// ANALOG INPUT CONFIG  // 
//////////////////////////////////////////////////////////////////////////////
//define analog pins that will be connected to the joystick pins
#define ANALOGX       0  //connected to Horizontal Axis on Joystick # 1
#define ANALOGY       1  //connected to Vertical Axis on Joystick # 2
#define ANALOGZ       2  //connected to Vertical Axis on Joystick # 3
#define ANALOGGA      3  //connected to Vertical Axis on Joystick # 4
#define ANALOGGRIP    4  //connected to Rotation Knob / Potentiometer # 1

//generic deadband limits - not all joystics will center at 512, so these limits remove 'drift' from joysticks that are off-center.
#define DEADBANDLOW 492   //decrease this value if drift occurs, increase it to increase sensitivity around the center position
#define DEADBANDHIGH 532  //increase this value if drift occurs, decrease it to increase sensitivity around the center position

 //last read values of analog sensors (Native values, 0-1023)
int joyXVal = 0;     //present value of the base rotation knob (analog 0)
int joyYVal = 0; //present value of the shoulder joystick (analog 1)
int joyZVal = 0;    //present value of the elbow joystick (analog 2)
int joyGAVal = 0;    //present value of the wrist joystick (analog 3)
int joyGripperVal = 0;  //present value of the gripper rotation knob (analog 4)

//last calculated values of analog sensors (Mapped values)
//knob values (base and gripper) will be mapped directly to the servo limits
//joystick values (shoulder, elbow and wrist) will be mapped from -spd to spd, to faciliate incremental control
float joyXMapped = 0;      //base knob value, mapped from 1-1023 to BASE_MIN-BASE_MAX
float joyYMapped = 0;  //shoulder joystick value, mapped from 1-1023 to -spd to spd
float joyZMapped = 0;     //elbow joystick value, mapped from 1-1023 to -spd to spd
float joyGAMapped = 0;     //wrist joystick value, mapped from 1-1023 to -spd to spd
float joyGripperMapped = 0;   //gripper knob  value, mapped from 1-1023 to GRIPPER_MIN-GRIPPER_MAX

float spd = 1.00;  //speed modififer, increase this to increase the speed of the movement
int delayTime = 5; //milliseocnds to delay in each processAnalog function - reduce this to get full speed from the arm

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//////////////////////////////////////////////////////////////////////////////



//===================================================================================================
// ProcessUserInput3D: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInput3D(void) {
  // We Are in IK mode, so figure out what position we would like the Arm to move to.
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //
  boolean fChanged = false;


    sIKX = min(max(sIKX + command.walkH/10, IK_MIN_X), IK_MAX_X);
    sIKY = min(max(sIKY + command.walkV/10, IK_MIN_Y), IK_MAX_Y);
    sIKZ = min(max(sIKZ + command.lookV/15, IK_MIN_Z), IK_MAX_Z);
      if (command.buttons & BUT_LT) 
      {
        sIKGA = min(max(sIKGA + command.lookH/30, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...
      }
      else 
      {
      sIKGA = min(max(sIKGA, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...    
      }
    


  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) ;

  if (fChanged) {
    g_bIKStatus = doArmIK(true, sIKX, sIKY, sIKZ, sIKGA);
  }
  return fChanged;
}

//===================================================================================================
// ProcessUserInputCylindrical: Process the Userinput when we are in 3d Mode
//===================================================================================================
boolean ProcessUserInputCylindrical() {
  // We Are in IK mode, so figure out what position we would like the Arm to move to.
  // We have the Coordinates system like:
  //
  //                y   Z
  //                |  /
  //                |/
  //            ----+----X (X and Y are flat on ground, Z is up in air...
  //                |
  //                |
  //
  boolean fChanged = false;
  


  // The base rotate is real simple, just allow it to rotate in the min/max range...
  sBase = min(max(g_sBase - command.walkH/10, BASE_MIN), BASE_MAX);

  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...
  // Use Y for 2d distance from base
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKY > 0) && (command.walkV < 0)) || ((g_sIKY < 0) && (command.walkV > 0)))
    sIKY += command.walkV/10;

  // Now Z coordinate...
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKZ > 0) && (command.lookV < 0)) || ((g_sIKZ < 0) && (command.lookV > 0)))
    sIKZ += command.lookV/15;

  // And gripper angle.  May leave in Min/Max here for other reasons...   
  if (command.buttons & BUT_LT) {
  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKGA > 0) && (command.lookH < 0)) || ((g_sIKGA < 0) && (command.lookH > 0)))
    sIKGA = min(max(sIKGA + command.lookH/30, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...
  }

  fChanged = (sBase != g_sBase) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) ;

  if (fChanged) {
    g_bIKStatus = doArmIK(false, sBase, sIKY, sIKZ, sIKGA);
  }
  return fChanged;
}


//===================================================================================================
// ProcessAnalogBackHoe
//===================================================================================================
boolean ProcessUserInputBackHoe() {
  // lets update positions with the 4 joystick values
  // First the base
  boolean fChanged = false;
  sBase = min(max(g_sBase - command.walkH/6, BASE_MIN), BASE_MAX);
  if (sBase != g_sBase)
    fChanged = true;

  // Now the Boom
  sShoulder = min(max(g_sShoulder + command.lookV/6, SHOULDER_MIN), SHOULDER_MAX);
  if (sShoulder != g_sShoulder)
    fChanged = true;

  // Now the Dipper 
  sElbow = min(max(g_sElbow + command.walkV/6, ELBOW_MIN), ELBOW_MAX);
  if (sElbow != g_sElbow)
    fChanged = true;

  // Bucket Curl
  sWrist = min(max(g_sWrist + command.lookH/6, WRIST_MIN), WRIST_MAX);
  if (sWrist != g_sWrist)
    fChanged = true;
  return fChanged;
}



//===================================================================================================
// SetServo: Writes Servo Position Solutions
//===================================================================================================
void SetServo(unsigned int DeltaTime)
{
  ServoGroupMove.start();
  ArmServo[BAS_SERVO].writeMicroseconds(Base + BAS_SERVO_ERROR);
  ArmServo[SHL_SERVO].writeMicroseconds(Shoulder + SHL_SERVO_ERROR);
  ArmServo[ELB_SERVO].writeMicroseconds(Elbow + ELB_SERVO_ERROR);
  ArmServo[WRI_SERVO].writeMicroseconds(Wrist + WRI_SERVO_ERROR);
  ArmServo[GRI_SERVO].writeMicroseconds(Gripper + GRI_SERVO_ERROR);
  ServoGroupMove.commit(DeltaTime);
}

//=============================================================================
//=============================================================================



void zero_x()
{
  for(double yaxis = 150.0; yaxis < 200.0; yaxis += 1){
    doArmIK(true, 0, yaxis, 100.0, 0);
    SetServo(0);
    delay(10);
  }
  for(double yaxis = 200.0; yaxis > 150.0; yaxis -= 1){
    doArmIK(true, 0, yaxis, 100.0, 0);
    SetServo(0);
    delay(10);
  }
}
 
/* moves arm in a straight line */
void line()
{
    for(double xaxis = -100.0; xaxis < 100.0; xaxis += 0.5){
      doArmIK(true, xaxis, 200, 100, 0);
      SetServo(0);
      delay(10);
    }
    for(float xaxis = 100.0; xaxis > -100.0; xaxis -= 0.5){
      doArmIK(true, xaxis, 200, 100, 0);
      SetServo(0);
      delay(10);
    }
}
 
void circle()
{
  #define RADIUS 20.0
  //float angle = 0;
  float zaxis,yaxis;
  for(float angle = 0.0; angle < 360.0; angle += 1.0){
      yaxis = RADIUS * sin(radians(angle)) + 150;
      zaxis = RADIUS * cos(radians(angle)) + 150;
      doArmIK(true, 0, yaxis, zaxis, 0);
      SetServo(0);
      delay(5);
  }
}



#endif
