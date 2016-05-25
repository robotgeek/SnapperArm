#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H

#include "Kinematics.h"
#include "GlobalArm.h"
#include <Commander.h>

extern ServoEx    ArmServo[5];
extern Commander command ;
extern int speedMod;

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


int IKXChange;
int IKYChange;
int IKZChange;
int baseChange;
int shoulderChange;
int elbowChange;
int wristChange;
int gripperChange;


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
 if(command.buttons & BUT_LT)
  {
   IKXChange = 0;
   IKYChange = 0;
   IKZChange = 0;
   wristChange = 0;
   gripperChange = command.lookV/(speedMod/2);
  }
  else
  {  
   IKXChange = command.walkH/speedMod;
   IKYChange = command.walkV/speedMod;
   IKZChange = command.lookV/speedMod;
   wristChange = command.lookH/speedMod;
   gripperChange = 0;
  
  }
   

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

  if(command.buttons & BUT_LT)
  {
   baseChange = 0;
   IKYChange = 0;
   IKZChange = 0;
   wristChange = 0;
   gripperChange = command.lookV/(speedMod/2);
  }
  else
  {  
   baseChange = command.walkH/(speedMod/2);
   IKYChange = command.walkV/speedMod;
   IKZChange = command.lookV/speedMod;
   wristChange = command.lookH/speedMod;
   gripperChange = 0;
  
  }




}


//===================================================================================================
// ProcessAnalogBackHoe
//===================================================================================================
boolean ProcessUserInputBackHoe() {
  
  if(command.buttons & BUT_LT)
  {
   baseChange = 0;
   elbowChange = 0;
   shoulderChange = 0;
   wristChange = 0;
   gripperChange = command.lookV/(speedMod/2);
  }
 
  else
  {  
   baseChange = command.walkH/(speedMod/2);
   elbowChange = command.walkV/(speedMod/2);
   shoulderChange = command.lookV/(speedMod/2);
   wristChange = command.lookH/(speedMod/2);
   gripperChange = 0;
  
  }
   


  
  
}





boolean updateServoPositions()
{

  boolean fChanged = false;
  
 switch (g_bIKMode) 
        {
          case IKM_IK3D_CARTESIAN:
            sIKX = constrain(sIKX + IKXChange, IK_MIN_X, IK_MAX_X);
            sIKY = constrain(sIKY + IKYChange, IK_MIN_Y, IK_MAX_Y);
            sIKZ = constrain(sIKZ + IKZChange, IK_MIN_Z, IK_MAX_Z);
            sIKGA = constrain(sIKGA + wristChange, IK_MIN_GA, IK_MAX_GA);
            sGrip = constrain(sGrip + gripperChange, GRIPPER_MIN, GRIPPER_MAX);
            
            fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) ;
            if (fChanged) 
            {
              g_bIKStatus = doArmIK(true, sIKX, sIKY, sIKZ, sIKGA);
            }


            break;
            
          case IKM_CYLINDRICAL:
            sBase = constrain(sBase + baseChange, BASE_MIN, BASE_MAX);
            sIKY = constrain(sIKY + IKYChange, IK_MIN_Y, IK_MAX_Y);
            sIKZ = constrain(sIKZ + IKZChange, IK_MIN_Z, IK_MAX_Z);
            sIKGA = constrain(sIKGA + wristChange, IK_MIN_GA, IK_MAX_GA);
            sGrip = constrain(sGrip + gripperChange, GRIPPER_MIN, GRIPPER_MAX);

            fChanged = (sBase != g_sBase) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) ;
            if (fChanged) 
            {
              g_bIKStatus = doArmIK(false, sBase, sIKY, sIKZ, sIKGA);
            }
              
            break;

          case IKM_BACKHOE:
            sBase = constrain(sBase + baseChange, BASE_MIN, BASE_MAX);
            sShoulder = constrain(sShoulder + shoulderChange, SHOULDER_MIN, SHOULDER_MAX);
            sElbow = constrain(sElbow + elbowChange, ELBOW_MIN, ELBOW_MAX);
            sWrist = constrain(sWrist + wristChange, WRIST_MIN, WRIST_MAX);
            sGrip = constrain(sGrip + gripperChange, GRIPPER_MIN, GRIPPER_MAX);
            
            break;
        }



  
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
