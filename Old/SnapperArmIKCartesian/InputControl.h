#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H

#include "Kinematics.h"
#include "GlobalArm.h"

extern ServoEx    ArmServo[5];


//=============================================================================
// Global Variables...
//=============================================================================
boolean  g_fArmActive = false;   // Is the arm logically on?


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

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//////////////////////////////////////////////////////////////////////////////



//===================================================================================================
// ProcessAnalogInput3D: Process input from Analog Joysticks, convert into X,Y,Z coord
//===================================================================================================
boolean ProcessAnalogInput3D(void) {

    boolean fChanged = false;
    
    //read analog values from analog sensors
   joyXVal = analogRead(ANALOGX);
   joyYVal = analogRead(ANALOGY);
   joyZVal = analogRead(ANALOGZ);
   joyGAVal = analogRead(ANALOGGA);
   joyGripperVal = analogRead(ANALOGGRIP);
   delay(5);


   //only update the base joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
   {
     joyXMapped = mapfloat(joyXVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
     g_sIKX -= joyXMapped;
   }

   //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
   {
     joyYMapped = mapfloat(joyYVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
     g_sIKY -= joyYMapped;
   }

   //only update the elbow joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
   {
     joyZMapped = mapfloat(joyZVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
     g_sIKZ -= joyZMapped;
   }
   
   //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
   {
     joyGAMapped = mapfloat(joyGAVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
     g_sIKGA += joyGAMapped;
   }
     
   //Mapping analog joystick value to servo PWM signal range
   joyGripperMapped = mapfloat(joyGripperVal, 0, 1023, GRIPPER_MIN, GRIPPER_MAX);
   sGrip = joyGripperMapped;//set servo position variable to the mapped value from the knob

   
//     
    sIKX = min(max(g_sIKX, IK_MIN_X), IK_MAX_X);
    sIKY = min(max(g_sIKY, IK_MIN_Y), IK_MAX_Y);
    sIKZ = min(max(g_sIKZ, IK_MIN_Z), IK_MAX_Z);
    sIKGA = min(max(g_sIKGA, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...
    
//      // remember our current IK position
//    g_sIKX = sIKX; 
//    g_sIKY = sIKY;
//    g_sIKZ = sIKZ;
//    g_sIKGA = sIKGA;


// IK STATUS CHECK NEEDS TO BE IMPLEMENTED, TODO
// Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...
//  else {
//    // In an Error/warning condition, only allow things to move in closer...
//    sIKX = g_sIKX;
//    sIKY = g_sIKY;
//    sIKZ = g_sIKZ;
//    sIKGA = g_sIKGA;
//
//    if (((g_sIKX > 0) && (command.walkH < 0)) || ((g_sIKX < 0) && (command.walkH > 0)))
//      sIKX = min(max(g_sIKX + command.walkH/10, IK_MIN_X), IK_MAX_X);
//    if (((g_sIKY > 0) && (command.walkV < 0)) || ((g_sIKY < 0) && (command.walkV > 0)))
//      sIKY = min(max(g_sIKY + command.walkV/10, IK_MIN_Y), IK_MAX_Y);
//    if (((g_sIKZ > 0) && (command.lookV < 0)) || ((g_sIKZ < 0) && (command.lookV > 0)))
//      sIKZ = min(max(g_sIKZ + command.lookV/15, IK_MIN_Z), IK_MAX_Z);
//    if (((g_sIKGA > 0) && (command.lookH < 0)) || ((g_sIKGA < 0) && (command.lookH > 0))) 
//      sIKGA = min(max(g_sIKGA, IK_MIN_GA), IK_MAX_GA);            
//  }

  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA);

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
 
void circle(int radius)
{
  #define RADIUS 20.0
  //float angle = 0;
  float xaxis,yaxis;
  for(float angle = 0.0; angle < 360.0; angle += 1.0){
      yaxis = radius * sin(radians(angle)) + 150;
      xaxis = radius * cos(radians(angle)) ;
      doArmIK(true, xaxis, yaxis, 100, 0);
      sGrip = 1000;
          MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, sDeltaTime, true);

      SetServo(0);
      delay(10);
  }
}



#endif
