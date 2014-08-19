#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H

#include "GlobalArm.h"
#include <Arduino.h>

extern ServoEx    ArmServo[5];

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

//=============================================================================
// ANALOG INPUT CONFIG  // 
//=============================================================================
//define analog pins that will be connected to the joystick pins
#define BASE     0  //connected to Horizontal Axis on Joystick # 1
#define SHOULDER 1  //connected to Vertical Axis on Joystick # 2
#define ELBOW    2  //connected to Vertical Axis on Joystick # 3
#define WRIST    3  //connected to Vertical Axis on Joystick # 4
#define GRIPPER  4  //connected to Rotation Knob / Potentiometer # 1

//generic deadband limits - not all joystics will center at 512, so these limits remove 'drift' from joysticks that are off-center.
#define DEADBANDLOW 492   //decrease this value if drift occurs, increase it to increase sensitivity around the center position
#define DEADBANDHIGH 532  //increase this value if drift occurs, decrease it to increase sensitivity around the center position

//last read values of analog sensors (Native values, 0-1023)
int joyBaseVal = 0;     //present value of the base rotation knob (analog 0)
int joyShoulderVal = 0; //present value of the shoulder joystick (analog 1)
int joyElbowVal = 0;    //present value of the elbow joystick (analog 2)
int joyWristVal = 0;    //present value of the wrist joystick (analog 3)
int joyGripperVal = 0;  //present value of the gripper rotation knob (analog 4)

//last calculated values of analog sensors (Mapped values)
//knob values (base and gripper) will be mapped directly to the servo limits
//joystick values (shoulder, elbow and wrist) will be mapped from -spd to spd, to faciliate incremental control
float joyBaseMapped = 0;      //base knob value, mapped from 1-1023 to BASE_MIN-BASE_MAX
float joyShoulderMapped = 0;  //shoulder joystick value, mapped from 1-1023 to -spd to spd
float joyElbowMapped = 0;     //elbow joystick value, mapped from 1-1023 to -spd to spd
float joyWristMapped = 0;     //wrist joystick value, mapped from 1-1023 to -spd to spd
float joyGripperMapped = 0;   //gripper knob  value, mapped from 1-1023 to GRIPPER_MIN-GRIPPER_MAX

float spd = 10.00;  //speed modififer, increase this to increase the speed of the movement

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//=============================================================================
//=============================================================================

/******************************************************
 *  SetServo()
 *
 *  This function sets the 5 servos on the snapper arm
 *  to the 5 positions variables. All servos and
 *  positions variables are globals 
 *
 *  Parameters:
 *    DeltaTime  - interpolation value in mS
  *
 *  Globals Used:
 *      ArmServo BAS_SERVO
 *      ArmServo WRI_SERVO
 *      ArmServo SHL_SERVO
 *      ArmServo ELB_SERVO
 *      ArmServo GRI_SERVO
 *      int Base
 *      int Wrist
 *      int Shoulder
 *      int Elbow
 *      int Gripper
 *
 *  Returns: 
 *    none
 ******************************************************/ 
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



//===================================================================================================
// ProcessAnalogBackHoe
//===================================================================================================
void ProcessAnalogBackhoe() {

  
   //read analog values from analog sensors
   joyBaseVal = analogRead(BASE);
   joyShoulderVal = analogRead(SHOULDER);
   joyElbowVal = analogRead(ELBOW);
   joyWristVal = analogRead(WRIST);
   joyGripperVal = analogRead(GRIPPER);
   delay(5);
   
// Base joint is handled differently depending upon analog control solution. v1.0 Snapper Arms used a rotational knob with direct/absolute control of the base servo, where v1.1 Snapper Arms use an incremental Joystick.    
//        
#ifdef v10   
//   joyBaseMapped = mapfloat(joyBaseVal, 1023, 0, BASE_MIN, BASE_MAX);  //Mapping analog knob value to servo PWM signal range
 //  Base = joyBaseMapped; //set servo position variable to the mapped value from the knob. Absolute/direct control.
#endif

#ifdef v11   
   //only update the base joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyBaseVal > DEADBANDHIGH || joyBaseVal < DEADBANDLOW)
   {
     joyBaseMapped = mapfloat(joyBaseVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
     Base += joyBaseMapped; //add mapped base joystick value to present Base Value (positive values of joyBaseMapped will increase the position, negative values will decrease the position)
   }
#endif


   //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyShoulderVal > DEADBANDHIGH || joyShoulderVal < DEADBANDLOW)
   {
     joyShoulderMapped = mapfloat(joyShoulderVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
     Shoulder = Shoulder - joyShoulderMapped; //add mapped shoulder joystick value to present Shoulder Value (positive values of joyShoulderMapped will increase the position, negative values will decrease the position)
   }

   //only update the elbow joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyElbowVal > DEADBANDHIGH || joyElbowVal < DEADBANDLOW)
   {
     joyElbowMapped = mapfloat(joyElbowVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
     Elbow = Elbow + joyElbowMapped;//add mapped elbow joystick value to present elbow Value (positive values of joyElbowMapped will increase the position, negative values will decrease the position)
   }
   

   //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyWristVal > DEADBANDHIGH || joyWristVal < DEADBANDLOW)
   {
     joyWristMapped = mapfloat(joyWristVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
     Wrist = Wrist + joyWristMapped;//add mapped wrist joystick value to present wrist Value (positive values of joyWristMapped will increase the position, negative values will decrease the position)
   }
   
   
   //Mapping analog joystick value to servo PWM signal range
   joyGripperMapped = mapfloat(joyGripperVal, 0, 1023, GRIPPER_MIN, GRIPPER_MAX);
   Gripper = joyGripperMapped;//set servo position variable to the mapped value from the knob

      
    //enforce upper/lower limits for servo position variable 
    //(actual servo positions min/max are setup above as a part of .attach(), this is just for variable)
    if (Base < BASE_MIN)
    {
      Base =BASE_MIN;
    }  
    else if (Base > BASE_MAX)
    {
      Base =BASE_MAX;
    }
    
    //enforce upper/lower limits for shoulder servo
    if (Shoulder < SHOULDER_MIN)
    {
      Shoulder =SHOULDER_MIN;
    }  
  
    else if (Shoulder > SHOULDER_MAX)
    {
      Shoulder =SHOULDER_MAX;
    }
  
  
  
    //enforce upper/lower limits for elbow servo
    if (Elbow < ELBOW_MIN)
    {
      Elbow = ELBOW_MIN;
    }  
    else if (Elbow > ELBOW_MAX)
    {
      Elbow = ELBOW_MAX;
    }
    
    
    //enforce upper/lower limits for wrist servo
    if (Wrist < WRIST_MIN)
    {
      Wrist = WRIST_MIN;
    }  
  
    else if (Wrist > WRIST_MAX)
    {
      Wrist =WRIST_MAX;
    }
  
  
    //enforce upper/lower limits for gripper servo
    if (Gripper < GRIPPER_MIN)
    {
      Gripper =GRIPPER_MIN;
    }  
    else if (Gripper > GRIPPER_MAX)
    {
      Gripper = GRIPPER_MAX;
    }
    
    //Function to set PWM Servo positions
    SetServo(0);
    
  }




//=============================================================================
//=============================================================================



#endif
