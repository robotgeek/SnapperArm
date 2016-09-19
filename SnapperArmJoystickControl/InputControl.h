#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H

#include "Kinematics.h"
#include "GlobalArm.h"

extern ServoEx    ArmServo[5];


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

//deadband value - not all joystics will center at 512, so this value removes 'positional-drift' caused from joysticks that are off-center.
#define DEADBAND 55 //Increase this value if positional drift occurs with no human input

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

int DeadBand( int p_deadband, int p_value )
{
  if ( p_value > 0 )
  {
    if ( p_value - p_deadband > 0 )
    {
      return p_value - p_deadband;
    }
    else return 0;
  }
  else if ( p_value < 0 )
  {
    if ( p_value + p_deadband < 0 )
    {
      return p_value + p_deadband;
    } else return 0;
  }
  return 0;
}
//////////////////////////////////////////////////////////////////////////////



//===================================================================================================
// ProcessAnalogInput3D: Process input from Analog Joysticks, convert into X,Y,Z coord
//===================================================================================================
boolean ProcessAnalogInput3D(void) {

    boolean fChanged = false;
    
    //read analog values from analog sensors
   joyXVal = analogRead(ANALOGX) - 512;
   joyYVal = analogRead(ANALOGY) - 512;
   joyZVal = analogRead(ANALOGZ) - 512;
   joyGAVal = analogRead(ANALOGGA) - 512;
   joyGripperVal = analogRead(ANALOGGRIP);
   delay(delayTime);  //slow down the readings - remove


   joyXVal = DeadBand( DEADBAND, joyXVal );
   if ( joyXVal != 0 )
   {
     joyXMapped = mapfloat(joyXVal, -512, 511, -spd, spd); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sIKX -= joyXMapped;
   }
   
   joyYVal = DeadBand( DEADBAND, joyYVal );
   if ( joyYVal != 0 )
   {
     joyYMapped = mapfloat(joyYVal, -512, 511, -spd, spd); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sIKY -= joyYMapped;
   }
   
   joyZVal = DeadBand( DEADBAND, joyZVal );
   if ( joyZVal != 0 )
   {
     joyZMapped = mapfloat(joyZVal, -512, 511, -spd, spd); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sIKZ -= joyZMapped;
   }
   
   joyGAVal = DeadBand( DEADBAND, joyGAVal );
   if ( joyGAVal != 0 )
   {
     joyGAMapped = mapfloat(joyGAVal, -512, 511, -spd, spd); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sIKGA += joyGAMapped;
   }
   
   //Mapping analog joystick value to servo PWM signal range
   joyGripperMapped = mapfloat(joyGripperVal, 0, 1023, GRIPPER_MAX, GRIPPER_MIN);
   sGrip = joyGripperMapped;//set servo position variable to the mapped value from the knob

   

    sIKX = min(max(sIKX, IK_MIN_X), IK_MAX_X);
    sIKY = min(max(sIKY, IK_MIN_Y), IK_MAX_Y);
    sIKZ = min(max(sIKZ, IK_MIN_Z), IK_MAX_Z);
    sIKGA = min(max(sIKGA, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...
    

    
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
  

  if(fChanged)
  {
    doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 

  }


  return fChanged;

}



//===================================================================================================
// ProcessAnalogInput: Process input from Analog Joysticks, convert into Base ,Y,Z coord
//===================================================================================================
boolean ProcessAnalogInputCylindrical(void) {

    boolean fChanged = false;
    
    //read analog values from analog sensors
   joyXVal = analogRead(ANALOGX) - 512;
   joyYVal = analogRead(ANALOGY) - 512;
   joyZVal = analogRead(ANALOGZ) - 512;
   joyGAVal = analogRead(ANALOGGA) - 512;
   joyGripperVal = analogRead(ANALOGGRIP);
   delay(delayTime);

   joyXVal = DeadBand( DEADBAND, joyXVal );
   if ( joyXVal != 0 )
   {
     joyXMapped = mapfloat(joyXVal, -512, 511, -spd * 10, spd * 10); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sBase += joyXMapped;
   }

    //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   joyYVal = DeadBand( DEADBAND, joyYVal );
   if ( joyYVal != 0 )
   {
     joyYMapped = mapfloat(joyYVal, -512, 511, -spd, spd); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sIKY -= joyYMapped;
   }
   
   joyZVal = DeadBand( DEADBAND, joyZVal );
   if ( joyZVal != 0 )
   {
     joyZMapped = mapfloat(joyZVal, -512, 511, -spd, spd); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sIKZ -= joyZMapped;
   }

   joyGAVal = DeadBand( DEADBAND, joyGAVal );
   if ( joyGAVal != 0 )
   {
     joyGAMapped = mapfloat(joyGAVal, -512, 511, -spd, spd); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sIKGA += joyGAMapped;
   }
     
   //Mapping analog joystick value to servo PWM signal range
   joyGripperMapped = mapfloat(joyGripperVal, 0, 1023, GRIPPER_MAX, GRIPPER_MIN);
   sGrip = joyGripperMapped;//set servo position variable to the mapped value from the knob

   
//     

    sBase = min(max(sBase, BASE_MIN), BASE_MAX);
    sIKY = min(max(sIKY, IK_MIN_Y), IK_MAX_Y);
    sIKZ = min(max(sIKZ, IK_MIN_Z), IK_MAX_Z);
    sIKGA = min(max(sIKGA, IK_MIN_GA), IK_MAX_GA);  





  fChanged = (sBase != g_sBase) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA);

  if(fChanged)
  {
      doArmIK(false, sBase, sIKY, sIKZ, sIKGA); 

  }

  return fChanged;

}



//===================================================================================================
// ProcessAnalogBackHoe
//===================================================================================================
boolean ProcessAnalogBackhoe() {

  
    boolean fChanged = false;
   //read analog values from analog sensors
   joyXVal = analogRead(ANALOGX) - 512;
   joyYVal = analogRead(ANALOGY) - 512;
   joyZVal = analogRead(ANALOGZ) - 512;
   joyGAVal = analogRead(ANALOGGA) - 512;
   joyGripperVal = analogRead(ANALOGGRIP);
   delay(delayTime);


   joyXVal = DeadBand( DEADBAND, joyXVal );
   if ( joyXVal != 0 )
   {
     joyXMapped = mapfloat(joyXVal, -512, 511, -spd * 10, spd * 10); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sBase += joyXMapped;
   }
   
   joyYVal = DeadBand( DEADBAND, joyYVal );
   if ( joyYVal != 0 )
   {
     joyYMapped = mapfloat(joyYVal, -512, 511, -spd * 10, spd * 10); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sShoulder += joyYMapped;
   }
   
   joyZVal = DeadBand( DEADBAND, joyZVal );
   if ( joyZVal != 0 )
   {
     joyZMapped = mapfloat(joyZVal, -512, 511, -spd * 10, spd * 10); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sElbow -= joyZMapped;
   }
   
   joyGAVal = DeadBand( DEADBAND, joyGAVal );
   if ( joyGAVal != 0 )
   {
     joyGAMapped = mapfloat(joyGAVal, -512, 511, -spd * 10, spd * 10); //Map offset analog value (-512 to 511) to incremental change (-spd to spd)
     sWrist -= joyGAMapped;
   }
   
   //Mapping analog joystick value to servo PWM signal range
   joyGripperMapped = mapfloat(joyGripperVal, 0, 1023, GRIPPER_MAX, GRIPPER_MIN);
   sGrip = joyGripperMapped; //set servo position variable to the mapped value from the knob

      
    //enforce upper/lower limits for servo position variable 
    //(actual servo positions min/max are setup above as a part of .attach(), this is just for variable)


    sBase = min(max(sBase, BASE_MIN), BASE_MAX);
    sShoulder = min(max(sShoulder, SHOULDER_MIN), SHOULDER_MAX);
    sElbow = min(max(sElbow, ELBOW_MIN), ELBOW_MAX);
    sWrist = min(max(sWrist, WRIST_MIN), WRIST_MAX);
    
    sGrip = min(max(sGrip, GRIPPER_MIN), GRIPPER_MAX);

    fChanged = (g_sBase != sBase) || (g_sShoulder != sShoulder) || (g_sElbow != sElbow) || (g_sWrist != sWrist)|| (g_sGrip != sGrip);




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
