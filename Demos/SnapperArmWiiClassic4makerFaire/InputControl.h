#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H

#include "Kinematics.h"
#include "GlobalArm.h"

extern ServoEx    ArmServo[5];
extern WiiClassy classy;     
boolean   loopbreak = LOW;

//=============================================================================
// Global Variables...
//=============================================================================
boolean  g_fArmActive = false;   // Is the arm logically on?

uint8_t  g_bIKStatus = IKS_SUCCESS;   // Status of last call to DoArmIK;;

//=============================================================================
// DIGITAL INPUT CONFIG...
//=============================================================================

#define WII_JOYSTICK_MAX 63

//generic deadband limits - not all joystics will center at 512, so these limits remove 'drift' from joysticks that are off-center.
#define DEADBANDLOW 30   //decrease this value if drift occurs, increase it to increase sensitivity around the center position
#define DEADBANDHIGH 34  //increase this value if drift occurs, decrease it to increase sensitivity around the center position

 //last read values of analog sensors (Native values, 0-WII_JOYSTICK_MAX)
int joyXVal = 0;     //present value of the base rotation knob (analog 0)
int joyYVal = 0; //present value of the shoulder joystick (analog 1)
int joyZVal = 0;    //present value of the elbow joystick (analog 2)
int joyGAVal = 0;    //present value of the wrist joystick (analog 3)
int joyGripperVal = 0;  //present value of the gripper rotation knob (analog 4)

//last calculated values of analog sensors (Mapped values)
//knob values (base and gripper) will be mapped directly to the servo limits
//joystick values (shoulder, elbow and wrist) will be mapped from -spd to spd, to faciliate incremental control
float joyXMapped = 0;      //base knob value, mapped from 1-WII_JOYSTICK_MAX to BASE_MIN-BASE_MAX
float joyYMapped = 0;  //shoulder joystick value, mapped from 1-WII_JOYSTICK_MAX to -spd to spd
float joyZMapped = 0;     //elbow joystick value, mapped from 1-WII_JOYSTICK_MAX to -spd to spd
float joyGAMapped = 0;     //wrist joystick value, mapped from 1-WII_JOYSTICK_MAX to -spd to spd
float joyGripperMapped = 0;   //gripper knob  value, mapped from 1-WII_JOYSTICK_MAX to GRIPPER_MIN-GRIPPER_MAX

float spd = 1.00;  //speed modififer, increase this to increase the speed of the movement
float spdMod = .1;
float maxSpd = 5;
float minSpd = .2;
unsigned long lastArmSpeedUpdate;

int gripSpd = 10;
int gripSpdMod = 5;
int maxGripSpd = 100;
int minGripSpd = 1;
unsigned long lastGripperSpeedUpdate;

int speedUpdateInterval = 100; //10hz

bool lastLButtonState;
bool lastRButtonState;

int delayTime = 5; //milliseocnds to delay in each processAnalog function - reduce this to get full speed from the arm

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//////////////////////////////////////////////////////////////////////////////




void readClassicJoysticks()
{  
   //read analog values from analog sensors
   joyXVal = classy.leftStickX;                       //read standard left stick data (0-63)
   joyYVal = classy.leftStickY;                       //read standard left stick  data (0-63)
   joyZVal = 2 * classy.rightStickY;                  //read standard right stick data (0-31), then double it to get to 6-bit range (0-62)
   joyGAVal = 2 * classy.rightStickX;                 //read standard right stick data (0-31), then double it to get to 6-bit range (0-62)
   joyGripperVal = 2 * classy.rightShoulderPressure;  //read standard shoulder data (0-31), then double it to get to 6-bit range (0-62)
   delay(delayTime);  //slow down the readings - remove
   
}

void setArmSpeeds()
{
  if(millis() - lastArmSpeedUpdate > speedUpdateInterval)
  {    
     if(classy.upDPressed)
     {
       spd = spd + spdMod;
       spd = constrain(spd, minSpd, maxSpd);
     } 
     if(classy.downDPressed)
     {
       spd = spd - spdMod;
       spd = constrain(spd, minSpd, maxSpd);
     } 
       
     if(classy.leftDPressed)
     {
       gripSpd = gripSpd - gripSpdMod;
       gripSpd = constrain(gripSpd, minGripSpd, maxGripSpd);
     } 
     if(classy.rightDPressed)
     {
       gripSpd = gripSpd + gripSpdMod;
       gripSpd = constrain(gripSpd, minGripSpd, maxGripSpd);
     }   
  
     lastArmSpeedUpdate = millis();
  }
    
   
   
}


void setGripper()
{
    
   if(classy.lzPressed || classy.leftShoulderPressed)
   {
     sGrip = 1500;
   } 
   if(classy.rzPressed || classy.rightShoulderPressed)
   {
     sGrip = 2300;
   } 
  
//   if(classy.leftShoulderPressed)
//   {
//    if(lastLButtonState == LOW)
//    {
//     sGrip = sGrip + 20; 
//    }
//    lastLButtonState = HIGH;
//   } 
//   else
//   {
//    lastLButtonState = LOW;
//   }
//
//   
//   if(classy.rightShoulderPressed)
//   {
//    if(lastRButtonState == LOW)
//    {
//     sGrip = sGrip - 20;
//    }
//    lastRButtonState = HIGH;
//   }
//   else
//   {
//     lastRButtonState = LOW;
//   }
//   
  
     sGrip = constrain(sGrip, GRIPPER_MIN, GRIPPER_MAX);
  
}


void processClassicController()
{
  
   classy.update();  //get updated controls from classic controller
   readClassicJoysticks();
   setArmSpeeds();
   setGripper();
}


//===================================================================================================
// ProcessAnalogInput3D: Process input from Analog Joysticks, convert into X,Y,Z coord
//===================================================================================================
boolean ProcessAnalogInput3D(void) {

   boolean fChanged = false;
   processClassicController();
   
   //only update the base joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
   {
     joyXMapped = mapfloat(joyXVal, WII_JOYSTICK_MAX, 0, -spd, spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sIKX -= joyXMapped;
   }

   //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
   {
     joyYMapped = mapfloat(joyYVal, WII_JOYSTICK_MAX, 0, -spd, spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sIKY -= joyYMapped;
   }

   //only update the elbow joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
   {
     joyZMapped = mapfloat(joyZVal, WII_JOYSTICK_MAX, 0, -spd, spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sIKZ -= joyZMapped;
   }
   
   //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
   {
     joyGAMapped = mapfloat(joyGAVal, WII_JOYSTICK_MAX, 0, -spd, spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sIKGA += joyGAMapped;
   }
     
   //Mapping analog joystick value to servo PWM signal range
  // joyGripperMapped = mapfloat(joyGripperVal, WII_JOYSTICK_MAX, 0, GRIPPER_MIN, GRIPPER_MAX);
   //sGrip = joyGripperMapped;//set servo position variable to the mapped value from the knob

    sIKX = min(max(sIKX, IK_MIN_X), IK_MAX_X);
    sIKY = min(max(sIKY, IK_MIN_Y), IK_MAX_Y);
    sIKZ = min(max(sIKZ, IK_MIN_Z), IK_MAX_Z);
    sIKGA = min(max(sIKGA, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...
    

  
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
   processClassicController();
   
   //only update the base joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
   {
     joyXMapped = mapfloat(joyXVal, WII_JOYSTICK_MAX, 0,  -spd * 10 , spd * 10); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sBase += joyXMapped;
   }

   //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
   {
     joyYMapped = mapfloat(joyYVal, WII_JOYSTICK_MAX, 0, -spd, spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sIKY -= joyYMapped;
   }

   //only update the elbow joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
   {
     joyZMapped = mapfloat(joyZVal, WII_JOYSTICK_MAX, 0, -spd, spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sIKZ -= joyZMapped;
   }
   
   //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
   {
     joyGAMapped = mapfloat(joyGAVal, WII_JOYSTICK_MAX, 0, -spd, spd); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sIKGA += joyGAMapped;
   }
   
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
   processClassicController();

   //only update the base joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
   {
     joyXMapped = mapfloat(joyXVal, WII_JOYSTICK_MAX, 0, -spd * 10, spd * 10); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sBase += joyXMapped; //add mapped base joystick value to present Base Value (positive values of joyBaseMapped will increase the position, negative values will decrease the position)
   }

   //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
   {
     joyYMapped = mapfloat(joyYVal, WII_JOYSTICK_MAX, 0, -spd * 10, spd * 10); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sShoulder -= joyYMapped; //add mapped shoulder joystick value to present Shoulder Value (positive values of joyShoulderMapped will increase the position, negative values will decrease the position)
   }

   //only update the elbow joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
   {
     joyZMapped = mapfloat(joyZVal, WII_JOYSTICK_MAX, 0, -spd * 10, spd * 10); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sElbow -= joyZMapped;//add mapped elbow joystick value to present elbow Value (positive values of joyElbowMapped will increase the position, negative values will decrease the position)
   }
   

   //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
   if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
   {
     joyGAMapped = mapfloat(joyGAVal, WII_JOYSTICK_MAX, 0, -spd * 10, spd * 10); //Map analog value from native joystick value (0 to WII_JOYSTICK_MAX) to incremental change (-spd to spd)
     sWrist -= joyGAMapped;//add mapped wrist joystick value to present wrist Value (positive values of joyWristMapped will increase the position, negative values will decrease the position)
   }
   
    sBase = min(max(sBase, BASE_MIN), BASE_MAX);
    sShoulder = min(max(sShoulder, SHOULDER_MIN), SHOULDER_MAX);
    sElbow = min(max(sElbow, ELBOW_MIN), ELBOW_MAX);
    sWrist = min(max(sWrist, WRIST_MIN), WRIST_MAX);
    
    //sGrip = min(max(sGrip, GRIPPER_MIN), GRIPPER_MAX);

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
