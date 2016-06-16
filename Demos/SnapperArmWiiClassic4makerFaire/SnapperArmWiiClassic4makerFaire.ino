 /***********************************************************************************
 *  }--\     RobotGeek Snapper Robotic Arm     /--{
 *      |       Analog IK Control Code        |
 *   __/                                       \__
 *  |__|                                       |__|
 *
 *
 *  The following sketch will move allow you to contro the Snapper Robot Arm using
 *  joysticks and a knob. 
 *  
 *  By setting the control mode you can control the arm in joint/backhoe mode, 
 *  Cartesian IK or Cylindric
 * the arm to an X/Y/Z coordinate based on the inputs
 *  from the analog inputs (joysticks and knob). This sketch can also be used to play
 *  back a pre-programmed sequence.
 *
 *  Snapper Arm Getting Started Guide
 *   http://learn.robotgeek.com/getting-started/33-robotgeek-snapper-robot-arm/63-robotgeek-snapper-arm-getting-started-guide.html
 *  Using the IK Firmware
 *    http://learn.robotgeek.com/demo-code/demo-code/154-robotgeek-snapper-joystick-inverse-kinematics-demo.html
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
 
 *
 *  NOTES
 *
 *    SERVO POSITIONS
 *      The servos' positions will be tracked in microseconds, and written to the servos
 *      using .writeMicroseconds()
 *        http://arduino.cc/en/Reference/ServoWriteMicroseconds
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
 * Sources used:
 * https://github.com/KurtE
 * 
 * http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
 * 
 * Application Note 44 - Controlling a Lynx6 Robotic Arm 
 * http://www.micromegacorp.com/appnotes.html
 * http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf
 * 
 * 
 *   This code is a Work In Progress and is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 *   
 ***********************************************************************************/
//#define DEFAULT_CONTROL_MODE  IKM_BACKHOE
//#define DEFAULT_CONTROL_MODE  IKM_IK3D_CARTESIAN
#define DEFAULT_CONTROL_MODE  IKM_CYLINDRICAL

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

#include <WiiClassy.h>
#include <Wire.h>

#include <ServoEx.h>
#include "InputControl.h"
#include "armSequence1.h"
#include "armSequence2.h"
#include "armSequence3.h"
#include "armSequence4.h"



WiiClassy classy = WiiClassy();  //classic controller class
ServoEx    ArmServo[5];          //servo ex class

unsigned long lastReportTime;
int reportInterval = 1000;

  boolean fChanged = false;
  
//===================================================================================================
// Setup 
//===================================================================================================
void setup()
{
  attachServos();
  

  // start serial
  Serial.begin(9600);
  Serial.println(" ");
  Serial.println("Starting RobotGeek Analog IK Demo");
  delay(500);
  
  //AnalogControlLoop();
  
  
Serial.print("Starting Controller");
delay(100);
classy.init();  //start wii classic controller connection
Serial.print("Controller Started");
delay(100);
classy.update();  //get updated controls from classic controller


 // boolean fChanged = false;
  delay(500);
  
  Serial.println("Analog Control Mode Active.");
  Serial.print("Control Mode:");
  if(g_bIKMode == IKM_IK3D_CARTESIAN)
  {
    Serial.println("Cartesian IK Mode");
  } 
  else if(g_bIKMode == IKM_CYLINDRICAL)
  {
    Serial.println("Cylindrical IK Mode");
  }
  else if(g_bIKMode == IKM_BACKHOE)
  {
    Serial.println("Backhoe / Joint Control Mode");

  }
  
  
  
}


void loop()
{  
  if(millis() - lastReportTime > reportInterval)
  { 
     
    Serial.print("Control Mode:");
    if(g_bIKMode == IKM_IK3D_CARTESIAN)
    {
      Serial.print("Cartesian IK ");
      Serial.print("|Coordinates:");
      Serial.print(g_sIKX);
      Serial.print(",");
      Serial.print(g_sIKY);
      Serial.print(",");
      Serial.print(g_sIKZ);
      Serial.print(",");
      Serial.print(g_sIKGA);
      Serial.print(",");
      Serial.print(g_sGrip);



    } 
    else if(g_bIKMode == IKM_CYLINDRICAL)
    {
      Serial.print("Cylindrical IK ");
      Serial.print("|Coordinates:");
      Serial.print(g_sBase);
      Serial.print(",");
      Serial.print(g_sIKY);
      Serial.print(",");
      Serial.print(g_sIKZ);
      Serial.print(",");
      Serial.print(g_sIKGA);
      Serial.print(",");
      Serial.print(g_sGrip);
    }
    else if(g_bIKMode == IKM_BACKHOE)
    {
      Serial.print("Backhoe ");
      Serial.print("|Coordinates:");
      Serial.print(g_sBase);
      Serial.print(",");
      Serial.print(g_sShoulder);
      Serial.print(",");
      Serial.print(g_sElbow);
      Serial.print(",");
      Serial.print(g_sWrist);
      Serial.print(",");
      Serial.print(g_sGrip);

  
    }
    
    //Serial.print()
    Serial.print("|Arm Speed:");
    Serial.print(spd);
    Serial.print("|Gripper Speed:");
    Serial.print(gripSpd);
  
     Serial.println();
     
    lastReportTime = millis();
  }
  
  
  //Serial.println("loop."); 


  if(classy.xPressed == true)
  {
      digitalWrite(2, HIGH);
      digitalWrite(4, LOW);
      digitalWrite(7, LOW);
      digitalWrite(8, LOW);
  }
  if(classy.yPressed == true)
  {
      digitalWrite(2, LOW);
      digitalWrite(4, HIGH);
      digitalWrite(7, LOW);
      digitalWrite(8, LOW);
  }
  if(classy.aPressed == true)
  {
      digitalWrite(2, LOW);
      digitalWrite(4, LOW);
      digitalWrite(7, HIGH);
      digitalWrite(8, LOW);
  }
  
  if(classy.bPressed == true)
  {
      digitalWrite(2, LOW);
      digitalWrite(4, LOW);
      digitalWrite(7, LOW);
      digitalWrite(8, HIGH);
  }




//  if(classy.xPressed == true)
//  {
//    g_bIKMode = IKM_IK3D_CARTESIAN;
//    attachServos();
//  }
//  if(classy.yPressed == true)
//  {
//    g_bIKMode = IKM_CYLINDRICAL;
//    attachServos();
//  }
//  if(classy.aPressed == true)
//  {
//    g_bIKMode = IKM_BACKHOE;
//    attachServos();
//  }
//  
//  if(classy.bPressed == true)
//  {
//      playSequence();//play sequence once
//      attachServos();//return to home  position for current mode
//  }

//  if(classy.xPressed == true)
//  {
//      playSequence4();//play sequence once
//      attachServos();//return to home  position for current mode
//      //g_bIKMode = IKM_CYLINDRICAL;
//  }
//  if(classy.yPressed == true)
//  {
//      playSequence2();//play sequence once
//      attachServos();//return to home  position for current mode
//      //g_bIKMode = IKM_CYLINDRICAL;
//  }
//  if(classy.aPressed == true)
//  {
//      playSequence3();//play sequence once
//      attachServos();//return to home  position for current mode
//      //g_bIKMode = IKM_CYLINDRICAL;
//  }
//  
//  if(classy.bPressed == true)
//  {
//      playSequence1();//play sequence once
//      attachServos();//return to home  position for current mode
//      //g_bIKMode = IKM_CYLINDRICAL;
//  }
//
//  if(classy.selectPressed == true)
//  {
//    detachServos();
//  }
//
//  if(classy.startPressed == true)
//  {
//    attachServos();
//  }


     switch (g_bIKMode) {
        case IKM_IK3D_CARTESIAN:
          fChanged |= ProcessAnalogInput3D();

          break;

        case IKM_CYLINDRICAL:
          fChanged |= ProcessAnalogInputCylindrical();   

          break;

        case IKM_BACKHOE:
          fChanged |= ProcessAnalogBackhoe();
          break;
        }
     
        MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, sDeltaTime, true);
        SetServo(0);
     
     
  
}



void attachServos()
{
  
  ArmServo[BAS_SERVO].attach(3, BASE_MIN, BASE_MAX);
  ArmServo[SHL_SERVO].attach(5, SHOULDER_MIN, SHOULDER_MAX);
  ArmServo[ELB_SERVO].attach(6, ELBOW_MIN, ELBOW_MAX);
  ArmServo[WRI_SERVO].attach(9, WRIST_MIN, WRIST_MAX);
  ArmServo[GRI_SERVO].attach(10, GRIPPER_MIN, GRIPPER_MAX);
  
  
  if(g_bIKMode == IKM_IK3D_CARTESIAN)
  {
    g_sIKX  =0.00;    
    g_sBase = 1500 ; 
    g_sIKY  =150.00; 
    g_sIKZ  =150.00;
    g_sIKGA =0.00;
    sIKX  =0.00;    
    sBase = 1500 ; 
    sIKY  =150.00; 
    sIKZ  =150.00;
    sIKGA =0.00;
   
    doArmIK(true, g_sIKX,g_sIKY,g_sIKZ,g_sIKGA);
  } 
  else if(g_bIKMode == IKM_CYLINDRICAL)
  {

    sBase = 1500;
    g_sBase = 1500 ; 
    g_sIKY  =150.00; 
    g_sIKZ  =150.00;
    g_sIKGA =0.00; 
    
    sBase = 1500 ; 
    sIKY  =150.00; 
    sIKZ  =150.00;
    sIKGA =0.00; 
    
    doArmIK(false, g_sBase,g_sIKY,g_sIKZ,g_sIKGA);
  }
  else if(g_bIKMode == IKM_BACKHOE)
  {
    sBase = 1500;

    sShoulder = 1500;

    sElbow = 1500;

    sWrist = 1500;
    
  }
  MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, sDeltaTime, true);
  SetServo(1000);
  delay(2000); 
  
  Serial.println("Servos Activated!");
 

}

void detachServos()
{
 
  ArmServo[BAS_SERVO].detach();
  ArmServo[SHL_SERVO].detach();
  ArmServo[ELB_SERVO].detach();
  ArmServo[WRI_SERVO].detach();
  ArmServo[GRI_SERVO].detach();
  Serial.println("Servos Deactivated!");
 }


/***********************************************************
 * IKSequencingControl()
 *    Function used to set parameters for the Arm
 *
 * The following variables are named for Cartesian mode -
 * however the data that will be held/sent will vary based on the current IK mode
 ****************************************************************************
 * Variable name | Cartesian Mode | Cylindrcal Mode | Backhoe Mode          |
 *_______________|________________|_________________|_______________________|
 *   x           |   x            |   base          |   base joint          |
 *   y           |   y            |   y             |   shoulder joint      |
 *   z           |   z            |   z             |   elbow joint         |
 *   GA          |  wristAngle    |  wristAngle     |   wrist angle joint   |
 *   gripper     |  gripper       |  gripper        |   gripper joint       |
 *
 * interpolate - the amount of time to complete a pose
 * pause - time ti pause after a pose is completed
 * enable - setting this to '1' makes the function work. Setting it to '0' bypasses the function. This can be usefull for breaking out of sequences
 *
 *
 **********************************************************/
 
void IKSequencingControl(float X, float Y, float Z, float GA, int grip, int interpolate, int pause, int enable ){
  if(enable == 1)
  {
    
    if(g_bIKMode == IKM_IK3D_CARTESIAN || g_bIKMode == IKM_IK3D_CARTESIAN_90)
    {
      
      doArmIK(true, X, Y, Z, GA); 
      
    }
    else if(g_bIKMode == IKM_CYLINDRICAL || g_bIKMode ==IKM_CYLINDRICAL_90)
    {  
      sBase = X;
      doArmIK(false, X, Y, Z, GA); 
      
    }
    else if(g_bIKMode == IKM_BACKHOE)
    {
      sBase = X;
      sShoulder = Y;
      sElbow = Z;
      sWrist = GA;
      
    }
    
    
    
    sGrip = grip;
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, sDeltaTime, true);
    SetServo(interpolate);
    delay(interpolate + pause);
  }
}

//overloaded function to accout for extra empty wrist rotate packet
void IKSequencingControl(float X, float Y, float Z, float GA, float WR, int grip, int interpolate, int pause, int enable ){
   IKSequencingControl( X, Y, Z, GA,  grip,interpolate,  pause,  enable );
}


void stateChange()
{
  if(playState1 == 1)
    {
      playState1 = 0;
    }
  else if(playState2 == 1)
    {
      playState2 = 0;
    }
  else if(playState3 == 1)
    {
      playState3 = 0;
    }
  else if(playState4 == 1)
    {
      playState4 = 0;
    }
    
}
