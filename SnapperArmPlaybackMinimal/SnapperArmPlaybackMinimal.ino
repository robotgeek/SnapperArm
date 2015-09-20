/***********************************************************************************
 *  }--\     RobotGeek Snapper Robotic Arm     /--{
 *      |       Analog IK Control Code        |
 *   __/                                       \__
 *  |__|                                       |__|
 *
 *
 *  This sketch can also be used to play  back a pre-programmed sequence that was generated 
 *  in ArmLink or the IK joystick control code. This code automatically runs the sequence, 
 *  giving you the minimal amount of code needed. 
 *
 *
 *  Snapper Arm Getting Started Guide
 *    http://learn.robotgeek.com/getting-started/33-robotgeek-snapper-robot-arm/63-robotgeek-snapper-arm-getting-started-guide.html
 *   Playback using Arm Link software
 *     http://learn.trossenrobotics.com/20-interbotix/robot-arms/143-arm-link-sequence-playback.html
 *    Using the IK Firmware
 *    http://learn.robotgeek.com/demo-code/demo-code/154-robotgeek-snapper-joystick-inverse-kinematics-demo.html
 *
 *
 *  WIRING
 *    Servos
 *      Digital I/O 3 - Base Rotation - Robot Geek Servo 
 *      Digital I/O 5 - Shoulder Joint - Robot Geek Servo 
 *      Digital I/O 6 - Elbow Joint - Robot Geek Servo 
 *      Digital I/O 9 - Wrist Joint - Robot Geek Servo 
 *      Digital I/O 10 - Gripper Servo -  Robot Geek Servo for parrallel gripper, 9g servo for 9g gripper
 *
 *
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
#define ROBOT_GEEK_9G_GRIPPER 1
#define ROBOT_GEEK_PARALLEL_GRIPPER 2

//The 9G gripper is the gripper with the small blue 9g servo
//The Parralle gripper has a full robotgeek servo and paralle rails
//Uncomment one of the following lines depending on which gripper you are using.
//#define GRIPPER_TYPE ROBOT_GEEK_9G_GRIPPER
//#define GRIPPER_TYPE ROBOT_GEEK_PARALLEL_GRIPPER

#ifndef GRIPPER_TYPE
   #error YOU HAVE TO SELECT THE GRIPPER YOU ARE USING! Uncomment the correct line above for your gripper
#endif


#include <ServoEx.h>

//kinematics file - local
#include "Kinematics.h"

ServoEx    ArmServo[5];

//===================================================================================================
// Setup 
//===================================================================================================
void setup(){
  // Attach servo and set limits
  ArmServo[BAS_SERVO].attach(3, BASE_MIN, BASE_MAX);
  ArmServo[SHL_SERVO].attach(5, SHOULDER_MIN, SHOULDER_MAX);
  ArmServo[ELB_SERVO].attach(6, ELBOW_MIN, ELBOW_MAX);
  ArmServo[WRI_SERVO].attach(9, WRIST_MIN, WRIST_MAX);
  ArmServo[GRI_SERVO].attach(10, GRIPPER_MIN, GRIPPER_MAX);



  // send arm to default X,Y,Z coord
  doArmIK(true, g_sIKX,g_sIKY,g_sIKZ,g_sIKGA);
  SetServo(sDeltaTime);

  // start serial
  Serial.begin(9600);
  Serial.println("Starting RobotGeek Analog IK Demo");
  delay(500);
    g_bIKMode = IKM_IK3D_CARTESIAN;
    
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 2
    //###########################################################// 
    IKSequencingControl(-80 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 3
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 4
    //###########################################################// 
    IKSequencingControl(0 , 200 , 150 , 0 , 0 , 256 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 5
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 6
    //###########################################################// 
    IKSequencingControl(0 , 150 , 225 , 0 , 0 , 256 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 7
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 8
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 30 , 0 , 256 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 9
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 256 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 10
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 512 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 11
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 0 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 12
    //###########################################################// 
    IKSequencingControl(100 , 200 , 200 , 0 , 0 , 512 , 4080 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 13
    //###########################################################// 
    IKSequencingControl(0 , 150 , 150 , 0 , 0 , 0 , 2000 , 0, 1);
    //###########################################################// 

    //###########################################################//
    // SEQUENCE 14
    //###########################################################// 
    IKSequencingControl(100 , 200 , 200 , 0 , 0 , 10 , 160 , 0, 1);
    //###########################################################// 
  
}


void loop()
{



  
  
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


