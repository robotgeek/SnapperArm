/***********************************************************************************
    }--\     RobotGeek Snapper Robotic Arm     /--{
        |       Analog IK Control Code        |
     __/                                       \__
    |__|                                       |__|


    The following sketch will move allow you to contro the Snapper Robot Arm using
    joysticks and a knob.

    By setting the control mode you can control the arm in joint/backhoe mode,
    Cartesian IK or Cylindric
   the arm to an X/Y/Z coordinate based on the inputs
    from the analog inputs (joysticks and knob). This sketch can also be used to play
    back a pre-programmed sequence.

    Snapper Arm Getting Started Guide
     http://learn.robotgeek.com/getting-started/33-robotgeek-snapper-robot-arm/63-robotgeek-snapper-arm-getting-started-guide.html
    Using the IK Firmware
      http://learn.robotgeek.com/demo-code/demo-code/154-robotgeek-snapper-joystick-inverse-kinematics-demo.html


    WIRING
      Servos
        Digital I/O 3 - Base Rotation - Robot Geek Servo
        Digital I/O 5 - Shoulder Joint - Robot Geek Servo
        Digital I/O 6 - Elbow Joint - Robot Geek Servo
        Digital I/O 9 - Wrist Joint - Robot Geek Servo
        Digital I/O 10 - Gripper Servo - 9g Servo

      Analog Inputs
        Analog 0 - Joystick (Horizontal)
        Analog 1 - Joystick (Vertical)
        Analog 2 - Joystick (Vertical)
        Analog 3 - Joystick (Vertical)
        Analog 4 - Rotation Knob

      Digital Inputs
        Digital 2 - Button 1
        Digital 4 - Button 2


      Use an external power supply and set both PWM jumpers to 'VIN'

    CONTROL
        Analog 0 - Joystick - Control the Y Axis (forward/back)
        Analog 1 - Joystick - Control the X Axis (left/right)
        Analog 2 - Joystick - Control the Z Axis (up/down)
        Analog 3 - Joystick - Control the Wrist Angle
        Analog 4 - Rotation Knob - Control the Gripper
      http://learn.robotgeek.com/demo-code/demo-code/154-robotgeek-snapper-joystick-inverse-kinematics-demo.html



    NOTES

      SERVO POSITIONS
        The servos' positions will be tracked in microseconds, and written to the servos
        using .writeMicroseconds()
          http://arduino.cc/en/Reference/ServoWriteMicroseconds
        For RobotGeek servos, 600ms corresponds to fully counter-clockwise while
        2400ms corresponds to fully clock-wise. 1500ms represents the servo being centered

        For the 9g servo, 900ms corresponds to fully counter-clockwise while
        2100ms corresponds to fully clock-wise. 1500ms represents the servo being centered


    This code is a Work In Progress and is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
   Sources used:
   https://github.com/KurtE

   http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino

   Application Note 44 - Controlling a Lynx6 Robotic Arm
   http://www.micromegacorp.com/appnotes.html
   http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf


     This code is a Work In Progress and is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
     FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.

 ***********************************************************************************/
//#define DEFAULT_CONTROL_MODE  IKM_BACKHOE
#define DEFAULT_CONTROL_MODE  IKM_IK3D_CARTESIAN
//#define DEFAULT_CONTROL_MODE  IKM_CYLINDRICAL


#define ARBOTIX_TO  1000      // if no message for a second probably turned off...
#define DEADZONE    3        // deadzone around center of joystick values

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

#include <ServoEx.h>
#include <Commander.h>
#include "InputControl.h"

ServoEx    ArmServo[5];
extern Commander command = Commander();

//========
// Message informatino
unsigned long   ulLastMsgTime;          // Keep track of when the last message arrived to see if controller off
byte            buttonsPrev;            // will use when we wish to only process a button press once

 int speedMod = 10;

unsigned long lastUpdateTime;
int updateInterval = 33;

//===========================================================================================
// Setup
//===================================================================================================
void setup() {
  // Attach servo and set limits
  ArmServo[BAS_SERVO].attach(3, BASE_MIN, BASE_MAX);
  ArmServo[SHL_SERVO].attach(5, SHOULDER_MIN, SHOULDER_MAX);
  ArmServo[ELB_SERVO].attach(6, ELBOW_MIN, ELBOW_MAX);
  ArmServo[WRI_SERVO].attach(9, WRIST_MIN, WRIST_MAX);
  ArmServo[GRI_SERVO].attach(10, GRIPPER_MIN, GRIPPER_MAX);




  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);

  g_bIKMode = DEFAULT_CONTROL_MODE;





  if (g_bIKMode == IKM_IK3D_CARTESIAN)
  {
    doArmIK(true, g_sIKX, g_sIKY, g_sIKZ, g_sIKGA);
  }
  else if (g_bIKMode == IKM_CYLINDRICAL)
  {
    sBase = 1500;
    doArmIK(false, g_sBase, g_sIKY, g_sIKZ, g_sIKGA);
  }
  else if (g_bIKMode == IKM_BACKHOE)
  {
    sBase = 1500;

    sShoulder = 1500;

    sElbow = 1500;

    sWrist = 1500;

  }
  MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, sDeltaTime, true);
  SetServo(0);







  command.begin(38400);

  // start serial
  //Serial.begin(9600);
  Serial.println(" ");
  Serial.println("Starting RobotGeek Snapper Commander IK Demo");
  delay(500);


}


void loop()
{
  boolean fChanged = false;
  if (command.ReadMsgs())
  {
    digitalWrite(13, HIGH - digitalRead(13));
    // See if the Arm is active yet...
    if ((command.buttons & BUT_R1) == (0xff & BUT_R1) )
    {
      if((buttonsPrev & BUT_R1) != (0xff & BUT_R1))
      {
                g_bIKMode = IKM_IK3D_CARTESIAN;//xyz
                // For now lets always move arm to the home position of the new input method...
                // Later maybe we will get the current position and covert to the coordinate system
                // of the current input method.
                MoveArmToHome();
                  
      }


    }
    else if ((command.buttons & BUT_R2) == (0xff & BUT_R2))
    {
      if(!(buttonsPrev & BUT_R2)!= (0xff & BUT_R2))
      {
                g_bIKMode = IKM_CYLINDRICAL;//cylindrical
                // For now lets always move arm to the home position of the new input method...
                // Later maybe we will get the current position and covert to the coordinate system
                // of the current input method.
                MoveArmToHome();
                  
      }
    }
    else if ((command.buttons & BUT_R3) == (0xff & BUT_R3) )
    {
      if(!(buttonsPrev & BUT_R3) != (0xff & BUT_R3))
      {
                g_bIKMode = IKM_BACKHOE;//backhoe
                // For now lets always move arm to the home position of the new input method...
                // Later maybe we will get the current position and covert to the coordinate system
                // of the current input method.
                MoveArmToHome();
                  
      }
    }

//    // Going to use L6 in combination with the right joystick to control both the gripper and the
//    // wrist rotate...
//    else if (command.buttons & BUT_L4)
//    {
//      speedMod = 1;
//    }
//
//    else if (command.buttons & BUT_L5)
//    {
//      speedMod = 10;
//    }
//
//    else if (command.buttons & BUT_L6)
//    {
//      speedMod = 10;
//    }

    else
    {
      switch (g_bIKMode)
      {
        case IKM_IK3D_CARTESIAN:
          fChanged |= ProcessUserInput3D();
          break;
        case IKM_CYLINDRICAL:
          fChanged |= ProcessUserInputCylindrical();
          break;

        case IKM_BACKHOE:
          fChanged |= ProcessUserInputBackHoe();
          break;
      }
    }




     if (command.buttons & BUT_RT)
    {
      digitalWrite(2, HIGH);
    }
    else
    {
      
      digitalWrite(2, LOW);
    }


     if (command.buttons & BUT_LT)
    {
      digitalWrite(4, HIGH);
    }
    else
    {
      
      digitalWrite(4, LOW);
    }


    // else if (bioloid.interpolating > 0) {
    //   bioloid.interpolateStep();
    // }


    buttonsPrev = command.buttons;
    ulLastMsgTime = millis();    // remember when we last got a message...
  }
  else
  {
  }


  //update the servos regularly with the proper pan and tilt increments
  if (millis() - lastUpdateTime > updateInterval)
  {
    updateServoPositions();
    MoveArmTo(sBase, sShoulder, sElbow, sWrist, sWristRot, sGrip, sDeltaTime, true);
    SetServo(0);

    lastUpdateTime = millis();
  }



}



void MenuOptions() {

  Serial.println("###########################");
  Serial.println("Please enter option 1-2, or press Button 1 or 2 respectively");
  Serial.println("1) Start Analog IK Control Mode");
  //  Serial.println("2) Start Preprogrammed Sequence Mode");
  Serial.println("###########################");
}


