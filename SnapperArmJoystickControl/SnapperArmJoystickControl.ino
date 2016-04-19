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
 *    Analog Inputs
 *      Analog 0 - Joystick (Horizontal)
 *      Analog 1 - Joystick (Vertical)
 *      Analog 2 - Joystick (Vertical)
 *      Analog 3 - Joystick (Vertical)
 *      Analog 4 - Rotation Knob 
 *      
 *    Digital Inputs
 *      Digital 2 - Button 1
 *      Digital 4 - Button 2
 *
 *  
 *    Use an external power supply and set both PWM jumpers to 'VIN'
 *
 *  CONTROL
 *      Analog 0 - Joystick - Control the Y Axis (forward/back)
 *      Analog 1 - Joystick - Control the X Axis (left/right)
 *      Analog 2 - Joystick - Control the Z Axis (up/down)
 *      Analog 3 - Joystick - Control the Wrist Angle
 *      Analog 4 - Rotation Knob - Control the Gripper
 *    http://learn.robotgeek.com/demo-code/demo-code/154-robotgeek-snapper-joystick-inverse-kinematics-demo.html
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
#define DEFAULT_CONTROL_MODE  IKM_BACKHOE
//#define DEFAULT_CONTROL_MODE  IKM_IK3D_CARTESIAN
//#define DEFAULT_CONTROL_MODE  IKM_CYLINDRICAL
  


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
#include "InputControl.h"

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

  // initialize pin 2 on interupt 0
  attachInterrupt(0, stateChange, CHANGE);  
  // initialize the pins for the pushbutton as inputs:  


  pinMode(BUTTON1, INPUT);     
  pinMode(BUTTON2, INPUT);       

  g_bIKMode = DEFAULT_CONTROL_MODE;

  if(digitalRead(BUTTON1) == HIGH)
  {
    g_bIKMode = IKM_CYLINDRICAL;
  }

  if(digitalRead(BUTTON2) == HIGH)
  {
    g_bIKMode = IKM_IK3D_CARTESIAN;
  }


  while(digitalRead(BUTTON2) == HIGH || digitalRead(BUTTON1) == HIGH)
  {
    //wait for both pushbuttons to be low/innactive by doing nothing
  }




  if(g_bIKMode == IKM_IK3D_CARTESIAN)
  {
    doArmIK(true, g_sIKX,g_sIKY,g_sIKZ,g_sIKGA);
  } 
  else if(g_bIKMode == IKM_CYLINDRICAL)
  {
    sBase = 1500;
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
  SetServo(0);

  






  // start serial
  Serial.begin(9600);
  Serial.println(" ");
  Serial.println("Starting RobotGeek Analog IK Demo");
  delay(500);
  
  AnalogControlLoop();

}


void loop(){

  //use digitalRead to store the current state of the pushbutton in one of the 'buttonState' variables
  buttonState1 = digitalRead(BUTTON1);
  buttonState2 = digitalRead(BUTTON2);

  if (buttonState1 == HIGH) 
  {     
    AnalogControlLoop();     
  } 
//  if (buttonState2 == HIGH) 
//  { 
//    SequenceLoop(); 
//  }

  int inByte = Serial.read();

  switch (inByte) {

  case '1':    
    AnalogControlLoop();
    break;    
//  case '2':    
//    SequenceLoop(); 
//    break;    



  }
}



void MenuOptions(){

  Serial.println("###########################"); 
  Serial.println("Please enter option 1-2, or press Button 1 or 2 respectively"); 
  Serial.println("1) Start Analog IK Control Mode");        
//  Serial.println("2) Start Preprogrammed Sequence Mode");     
  Serial.println("###########################"); 
}



void AnalogControlLoop(){
  boolean fChanged = false;
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
  
  
  Serial.println("Send '1' or press the 'Capture' pushbutton to pause the joysticks and capture the current pose."); 
  loopbreak = LOW;  
  do
  {



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
  while((Serial.available() == 0) && (loopbreak == LOW)); 
  Serial.read(); // Read & discard the character that got us out of the loop.

  delay(100);
  Serial.println("");
  Serial.println("Exiting Analog IK Control Mode."); 
  Serial.println("");
  Serial.println("Current Arm Coordinate Values:");
  if(g_bIKMode == IKM_IK3D_CARTESIAN)
  {
    Serial.print("    X Axis: ");
    Serial.println(sIKX, 2);

  }
  else 
  {
    Serial.print("    Base Servo: ");
    Serial.println(sBase);

  }
  if(g_bIKMode == IKM_BACKHOE)
  {
    Serial.print("    Shoulder Servo: ");
    Serial.println(sShoulder);
    Serial.print("    Elbow Servo ");
    Serial.println(sElbow);
    Serial.print("    Wrist Servo: ");
    Serial.println(sWrist);

  }
  else
  { 
    Serial.print("    Y Axis: ");
    Serial.println(sIKY, 2);
    Serial.print("    Z Axis: ");
    Serial.println(sIKZ, 2);
    Serial.print("    Wrist Angle: ");
    Serial.println(sIKGA, 2);

  }
 
  Serial.print("    Gripper: ");
  Serial.println(Gripper, DEC);

  Serial.println("");
  Serial.println("Sequence Control Code");
    

  Serial.print("    g_bIKMode = ");
  if(g_bIKMode == IKM_IK3D_CARTESIAN)
  {
    Serial.print("IKM_IK3D_CARTESIAN");
  } 
  else if(g_bIKMode == IKM_CYLINDRICAL)
  {
    Serial.print("IKM_CYLINDRICAL");
  }
  else if(g_bIKMode == IKM_BACKHOE)
  {
    Serial.print("IKM_BACKHOE");

  }

  Serial.println(";");
  Serial.print("    IKSequencingControl(");

  if(g_bIKMode == IKM_IK3D_CARTESIAN)
  {

    Serial.print(sIKX, 2);
    Serial.print(", ");

  }
  else 
  {
    
    Serial.print(sBase);
    Serial.print(", ");

  }
  if(g_bIKMode == IKM_BACKHOE)
  {
      Serial.print(sShoulder);
      Serial.print(", ");
      Serial.print(sElbow);
      Serial.print(", ");
      Serial.print(sWrist);
      Serial.print(", ");


  }
  else
  { 
      Serial.print(sIKY, 2);
      Serial.print(", ");
      Serial.print(sIKZ, 2);
      Serial.print(", ");
      Serial.print(sIKGA, 2);
      Serial.print(", ");

  }
 



  Serial.print("0 , ");//wrist rotate is always 0
  Serial.print(Gripper, DEC);
  Serial.println(",2000,1000, 1); ");
  Serial.println("");

  delay(500);
  MenuOptions();
}




void stateChange(){
  loopbreak = HIGH;
}

