/***********************************************************************************
 *  }--\     RobotGeek Snapper Robotic Arm     /--{
 *      |       Analog IK Control Code        |
 *   __/                                       \__
 *  |__|                                       |__|
 *
 *
 *  The following sketch will move each joint of the arm based on analog inputs.
 *
 *  Snapper Arm Getting Started Guide
 *  http://learn.trossenrobotics.com/33-robotgeek-getting-started-guides/robotgeek-snapper-robot-arm/63-robotgeek-snapper-arm-getting-started-guide
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
 *    Turn the 
 *
 *
 *  NOTES
 *    ANALOG INPUT MAPPING
 *      This code uses a combination of direct and incremental code for converting 
 *      analog inputs into servo positions
 *    
 *      Direct/Absolute
 *        Absolute positioning is used for the knobs controlling the base and gripper servo.
 *        This means that the value of the knob is mapped directly to the corresponding servo
 *        value. This method is ideal for sensors that stay at static positions such as
 *        knobs and sliders. 
 *    
 *      Incremental
 *        Incremental code is used for the joysticks controlling the shoulder, elbow and
 *        gripper servo. Each joystick value is mapped to a small realtiveley small positive
 *        or negative value. This value is then added to the currrent position of the servo.
 *        The action of slowly moving the joystick away from its center position can slowly 
 *        move each joint of the robot. When the joystick is centered, no movement is made
 *     
 *      The choice for using Direct/Incremental mapping for each joint was made based
 *      on usability, however the code can be modified so that any joint can use
 *      either direct or incremental mapping
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
 * //  This code is a Work In Progress and is distributed in the hope that it will be useful,
 * //  but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * //  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
 * //  
 * //=============================================================================
 * 
 * Sources used:
 * http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
 * 
 * Application Note 44 - Controlling a Lynx6 Robotic Arm 
 * http://www.micromegacorp.com/appnotes.html
 * http://www.micromegacorp.com/downloads/documentation/AN044-Robotic%20Arm.pdf
 * 
 ***********************************************************************************/

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
  pinMode(BUTTON2, INPUT);      
  pinMode(11, OUTPUT);      

  // send arm to default X,Y,Z coord
  doArmIK(true, g_sIKX,g_sIKY,g_sIKZ,g_sIKGA);
  SetServo(sDeltaTime);

  // start serial
  Serial.begin(9600);
  Serial.println("Starting RobotGeek Analog IK Demo");
  delay(500);
  
  AnalogControlLoop();
}


void loop(){

  //use digitalRead to store the current state of the pushbutton in one of the 'buttonState' variables
  buttonState2 = digitalRead(BUTTON2);

  if (buttonState1 == HIGH) 
  {     
    //
    AnalogControlLoop();     
  } 
  if (buttonState2 == HIGH) 
  { 
    SequenceLoop(); 
  }

  
  int inByte = Serial.read();

  switch (inByte) {

  case '1':    
    AnalogControlLoop();
    break;    

  case '2':
    SequenceLoop(); 
    break;


  }

}



void MenuOptions(){

  Serial.println("###########################"); 
  Serial.println("Please enter option 1-2, or press Button 1 or 2 respectively"); 
  Serial.println("1) Start Analog IK Control Mode");        
  Serial.println("2) Start Preprogrammed Sequence Mode");     
  Serial.println("###########################"); 
}



void AnalogControlLoop(){
  Serial.println("Analog IK Control Mode Active.");
  Serial.println("Send '1' or press the 'Capture' pushbutton to pause the joysticks and capture the current pose."); 
  do
  {
    //Process analog input from ArmControl, translate to working X,Y,Z,GA Coord
    ProcessAnalogInput3D();
    //Calculate goal positions of servos based on X,Y,Z,GA coord determined by ProcessUserInput3D()
    doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 
    //Set servo positions via sDeltaTime interpolation value (set in UserInput as well)
    SetServo(0);
  } 
  while((Serial.available() == 0) && (buttonState1 == LOW)); 
  Serial.read(); // Read & discard the character that got us out of the loop.

  delay(100);
  Serial.println("");
  Serial.println("Exiting Analog IK Control Mode."); 
  Serial.println("");
  Serial.println("Current Arm Coordinate Values:");
  Serial.print("    X Axis: ");
  Serial.println(g_sIKX, 2);
  Serial.print("    Y Axis: ");
  Serial.println(g_sIKY, 2);
  Serial.print("    Z Axis: ");
  Serial.println(g_sIKZ, 2);
  Serial.print("    Wrist Angle: ");
  Serial.println(g_sIKGA, 2);
  Serial.print("    Gripper: ");
  Serial.println(Gripper, DEC);

  Serial.println("");
  Serial.println("Sequence Control Code");
  Serial.print("    IKSequencingControl(");
  Serial.print(g_sIKX, 2);
  Serial.print(", ");
  Serial.print(g_sIKY, 2);
  Serial.print(", ");
  Serial.print(g_sIKZ, 2);
  Serial.print(", ");
  Serial.print(g_sIKGA, 2);
  Serial.print(", ");
  Serial.print(Gripper, DEC);
  Serial.println(",2000,1000); ");
  Serial.println("");

  delay(500);
  MenuOptions();
}

void SequenceLoop(){
  Serial.println("Sequencing Mode Active."); 
  Serial.println("Send '1' or press Button 1 to pause and return to menu.");   
  do
  {
    //###########################################################//
    // SEQUENCE 1
    //###########################################################// 
    //IKSequencingControl(X-axis, Y-Axis, Z-axis, Wrist Angle, Gripper, Interpolation, Delay)
    Serial.println("Moving to Sequence Position 1");   
    IKSequencingControl(0, 150, 150, 0, 1500, 2000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 2
    //###########################################################//
    Serial.println("Moving to Sequence Position 2");  
    IKSequencingControl(-100, 150, 150, 0, 1500, 1000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 3
    //###########################################################//
    Serial.println("Moving to Sequence Position 3");  
    IKSequencingControl(0, 100, 100, 0, 1500, 3000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 4
    //###########################################################//
    Serial.println("Moving to Sequence Position 4");  
    IKSequencingControl(100, 150, 200, 0, 1500, 1000, 1000);
    //###########################################################//


    //###########################################################//
    // SEQUENCE 5
    //###########################################################//
    Serial.println("Moving to Sequence Position 5");  
    IKSequencingControl(0, 100, 100, -15, 1500, 2000, 1000);
    //###########################################################//

  } 
  while((Serial.available() == 0) && (buttonState1 == LOW)); 
  Serial.read(); // Read & discard the character that got us out of the loop.
  delay(100);
  Serial.println("Pausing Sequencing Mode."); 
  delay(500);
  MenuOptions();
}


void IKSequencingControl(float X, float Y, float Z, float GA, int grip, int interpolate, int pause){
  doArmIK(true, X, Y, Z, GA); 
  Gripper = grip;
  SetServo(interpolate);
  delay(interpolate + pause);
}


void stateChange(){
  buttonState1 = !buttonState1;
}



