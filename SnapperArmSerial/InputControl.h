#ifndef INPUTCONTROL_H
#define INPUTCONTROL_H

#include "Kinematics.h"
#include "GlobalArm.h"

extern ArmControl armcontrol;
extern ServoEx    ArmServo[5];

extern void IDPacket(void);
extern void ReportAnalog(unsigned char, unsigned int);

//=============================================================================
// Global Variables...
//=============================================================================
boolean  g_fArmActive = false;   // Is the arm logically on?
byte  g_bIKMode = IKM_IK3D_CARTESIAN;   // Which mode of operation are we in...
uint8_t  g_bIKStatus = IKS_SUCCESS;   // Status of last call to DoArmIK;

//===================================================================================================
// Check EXT packet to determine action
//===================================================================================================
   void ExtArmState(){
       if(armcontrol.ext < 0x10){
        // no action
        g_fArmActive = true;
     }
      else if(armcontrol.ext == 0x20){  //32
        g_bIKMode = IKM_IK3D_CARTESIAN;
//        MoveArmToHome(); 
        IDPacket();
      }
      else if(armcontrol.ext == 0x28){  //40
        // no action
        g_fArmActive = true;
      }        
      else if(armcontrol.ext == 0x30){  //48
        g_bIKMode = IKM_CYLINDRICAL;
//        MoveArmToHome(); 
        IDPacket();        
      }
      else if(armcontrol.ext == 0x38){  //56
    
      }        
      else if(armcontrol.ext == 0x48){  //72
      // do something
      }

      else if(armcontrol.ext == 0x70){  //112
        IDPacket();
      }
      else if(armcontrol.ext == 0x80){  //128
        //IK value response
      }
      else if(armcontrol.ext >= 0xC8){  //200
        // read analogs
        ReportAnalog(armcontrol.ext, analogRead(armcontrol.ext - 0xC8));
      }
    }




////===================================================================================================
//// ProcessAnalogInput3D: Process input from Analog Joysticks, convert into X,Y,Z coord
////===================================================================================================
//boolean ProcessAnalogInput3D(void) {
//
//    boolean fChanged = false;
//    
//    //read analog values from analog sensors
//   joyXVal = analogRead(ANALOGX);
//   joyYVal = analogRead(ANALOGY);
//   joyZVal = analogRead(ANALOGZ);
//   joyGAVal = analogRead(ANALOGGA);
//   joyGripperVal = analogRead(ANALOGGRIP);
//   delay(5);
//
//
//   //only update the base joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
//   if(joyXVal > DEADBANDHIGH || joyXVal < DEADBANDLOW)
//   {
//     joyXMapped = mapfloat(joyXVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
//     g_sIKX -= joyXMapped;
//   }
//
//   //only update the shoulder joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
//   if(joyYVal > DEADBANDHIGH || joyYVal < DEADBANDLOW)
//   {
//     joyYMapped = mapfloat(joyYVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
//     g_sIKY -= joyYMapped;
//   }
//
//   //only update the elbow joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
//   if(joyZVal > DEADBANDHIGH || joyZVal < DEADBANDLOW)
//   {
//     joyZMapped = mapfloat(joyZVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
//     g_sIKZ -= joyZMapped;
//   }
//   
//   //only update the wrist joint if the joystick is outside the deadzone (i.e. moved oustide the center position)
//   if(joyGAVal > DEADBANDHIGH || joyGAVal < DEADBANDLOW)
//   {
//     joyGAMapped = mapfloat(joyGAVal, 0, 1023, -spd, spd); //Map analog value from native joystick value (0 to 1023) to incremental change (-spd to spd)
//     g_sIKGA += joyGAMapped;
//   }
//     
//   //Mapping analog joystick value to servo PWM signal range
//   joyGripperMapped = mapfloat(joyGripperVal, 0, 1023, GRIPPER_MIN, GRIPPER_MAX);
//   Gripper = joyGripperMapped;//set servo position variable to the mapped value from the knob
//
//   
////     
//    sIKX = min(max(g_sIKX, IK_MIN_X), IK_MAX_X);
//    sIKY = min(max(g_sIKY, IK_MIN_Y), IK_MAX_Y);
//    sIKZ = min(max(g_sIKZ, IK_MIN_Z), IK_MAX_Z);
//    sIKGA = min(max(g_sIKGA, IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...
//    
//      // remember our current IK position
//    g_sIKX = sIKX; 
//    g_sIKY = sIKY;
//    g_sIKZ = sIKZ;
//    g_sIKGA = sIKGA;
//
//  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
//  // condition, don't allow the arm to move farther away...
////  else {
////    // In an Error/warning condition, only allow things to move in closer...
////    sIKX = g_sIKX;
////    sIKY = g_sIKY;
////    sIKZ = g_sIKZ;
////    sIKGA = g_sIKGA;
////
////    if (((g_sIKX > 0) && (command.walkH < 0)) || ((g_sIKX < 0) && (command.walkH > 0)))
////      sIKX = min(max(g_sIKX + command.walkH/10, IK_MIN_X), IK_MAX_X);
////    if (((g_sIKY > 0) && (command.walkV < 0)) || ((g_sIKY < 0) && (command.walkV > 0)))
////      sIKY = min(max(g_sIKY + command.walkV/10, IK_MIN_Y), IK_MAX_Y);
////    if (((g_sIKZ > 0) && (command.lookV < 0)) || ((g_sIKZ < 0) && (command.lookV > 0)))
////      sIKZ = min(max(g_sIKZ + command.lookV/15, IK_MIN_Z), IK_MAX_Z);
////    if (((g_sIKGA > 0) && (command.lookH < 0)) || ((g_sIKGA < 0) && (command.lookH > 0))) 
////      sIKGA = min(max(g_sIKGA, IK_MIN_GA), IK_MAX_GA);            
////  }
//
//  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA);
//
//  return fChanged;
//
//}





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
  boolean fChanged = false; 

  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
  // condition, don't allow the arm to move farther away...

//  if (g_bIKStatus == IKS_SUCCESS) {
    
// Keep IK values within limits
//
    sIKX = min(max((armcontrol.Xaxis-X_OFFSET), IK_MIN_X), IK_MAX_X);  
    sIKY = min(max(armcontrol.Yaxis, IK_MIN_Y), IK_MAX_Y);    
    sIKZ = min(max(armcontrol.Zaxis, IK_MIN_Z), IK_MAX_Z);
    sIKGA = min(max((armcontrol.W_ang-GA_OFFSET), IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords..
    Gripper = min(max(armcontrol.Grip, GRIPPER_MIN), GRIPPER_MAX);
    sDeltaTime = armcontrol.dtime*16;
    
//  }

//  fChanged = (sIKX != g_sIKX) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sGrip != g_sGrip);  
  
//  if (fChanged) {
    // report
  doArmIK(true, sIKX, sIKY, sIKZ, sIKGA); 
//  }
//  return fChanged;

}



////===================================================================================================
//// ProcessUserInputCylindrical: Process the Userinput when we are in 3d Mode
////===================================================================================================
//boolean ProcessUserInputCylindrical() {
//  // We Are in IK mode, so figure out what position we would like the Arm to move to.
//  // We have the Coordinates system like:
//  //
//  //                y   Z
//  //                |  /
//  //                |/
//  //            ----+----X (X and Y are flat on ground, Z is up in air...
//  //                |
//  //                |
//  //
//  boolean fChanged = false;
//
//  // Will try combination of the other two modes.  Will see if I need to do the Limits on the IK values
//  // or simply use the information from the Warning/Error from last call to the IK function...
//  sIKY = g_sIKY;
//  sIKZ = g_sIKZ;
//  sIKGA = g_sIKGA;
//
//  // The base rotate is real simple, just allow it to rotate in the min/max range...
//  Base = min(max(armcontrol.Xaxis, BASE_MIN), BASE_MAX);
//
//  // Limit how far we can go by checking the status of the last move.  If we are in a warning or error
//  // condition, don't allow the arm to move farther away...
//  // Use Y for 2d distance from base
//  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKY > 0) && (armcontrol.Yaxis < 0)) || ((g_sIKY < 0) && (armcontrol.Yaxis > 0)))
//    sIKY = min(max(armcontrol.Yaxis, IK_MIN_Y), IK_MAX_Y);
//
//  // Now Z coordinate...
//  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKZ > 0) && (armcontrol.Zaxis < 0)) || ((g_sIKZ < 0) && (armcontrol.Zaxis > 0)))
//    sIKZ = min(max(armcontrol.Zaxis, IK_MIN_Z), IK_MAX_Z);
//
//  // And gripper angle.  May leave in Min/Max here for other reasons...   
//
//  if ((g_bIKStatus == IKS_SUCCESS) || ((g_sIKGA > 0) && (armcontrol.W_ang < 0)) || ((g_sIKGA < 0) && (armcontrol.W_ang > 0)))
//    sIKGA = min(max((armcontrol.W_ang-GA_OFFSET), IK_MIN_GA), IK_MAX_GA);  // Currently in Servo coords...
//
//    Gripper = min(max(armcontrol.Grip, GRIPPER_MIN), GRIPPER_MAX);
////    sDeltaTime = armcontrol.dtime*16;
//   
//  fChanged = (sBase != g_sBase) || (sIKY != g_sIKY) || (sIKZ != g_sIKZ) || (sIKGA != g_sIKGA) || (sGrip != g_sGrip);
//
//
//  if (fChanged) {
//    g_bIKStatus = doArmIK(false, sBase, sIKY, sIKZ, sIKGA, sGrip);
//  }
//  return fChanged;
//}


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

void ReportAnalog(unsigned char command, unsigned int value)
{
  unsigned char AH;
  unsigned char AL;
  AH = (value & 0xFF00);
  AL = (value & 0x00FF);
  Serial.write(0xff);
  Serial.write(command);
  Serial.write(AH);
  Serial.write(AL);
  Serial.write((unsigned char)(255 - (command+AH+AL)%256));
}


void IDPacket()  
{
  Serial.write(0xFF);
  Serial.write((unsigned char) ARMID);
  Serial.write((unsigned char) g_bIKMode);
  Serial.write((unsigned char) 0);
  Serial.write((unsigned char)(255 - (ARMID+g_bIKMode+0)%256));
}



void DigitalOutputs(){
         // First bit = D1, 2nd bit = D2, etc. 
        int i;
        for(i=0;i<7;i++){
        unsigned char button = (armcontrol.buttons>>i)&0x01;
        if(button > 0){
          // button pressed, go high on a pin
          DDRB |= 0x01<<(i+1);
          PORTB |= 0x01<<(i+1);
        }
        else{
          DDRB &= 0xff - (0x01<<(i+1));
          PORTB &= 0xff - (0x01<<(i+1));
        } 
      } 
}



//=============================================================================
//=============================================================================
#endif
