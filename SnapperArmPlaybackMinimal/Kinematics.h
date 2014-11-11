#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "GlobalArm.h"
#include <Arduino.h>


//////////////////////////////////////////////////////////////////////////////
// KINEMATICS CONFIG  //
//////////////////////////////////////////////////////////////////////////////


// status messages for IK return codes..
enum {
  IKS_SUCCESS=0, IKS_WARNING, IKS_ERROR};

#define IK_FUDGE            5     // How much a fudge between warning and error
uint8_t         g_bIKStatus = IKS_SUCCESS;   // Status of last call to DoArmIK;

#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  //float to long conversion
/* pre-calculations */
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

// IK coordinate variables
// Current IK values
float            g_sIKX  =0.00;                     // Current X value in mm
float            g_sIKY  =150.00;                  //
float            g_sIKZ  =150.00;
float            g_sIKGA =0.00;                  // IK Gripper angle..

//Next IK Values
float            sIKX  =0.00;                     // Current X value in mm
float            sIKY  =0.00;                  //
float            sIKZ  =0.00;
float            sIKGA =0.00;                  // IK Gripper angle..

////////////////////////////////////////////////////////////////////////////// 


//===================================================================================================
// Compute Arm IK for 3DOF+Mirrors+Gripper - was based on code by Michael E. Ferguson
// Hacked up by me, to allow different options...
//===================================================================================================
uint8_t doArmIK(boolean fCartesian, float sIKX, float sIKY, float sIKZ, float sIKGA)
{
  float t;
  uint8_t bRet = IKS_SUCCESS;  // assume success
  float base_angle_r;

  if (fCartesian) {
    // first, make this a 2DOF problem... by solving baseAngle, converting to servo pos
    base_angle_r = atan2(sIKX, sIKY);
    // remove gripper offset from base
    t = sqrt(sq((long)sIKX)+sq((long)sIKY));
  }
  else {
    // We are in cylindrical mode, probably simply set t to the y we passed in...
    t = sIKY;
  }
  
  
  // convert to sIKX/sIKZ plane, remove wrist, prepare to solve other DOF           
  float flGripRad = radians(sIKGA);
  float trueX = t - ((float)GRIPPER*cos(flGripRad));   
  float trueZ = sIKZ - BASE_HGT - ((float)GRIPPER*sin(flGripRad));

  float im = sqrt(sq(trueX)+sq(trueZ));        // length of imaginary arm
  float q1 = atan2(trueZ,trueX);              // angle between im and X axis
  float d1 = sq((float)HUMERUS) - sq(ULNA) + sq((long)im);
  float d2 = 2*(float)HUMERUS*im;
  float q2 = acos((float)d1/float(d2));
  q1 = q1 + q2;

  d1 = sq((float)HUMERUS)-sq(im)+sq((float)ULNA);
  d2 = 2*(float)ULNA*(float)HUMERUS;
  q2 = acos((float)d1/(float)d2);


  //Use different radians equation for AX servos
  float sol0 = degrees(base_angle_r);
  float sol1 = degrees(q1-1.57);
  float sol2 = degrees(3.14-q2);
  // solve for wrist angle
  float sol3 = degrees(3.2 + flGripRad - q1 - q2 );



  /* Servo pulses */
  Base = (ftl(1500.0 - (sol0 * 10.55) ) );
  Shoulder = (ftl(1500.0 - ( (sol1) * 10.55 )));
  Elbow = (ftl(1500.0 - ( (sol2 -90) * 10.55 )));
  Wrist = (ftl(1500 + ( sol3  * 10.55 )));
  
//
//  /* Servo pulses */
//  Base = (ftl(1500.0 - (sol0 * 10.55) ) );
//  Shoulder = (ftl(1500.0 - (( sol1 - 90 ) * 10.55 )));
//  Elbow = (ftl(1500.0 + (( sol2 - 90.0 ) * 10.55 )));
//  Wrist = (ftl(1500 + ( sol3  * 10.55 )));  

  // Remember our current IK positions
  g_sIKX = sIKX; 
  g_sIKY = sIKY;
  g_sIKZ = sIKZ;
  g_sIKGA = sIKGA;
  // Simple test im can not exceed the length of the Shoulder+Elbow joints...

  if (im > (HUMERUS + ULNA)) {
        Serial.println("IK Error");
        bRet = IKS_ERROR;  
  }
  else if(im > (HUMERUS + ULNA-IK_FUDGE)) {
        Serial.println("IK Warning");
        bRet = IKS_WARNING;  
  }

  return bRet;
  
}


////===================================================================================================
//// doArmIK: Floating Point Arm IK Solution for PWM Servos
////===================================================================================================
///* arm positioning routine utilizing inverse kinematics */
///* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
//uint8_t doArmIK(boolean fCartesian, float x, float y, float z, float grip_angle_d)
//{
//  
//  
//  uint8_t bRet = IKS_SUCCESS;  // assume success
//  
//  float grip_angle_r = radians( grip_angle_d );    //grip angle in radians for use in calculations
//
//  /* Base angle and radial distance from x,y coordinates */
//!  float bas_angle_r = atan2( x, y );
//  
//  float rdist = sqrt(( x * x ) + ( y * y ));
//  /* rdist is y coordinate for the arm */
//  y = rdist;
//  /* Grip offsets calculated based on grip angle */
//  float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
//  
//  float grip_off_y = ( cos( grip_angle_r )) * GRIPPER;
//  /* Wrist position */
//  float wrist_z = ( z - grip_off_z ) - BASE_HGT;
//  float wrist_y = y - grip_off_y;
//  /* Shoulder to wrist distance ( AKA sw ) */
//  float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
//  float s_w_sqrt = sqrt( s_w );
//  /* s_w angle to ground */
//  //float a1 = atan2( wrist_y, wrist_z );
//  float a1 = atan2( wrist_z, wrist_y );
//  /* s_w angle to humerus */
//  float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));
//  /* shoulder angle */
//  float shl_angle_r = a1 + a2;
//  float shl_angle_d = degrees( shl_angle_r );
//  /* elbow angle */
//  float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
//  float elb_angle_d = degrees( elb_angle_r );
//  float elb_angle_dn = -( 180.0 - elb_angle_d );
//  /* wrist angle */
//  float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;
//
//  /* Servo pulses */
//  Base = (ftl(1500.0 - (( degrees( bas_angle_r )) * 10.55 )));
//  Shoulder = (ftl(1500.0 - (( shl_angle_d - 90) * 10.55 )));
//  Elbow = (ftl(1500.0 + (( elb_angle_d - 90.0 ) * 10.55 )));
//  Wrist = (ftl(1500 + ( wri_angle_d  * 10.55 )));
//
//
//  if (rdist > (HUMERUS + ULNA)) {
//    bRet = IKS_ERROR; 
//    Serial.println("IK Error");
//    Serial.println(".");
//  }
//  else if (rdist > (HUMERUS + ULNA - IK_FUDGE)){
//    bRet = IKS_WARNING;
//    Serial.println("IK Warning");
//    Serial.println(".");
//  }
//  return bRet;
//}

//=============================================================================
//=============================================================================
#endif


