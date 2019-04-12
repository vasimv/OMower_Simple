// OMower ROS support routines
// $Id$

#include "robot.h"
#include "rossupport.h"
#include <math.h>

// Translate Quaternion to roll/pitch/yaw
void convertQuat(float qX, float qY, float qZ, float qW, float &pitch, float &roll, float &yaw) {
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (qW * qX + qY * qZ);
  double cosr_cosp = +1.0 - 2.0 * (qX * qX + qY * qY);
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (qW * qY - qZ * qX);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (qW * qZ + qX * qY);
  double cosy_cosp = +1.0 - 2.0 * (qY * qY + qZ * qZ);  
  yaw = atan2(siny_cosp, cosy_cosp);
} // void convertQuat(float qX, float qY, float qZ, float qW, float &pitch, float &roll, float &yaw)

#ifdef USE_ROS
// move_base_simple/goal callback (use only yaw angle and ignore altitude)
void cbROSMove(float pX, float pY, float pZ, float qX, float qY, float qZ, float qW) {
  float pitch, roll, yaw;

  // Calculate yaw angle at the end of command
  convertQuat(qX, qY, qZ, qW, pitch, roll, yaw);

  // Set yaw (OMower uses inverted yaw angle)
  currCmdAngle = imu::piDegree(imu::scalePI(-yaw + M_PI / 2));
  currCmdLat = pY * 100.0;
  currCmdLon = pX * 100.0;
  debug(L_NOTICE, "move_base_simple/goal: %f %f %f; %f %f %f %f (yaw: %hd)\n", pX, pY, pZ, qX, qY, qZ, qW, yaw);
  oGps.UTMmode = true;
  // TODO: set flags
  cmdTime = 3600000;
  startCommand(CMD_ROS_MOVE_GPS);
} // void cbROSMove(float pX, float pY, float pZ, float qX, float qY, float qZ, float qW)

// cmd_vel callback
void cbROSCmdVel(float lX, float lY, float lZ, float aX, float aY, float aZ) {
  // Reset timestamp
  cmdStartTime = millis();
  // Set velocities and angles
  yawSpeed = aZ;
  forwSpeed = lX;
  cmdFlags |= FL_IGNORE_WIRE | FL_IGNORE_VIRTUAL;
  // 500 milliseconds maximum
  cmdRun = CMD_STOP;
  cmdTime = ROS_CMDVEL_TIMEOUT;
  startCommand(CMD_ROS_CMDVEL);
} // void cbROSCmdVel(float lX, float lY, float lZ, float aX, float aY, float aZ)

// current_position callback
void cbROSCurPose(float pX, float pY, float pZ, float qX, float qY, float qZ, float qW) {
  // Set GPS coordinates
  oGps.UTMmode = true;
  oGps.setCoords(pY * 100.0, pX * 100.0, pZ * 100.0, oGps.zoneUTMRaw, 254, 0, 0, 0, 0, 0, 0);
  // TODO: do something with angles
} // void cbROSCurPose(float pX, float pY, float pZ, float qX, float qY, float qZ, float qW)
#endif

// Set up callbacks
void rosSupportInit() {
#ifdef USE_ROS
  // oROS.setCallBackMove(&cbROSMove);
  oROS.setCallBackCurPose(&cbROSCurPose);
  oROS.setCallBackCmdVel(&cbROSCmdVel);
#endif
} // void rosSupportInit()
