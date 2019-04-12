// OMower ROS support routines
// $Id$

#include "OMower_Simple.h"
#include <stdint.h>

// Translate Quaternion to roll/pitch/yaw
void convertQuat(float qX, float qY, float qZ, float qW, float &pitch, float &roll, float &yaw);

// Init stuff (set up callbacks and such)
void rosSupportInit();
