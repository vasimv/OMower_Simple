// OMower simple - simple automatic lawn mowing with OMower SDK
// Inspired by ardumower project - http://www.ardumower.de/
// $Id$

#ifndef _OMOWER_SIMPLE_H
#define _OMOWER_SIMPLE_H

// Use DW1000 radio tags for navigation
#define USE_DW1000

// OMower SDK includes
#include <omower-root.h>
#include <omower-defs.h>
#include <omower-motors.h>
#include <omower-odometry.h>
#include <omower-imu.h>
#include <omower-perimeter.h>
#include <omower-gps.h>
#include <omower-chassis.h>
#include <omower-debug.h>
#include <omower-buttonsleds.h>
#include <omower-rtc.h>
#include <omower-mow.h>
#include <omower-pwmservo.h>
#include <omower-sonars.h>
#include <omower-lawn.h>
#include <omower-rain.h>
#include <omower-drop.h>
#include <omower-bumper.h>
#include <omower-power.h>
#include <omower-nvmem.h>
#include <omower-serial.h>
#ifdef USE_DW1000
#include <omower-radiotag.h>
#endif

// For rpl_vsnprintf and rpl_snprintf with float support
#include <xsystem.h>

// Software revision number (used for NVMEM check, must be changed if different numbers of variables saved)
#define REVISION 0x00000002

// OMower SDK objects
extern chassis oChassis;
extern motors oMotors;
extern imu oImu;
extern perimeter oPerim;
extern gps oGps;
#ifdef USE_DW1000
extern radiotag oTag;
#endif
extern buttonsLeds oSwitches;
extern motorMow oMow;
extern pwmServo oPwm;
extern sonars oSonars;
extern bumper oBumper;
extern drop oDrop;
extern rain oRain;
extern lawn oLawn;
extern currentMow oCrrMow;
extern currentPow oCrrPower;
extern currentMotors oCrrMotors;
extern odometryMotors oOdoMotors;
extern power oPower;
extern rtc oRtc;
extern serial oSerial;
extern nvmem oSave;

// NVMEM address for load/save settings
extern uint32_t nvmemAddr;

// LED control (0 - no light, 1 - fast blinking, 2 - slow blinking
extern volatile uint8_t ledState;

// Load/save settings variables through NVMEM object
void loadSaveVars(boolean save);

void displayStuff();
#endif
