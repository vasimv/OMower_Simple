// OMower demo command functions
// $Id$

#include "OMower_Simple.h"
#include "robot.h"
#include "pfod.h"

// Commands labels
const char *cmdLabels[] = { "STOP", "MOVE", "FIND_PERIMETER", "ZIGZAG", "NORTH", "SOUTH", "EAST", "WEST", "SPIRAL",
                            "POINT", "ERROR", "EXT_MOVE_ANGLE", "EXT_MOVE_GPS", "EXT_MOVE_TAGS", "EXT_ROLL_ANGLE",
                            "EXT_REVERSE", "EXT_ROLL_PERIMETER", "EXT_MOVE_PERIMETER", "EXT_CALIB_START",
                            "EXT_CALIB_FINISH", "EXT_SAVE_NVMEM", "PSAVE", "NORMAL", "SHUTDOWN", "MOVE_SEEKER",
                            "HOME", "VIRT_LAWN", "ROS_MOVE", "ROS_CMDVEL" };

// Current command running
uint16_t cmdRun = CMD_STOP;
// Next command to run
uint16_t cmdNextRun = CMD_STOP;

// Current command's flags
uint16_t cmdFlags = 0;

// For modbus interface commands
// Time of execution (in milliseconds)
uint32_t cmdTime;

// Angle for movement
int16_t currCmdAngle;

// Coordinates for GPS/RTK GPS movement
int32_t currCmdLat;
int32_t currCmdLon;

// Coordinates for radiotag movement
int32_t currCmdX;
int32_t currCmdY;
uint16_t currCmdRoom;

// Last command stop reason
uint16_t cmdStopReason;

// ROS /cmdvel arguments
float forwSpeed, yawSpeed;

// Finish sub-command (new command received or serious hardware error)
boolean finishSub = false;

// Last turn direction at perimeter
_dir lastPerimTurn = _dir::LEFT;
uint16_t lastPerimTurnCount = 0;

// Last perimeter out time and counter increasing if the turn was in less than 20 seconds
uint32_t lastPerimOut = 0;
uint32_t lastOutCnt = 0;

// Configuration variables
// How much we have to slow speed in "save power" mode
float optSlowRatio;

// How much we have to move backward when hitting obstacle or perimeter
uint32_t optBackwardTime;

// Minimum roll time for random rolls
uint32_t optRollTimeMin;

// Maximum roll time for random rolls
uint32_t optRollTimeMax;

// Height of mowing motor
uint8_t optMowHeight;

// width of the square (step is about 1.1cm)
uint16_t optSquareSize;
// Number of latitude steps
uint16_t optSquareCycles;
// Latitude step (is about 1.1cm)
uint16_t optSquareInc;
// Latitude offset for eastern side
int16_t optSquareOffset;

// target coordinates (for CMD_POINT)
int32_t pointLat, pointLon;

// Coordinates for CMD_HOME command
// Start point for CMD_HOME (inside of mowing area)
int32_t homeFLat, homeFLon;
// Point near of the base to switch to seeker
int32_t homeLLat, homeLLon;
// Direction to the base from homeLLat,homeLLon point
int16_t homeDir;

// Coordinates for emergency return point inside virtual perimeter
int32_t virtRLat, virtRLon;

// Returns random number in range
uint32_t getRandomRange(uint32_t minNum, uint32_t maxNum) {
  return random(minNum, maxNum);
} // uint32_t getRandomRange(uint32_t minNum, uint32_t maxNum)

// Restart compass calibration
void resetCompassCalib() {
  oImu.startCompassCalib();
  ledState = 2;
} // void resetCompassCalib()

// Stop compass calibration
void stopCompassCalib() {
  oImu.stopCompassCalib();
  ledState = 1;
} // void stopCompassCalib()

// Perform accelerometer calibration (robot must stay on flat leveled surface!)
void accelCalib() {
  oImu.calibAccel();
} // void accelCalib()

uint32_t lastDisplay = 0;
uint32_t lastLoop = 0;

// Check important stuff (battery, commands received, GPS serial port), reset IMU if needed
void subMustCheck() {
  uint8_t nChecks = 0;

  lastLoop = millis();
  subCheckConsole();
  oROS.spinOnce();

  // Reset IMU if we've got an error
  if (oImu.softError() == _hwstatus::ERROR)
    oImu.init(&oSave);

  // Check for battery voltage for emergency shutdown
  while (oPower.readSensor(P_BATTERY) < oPower.minBatteryVoltage) {
    oPower.reportToROS();
    // Don't turn off robot for first 5 minutes or if charging is in progress
    if ((millis() < 300000) || (oCrrPower.readCurrent(P_BATTERY) < -0.2))
      break;
    delayWithROS(100);
    nChecks++;
    if (nChecks > 10) {
      debug(L_WARNING, (char *) F("Battery voltage is discharged completely, shutting down!\n"));
      delayWithROS(50);
      oPower.emergShutdown();
    }
  }
} // void subMustCheck()

// Set speeds of wheel and mowing motors
void subSpeedSetup(uint16_t flags) {
  if (flags & FL_SAVE_POWER) {
    oMotors.maxSpeed = SPEED_MAX / optSlowRatio;
    oMow.maxSpeed = 0;
  } else {
    oMotors.maxSpeed = SPEED_MAX;
    // Start mowing motor if inside of perimeter
    if ((flags & FL_IGNORE_WIRE) || oPerim.inside(-1))
      oMow.maxSpeed = SPEED_MAX;
    else
      oMow.maxSpeed = 0;
  }
} // void subSpeedSetup(uint16_t flags)

// Check overload/fails on motors
boolean subCheckMotors() {
  if (oMotors.softError() == _hwstatus::ERROR) {
    oMotors.reportToROS();
    oCrrMotors.reportToROS();
    return true;
  }
  return false;
} // boolean subCheckMotors()

// Check overload/fails on mowing motor
boolean subCheckMow() {
  if (oMow.softError() == _hwstatus::ERROR) {
    oMow.reportToROS();
    oCrrMow.reportToROS();
    return true;
  }
  return false;
} // boolean subCheckMow()

// Reset mowing motor driver
void subResetMow() {
  debug(L_NOTICE, (char *) F("subResetMow: resetting mowing motor driver\n"));
  // Stop moving around while resetting mowing motors driver
  oMotors.emergStop();
  oMow.init();
} // Reset mowing motor driver

// Wait motors to stop
void subWaitMotorsStop() {
  uint32_t startWait = millis();

  while ((millis() - startWait) < 20000) {
    subMustCheck();
    if ((oMotors.curPWM(0) == 0) && (oMotors.curPWM(1) == 0))
      break;
  }
  
//  // Wait 500 ms more
//  startWait = millis();
//  while ((millis() - startWait) < 500)
//    subMustCheck();
} // void subWaitMotorsStop()

uint32_t subStop() {
  uint32_t stopTime = millis();
  
  oMotors.emergStop();
  oMow.maxSpeed = 0;
  subWaitMotorsStop();
  return millis() - stopTime;
} // uint32_t subStop(uint16_t flags)

// Reset motors driver
void subResetMotors() {
  debug(L_NOTICE, (char *) F("subResetMotors: resetting motor drivers\n"));
  oMotors.emergStop();
  oMow.maxSpeed = 0;
  subWaitMotorsStop();
  oMotors.init();
  delayWithROS(100);  
} // void subResetMotors()


// Check rain sensor, switch to station return if detected
void subCheckRain() {

} // void subCheckRain()

// Check drop sensor, emergency stop if detected
void subCheckDrop() {
  
} // void subCheckDrop()

// Check sonars and return direction of which sonar hit
_dir subCheckSonar() {
  int left, right, forward;

  left = oSonars.readSonar(0);
  forward = oSonars.readSonar(1);
  right = oSonars.readSonar(2);
  // Check center sonar if we do have it
  if (((oSonars.locThings() == _locationThings::JUSTONE)
       || (oSonars.locThings() == _locationThings::LEFT_CENTER_RIGHT))
      && ((forward > 0) && (forward < oSonars.triggerDist))) {
    debug(L_INFO, (char *) F("subCheckSonar: center %d\n"), forward);
    oSonars.reportToROS();
    return _dir::FORWARD;
  }
  // Check left forward sonar
  if ((left > 0) && (left < oSonars.triggerDist)) {
    debug(L_INFO, (char *) F("subCheckSonar: left %d\n"), left);
    oSonars.reportToROS();
    return _dir::LEFT;
  }
  // Check right forward sonar
  if ((right > 0) && (right < oSonars.triggerDist)) {
    debug(L_INFO, (char *) F("subCheckSonar: right %d\n"), right);
    oSonars.reportToROS();
    return _dir::RIGHT;
  }
  return _dir::STOP;
} // _dir subCheckSonar()

// Check bumpers and return direction of which bumper hit
_dir subCheckBumper() {
  boolean left, right;

  left = oBumper.readSensor(0);
  right = oBumper.readSensor(1);
  if (left && right) {
    oBumper.reportToROS();
    return _dir::FORWARD;
  }
  if (left) {
    oBumper.reportToROS();
    return _dir::LEFT;
  }
  if (right) {
    oBumper.reportToROS();
    return _dir::RIGHT;
  }
  return _dir::STOP;
} // _dir subCheckBumper()

// Check perimeter (wire and virtual). Note, it returns "true" when both perimeters are disabled!
boolean subCheckPerimeter(uint16_t flags) {
  // Both perimeters are disabled by flags, so we return "inside" status
  if ((flags & FL_IGNORE_WIRE) && (flags & FL_IGNORE_VIRTUAL))
    return true;
  // Check perimeter
  if ((!(flags & FL_IGNORE_WIRE) && oPerim.inside(-1))
      || (!(flags & FL_IGNORE_VIRTUAL) && oVPerim.inside()))
    return true;

  // Check if we're out of perimeter too long
  if ((!(flags & FL_IGNORE_WIRE) && (oPerim.softError() != _hwstatus::ONLINE))
      || (!(flags & FL_IGNORE_VIRTUAL) && (oVPerim.softError() != _hwstatus::ONLINE))) {
    if (flags & FL_IGNORE_WIRE)
      oVPerim.reportToROS();
    else
      oPerim.reportToROS();
    debug(L_INFO, (char *) F("subCheckPerimeter: breaking execution because softError [wire: %d, virtual: %d]\n"),
          (int) oPerim.softError(), (int) oVPerim.softError());
    finishSub = true;
    cmdStopReason = FIN_PERIMETER;
  }
  return false;
} // boolean subCheckPerimeter(uint16_t flags)

// Subcommand to roll, returns execution time in milliseconds
// dirMove == _dir::STOP - use angle
// angle == INVALID_ANGLE - use dirMove
uint32_t subRoll(uint16_t flags, uint32_t timeout, _dir dirMove, int16_t angle) {
  uint32_t subStart = millis();
  uint32_t subEnd = millis() + timeout;
  int32_t leftTime = timeout;
  boolean checkImu = false;

  cmdStopReason = FIN_NONE;
  debug(L_INFO, (char *) F("subRoll: flags %hx, timeout %lu, dirMove %hd, angle %hd\n"),
        flags, timeout, dirMove, angle);
  while (!finishSub && (millis() < subEnd)) {
    // Set speed of wheel and mowing motors
    subSpeedSetup(flags);

    // Start rolling
    if (dirMove != _dir::STOP) {
      oMotors.roll(dirMove, leftTime);
    } else {
      if (angle != INVALID_ANGLE) {
        // Roll by an angle (IMU)
        if ((flags & FL_IGNORE_IMU) || (oImu.softError() != _hwstatus::ONLINE))
          // Well, we have to ignore IMU or it is not available, let's just stop
          return leftTime;
        // Initialize rolling by an angle
        oImu.setCourse(angle, true);
        oMotors.rollCourse((navThing *) &oImu, leftTime);
        checkImu = true;
      } else {
        cmdStopReason = FIN_TIME;
        return leftTime;
      }
    }

    // Internal loop
    while (!finishSub && (millis() < subEnd)) {
      // Check stuff like battery, commands, GPS serial port and reset IMU when needed
      subMustCheck();
      // Check rain sensor
      subCheckRain();
      // check drop sensors
      subCheckDrop();

      // Check and reset motor drivers if needed
      if (subCheckMotors()) {
        subResetMotors();
        if (flags & FL_SIMPLE_MOVE) {
          cmdStopReason = FIN_MOTORS;
          return leftTime;
        }
        // Move a bit backward in case if this is caused by an obstacle
        subMove(flags | FL_IGNORE_SONAR | FL_IGNORE_BUMPER, optBackwardTime, _dir::BACKWARD,
                INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        break;
      }

      // Check if we've reached needed angle
      if (checkImu && oImu.reachedDest()) {
        subStop();
        cmdStopReason = FIN_DESTINATION;
        return leftTime;
      }
        
      // Roll until we get in or out of the wire perimeter
      if (!(flags & FL_IGNORE_WIRE)) {
        if (oPerim.inside(-1)) {
          // Check special stop condition
          if (flags & FL_STOP_INSIDE) {
            oPerim.reportToROS();
            subStop();
            cmdStopReason = FIN_PERIMETER;
            return leftTime;
          }
        } else {
          // Check special stop condition
          if (flags & FL_STOP_OUTSIDE) {
            oPerim.reportToROS();
            subStop();
            cmdStopReason = FIN_PERIMETER;
            return leftTime;
          }
        }
      }

      // Roll until we get in or out of the virtual perimeter
      if (!(flags & FL_IGNORE_VIRTUAL)) {
        if (oVPerim.inside()) {
          // Check special stop condition
          if (flags & FL_STOP_INSIDE) {
            oVPerim.reportToROS();
            subStop();
            cmdStopReason = FIN_PERIMETER;
            return leftTime;
          }
        } else {
          // Check special stop condition
          if (flags & FL_STOP_OUTSIDE) {
            oVPerim.reportToROS();
            subStop();
            cmdStopReason = FIN_PERIMETER;
            return leftTime;
          }
        }
      }
    }
    // Calculate time left
    leftTime = (int32_t) subEnd - (int32_t) millis();
  }

  if (cmdStopReason == FIN_NONE)
    cmdStopReason = FIN_TIME;
  if (leftTime >= 0)
    return leftTime;
  else
    return 0;
} // uint32_t subRoll(uint16_t flags, uint32_t timeout, _dir dirMove, int16_t angle)

// Roll to perimeter's maximum signal strength
// For wire perimeter - first roll around, then roll to maximum of this
// For virtual perimeter - just roll to direction of closest point
uint32_t subPerimSearch(uint16_t flags, uint32_t timeout) {
  uint32_t subStart = millis();
  uint32_t subEnd = millis() + timeout;
  int32_t leftTime = timeout;
  uint32_t lastMove;

  cmdStopReason = FIN_NONE;
  debug(L_INFO, (char *) F("subPerimSearch: flags %hx, timeout %lu\n"),
        flags, timeout);
  while (!finishSub && (millis() < subEnd)) {
    // Set speed of wheel and mowing motors
    subSpeedSetup(flags);

    // Start rolling
    if (!(flags & FL_IGNORE_WIRE)) {
      oPerim.setTracking(false);
      oMotors.rollCourse((navThing *) &oPerim, leftTime);
    }
    if (!(flags & FL_IGNORE_VIRTUAL)) {
      oVPerim.setTracking(false);
      oMotors.rollCourse((navThing *) &oVPerim, leftTime);
    }
    lastMove = millis();
    
    // Internal loop
    while (!finishSub && (millis() < subEnd)) {
      // Check stuff like battery, commands, GPS serial port and reset IMU when needed
      subMustCheck();
      // Check rain sensor
      subCheckRain();
      // check drop sensors
      subCheckDrop();

      // Check and reset motor drivers if needed
      if (subCheckMotors()) {
        subResetMotors();
        if (flags & FL_SIMPLE_MOVE) {
          cmdStopReason = FIN_MOTORS;
          return leftTime;
        }
        // Move a bit backward in case if this is caused by an obstacle
        subMove(flags | FL_IGNORE_SONAR | FL_IGNORE_BUMPER | FL_SAVE_POWER, optBackwardTime, _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        break;
      }
      if (!(flags & FL_IGNORE_WIRE) || !(flags & FL_IGNORE_VIRTUAL)) {
        if (subCheckPerimeter(flags)) {
          // Check special stop condition
          if (flags & FL_STOP_INSIDE) {
            debug(L_INFO, (char *) F("subPerimSearch: inside perimeter, stopping\n"));
            // Move a bit longer to be sure we'll get into perimeter
            subMove(flags | FL_IGNORE_SONAR | FL_IGNORE_BUMPER | FL_SAVE_POWER, optBackwardTime / 2, _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
            subStop();
            cmdStopReason = FIN_PERIMETER;
            return leftTime;
          }
        } else {
          // Check special stop condition
          if (flags & FL_STOP_OUTSIDE) {
            debug(L_INFO, (char *) F("subPerimSearch: outside of perimeter, stopping\n"));
            subStop();
            cmdStopReason = FIN_PERIMETER;
            return leftTime;
          }
        }
      }
      // Check if we've parked in the station
      if (oPower.readSensor(P_BATTERY) < oPower.readSensor(P_CHARGE)) {
        oPower.reportToROS();
        subStop();
        cmdStopReason = FIN_CHARGE;
        return leftTime;
      }
      // Check if we weren't move for 2 seconds or reached perimeter's direction
      if (((oMotors.curPWM(0) == 0) && (oMotors.curPWM(1) == 0)) 
          || (!(flags & FL_IGNORE_VIRTUAL) && oVPerim.reachedDest())) {
        if ((millis() - lastMove) > 2000) {
          cmdStopReason = FIN_DESTINATION;
          return leftTime;
        }
      } else
        lastMove = millis();
    }
    // Calculate time left
    leftTime = (int32_t) subEnd - (int32_t) millis();
  }

  cmdStopReason = FIN_TIME;
  if (leftTime >= 0)
    return leftTime;
  else
    return 0;
} // uint32_t subPerimSearch(uint16_t flags, uint32_t timeout)

// Subcommand to move, returns execution time in milliseconds
// dirMove == _dir::STOP - use angle or coordinates
// angle == INVALID_ANGLE - use dirMove or coordinates
// latitude == INVALID_COORD - use angle or dirMove
// If _dir::FORW and INVALID_* - switch to tracking perimeter mode (except FL_IGNORE_WIRE && FL_IGNORE_VIRTUAL - switch to STOP otherwise)
uint32_t subMove(uint16_t flags, uint32_t timeout, _dir dirMove, int16_t angle, int32_t latitude, int32_t longitude) {
  uint32_t subStart = millis();
  uint32_t subEnd = millis() + timeout;
  int32_t leftTime = timeout;
  _dir resSonar, resBumper;
  boolean perimTrack = false;
  boolean seekerCheck = false;
  int32_t latNext, lonNext;

  cmdStopReason = FIN_NONE;
  debug(L_INFO, (char *) F("subMove: flags %02hX, timeout %lu, dirMove %hd, angle %hd, latitude %ld, longitude %ld\n"),
        flags, timeout, dirMove, angle, latitude, longitude);

  while (!finishSub && (millis() < subEnd)) {
    // Set speed of wheel and mowing motors
    subSpeedSetup(flags);
     
    // Start moving
    if (dirMove != _dir::STOP) {
      debug(L_INFO, (char *) F("subMove: direction movement\n"));
      oMotors.move(dirMove, leftTime);
    } else {
      if ((angle != INVALID_ANGLE) && (angle != -INVALID_ANGLE)) {
        // Move by an angle (IMU)
        if ((flags & FL_IGNORE_IMU) || (oImu.softError() != _hwstatus::ONLINE)) {
          // Well, we have to ignore IMU or it is not available, let's just stop
          debug(L_NOTICE, (char *) F("subMove: IMU error %hd\n"), oImu.softError());
          return leftTime;
        }
        debug(L_INFO, (char *) F("subMove: moving by compass\n"));
        oImu.setCourse(angle);
        oMotors.moveCourse((navThing *) &oImu, leftTime);
      } else {
        if (angle == -INVALID_ANGLE) {
          debug(L_INFO, (char *) F("subMove: moving by seeker\n"));
          seekerCheck = true;
          oSeeker.startSeeker(true, 40);
          oMotors.moveCourse(&oSeeker, leftTime);
        } else {
          if (latitude != INVALID_COORD) {
            // Move to GPS/tags coordinates
            // GPS is not functioning, just stop
            if (oGps.softError() != _hwstatus::ONLINE)
              return leftTime;
            // Initialize movement by the GPS
            debug(L_INFO, (char *) F("subMove: moving by GPS\n"));
            oGps.setTarget(latitude, longitude, oGps.zoneUTM, (flags & FL_HIGH_PRECISION) > 0, true);
            oMotors.moveCourse(&oGps, leftTime);
          } else {
            if ((flags & FL_IGNORE_WIRE) && (flags & FL_IGNORE_VIRTUAL))
              oMotors.move(_dir::STOP, leftTime);
            else {
              // Initialize movement by perimeter (wire or virtual)
              perimTrack = true;
              if (flags & FL_IGNORE_WIRE) {
                debug(L_INFO, (char *) F("subMove: moving by virtual Perimeter\n"));
                oVPerim.setTracking(true);
                oMotors.moveCourse(&oVPerim, leftTime);
              } else {
                debug(L_INFO, (char *) F("subMove: moving by Perimeter\n"));
                oPerim.setTracking(true);
                oMotors.moveCourse(&oPerim, leftTime);
              }
            }
          }
        }
      }
    }

    // Internal loop
    while (!finishSub && (millis() < subEnd)) {
      leftTime = (int32_t) subEnd - (int32_t) millis();
      // Check stuff like battery, commands, GPS serial port and reset IMU when needed
      subMustCheck();
      // Check rain sensor
      subCheckRain();
      // check drop sensors
      subCheckDrop();
      
      // Check and reset motor drivers if needed
      if (subCheckMotors()) {
        subResetMotors();
        // Simply stop if we have simple move flag
        if (flags & FL_SIMPLE_MOVE) {
          cmdStopReason = FIN_MOTORS;
          return leftTime;
        }
        // Move a bit backward in case if this is caused by an obstacle
        subMove(flags | FL_IGNORE_SONAR | FL_IGNORE_BUMPER | FL_SAVE_POWER, optBackwardTime, _dir::BACKWARD,
                INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        break;
      }

      // Check and reset mowing motor if needed
      if (subCheckMow()) {
        // Simply stop if we have simple move flag
        if (flags & FL_SIMPLE_MOVE) {
          cmdStopReason = FIN_MOWER;
          return leftTime;
        }
        subResetMow();
      }

      // Check sonar and bumper sensors
      if ((!(flags & FL_IGNORE_SONAR) && ((resSonar = subCheckSonar()) != _dir::STOP))
          || (!(flags & FL_IGNORE_BUMPER) && ((resBumper = subCheckBumper()) != _dir::STOP))) {
        _dir dirRoll;

        debug(L_NOTICE, (char *) F("Found obstacle\n"));
        subStop();
        // Check if we should try to move around or just stop
        if (flags & FL_SIMPLE_MOVE) {
          if (resSonar != _dir::STOP)
            cmdStopReason = FIN_SONAR;
          else
            cmdStopReason = FIN_BUMPER;
          return leftTime;
        }
        // Calculate coordinates of the next point after moving around the obstacle (2 meters away)
        if (latitude != INVALID_COORD)
          oGps.calcPoint(2.0, latNext, lonNext, latitude, longitude);
        subMove(flags | FL_STOP_OUTSIDE | FL_IGNORE_SONAR | FL_IGNORE_BUMPER, optBackwardTime,
                _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        subStop();
        // Try to roll in other direction than an obstacle
        if ((resSonar == _dir::LEFT) || (resBumper == _dir::LEFT))
          dirRoll = _dir::RIGHT;
        else {
          dirRoll = _dir::LEFT;
        }
        subRoll(flags | FL_STOP_OUTSIDE | FL_IGNORE_SONAR, getRandomRange(optRollTimeMin, optRollTimeMax),
                dirRoll, INVALID_ANGLE);
        subMove(flags | FL_STOP_OUTSIDE | FL_IGNORE_SONAR, getRandomRange(optRollTimeMin, optRollTimeMax),
                _dir::FORWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        subStop();
        // Return to way point
        if (latitude != INVALID_COORD) {
          subMove(flags, 30, _dir::STOP, INVALID_ANGLE, latNext, lonNext);
          oGps.setTarget(latitude, longitude, oGps.zoneUTM, (flags & FL_HIGH_PRECISION) > 0, true);
          oMotors.moveCourse((navThing *) &oGps, leftTime);
        }
        break;
      }

      // Check wire/virtual perimeter
      if (!(flags & FL_IGNORE_WIRE) || !(flags & FL_IGNORE_VIRTUAL)) {
        // Check if we don't know virtual perimeter status (no GPS info)
        if (!(flags & FL_IGNORE_VIRTUAL) && oVPerim.unknown()) {
          debug(L_INFO, (char *) F("Unknown virtual perimeter status, stopping\n"));
          subStop();
          while (!finishSub && oVPerim.unknown()) {
            subMustCheck();
            subCheckRain();
            subCheckDrop();
            delayWithROS(1);
          }
          break;
        }
        if (subCheckPerimeter(flags)) {
          // Check special stop condition
          if (flags & FL_STOP_INSIDE) {
            debug(L_NOTICE, (char *) F("subMove: Inside perimeter, stop\n"));
            subStop();
            cmdStopReason = FIN_PERIMETER;
            return leftTime;
          }
        } else {
          // We know that we're outside, continue
          if ((flags & FL_STOP_INSIDE) || perimTrack)
            continue;
          // Perform stop when reached out of perimeter
          debug(L_NOTICE, (char *) F("subMove: Out of perimeter, wire %hd (%f) %hd (%f), virtual %d, stop\n"),
                oPerim.magnitude(0), oPerim.quality(0), oPerim.magnitude(1), oPerim.quality(1), (int) oVPerim.inside());
          subStop();
          // Check special stop condition
          if ((flags & FL_STOP_OUTSIDE) || (flags & FL_SIMPLE_MOVE)) {
            cmdStopReason = FIN_PERIMETER;
            return leftTime;
          }
            
          // Well, we've got out of perimeter, let's move inside
          // Save time and count for crude detection "end of field" without mapping
          if ((millis() - lastPerimOut) > 20000)
            lastOutCnt++;
          else
            lastOutCnt = 0;
          lastPerimOut = millis();

          // Backward direction
          subMove((flags & (FL_IGNORE_WIRE | FL_IGNORE_VIRTUAL)) | FL_SAVE_POWER | FL_STOP_INSIDE | FL_IGNORE_SONAR, optBackwardTime,
                  _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
          // If we've got inside - move backward a bit more
          if (subCheckPerimeter(flags))
            subMove(FL_SAVE_POWER | FL_IGNORE_WIRE | FL_IGNORE_VIRTUAL | FL_IGNORE_SONAR, optBackwardTime / 2,
                    _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
          subStop();
            
          if (subCheckPerimeter(flags)) {
            // Ok, we're inside after moving back, let's roll and continue
            // Switching turn direction
            if (lastPerimTurnCount > 3) {
              lastPerimTurnCount = 0;
              if (lastPerimTurn == _dir::LEFT)
                lastPerimTurn = _dir::RIGHT;
              else
                lastPerimTurn = _dir::LEFT;
            }
            lastPerimTurnCount++;
            subRoll(FL_SAVE_POWER | FL_IGNORE_WIRE | FL_IGNORE_VIRTUAL, getRandomRange(optRollTimeMin, optRollTimeMax), lastPerimTurn, INVALID_ANGLE);
            subStop();
          }
          // Perimeter check
          if (!subCheckPerimeter(flags)) {
            lastOutCnt++;
            lastPerimOut = millis();
            
            // We are not inside. Looks like we have to roll around to find the perimeter and move a bit forward
            subPerimSearch((flags & (FL_IGNORE_WIRE | FL_IGNORE_VIRTUAL)) | FL_SAVE_POWER | FL_STOP_INSIDE, 40000);
            // If we haven't got inside the perimeter after rolling - move a bit forward
            if (!subCheckPerimeter(flags))
              subMove((flags & (FL_IGNORE_WIRE | FL_IGNORE_VIRTUAL)) | FL_SAVE_POWER | FL_STOP_INSIDE, optBackwardTime, _dir::FORWARD,
                      INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
            subStop();
          }
          if (angle != INVALID_ANGLE) {
            // calculate new angle to move
            // Too many out events in last minute, let's change direction by 90 degrees
            
            if (lastOutCnt > 2) {
              angle = oImu.scaleDegree(angle + 90);
              lastOutCnt = 0;
            }
            // Change direction by 180 degrees
            angle = oImu.scaleDegree(angle + 180);
            break;
          }
          break;
        }
      }

      // Check if we've arrived at the coordinates
      if (latitude != INVALID_COORD) {
        float maxDist = 3;
        // Check if we're close enough
        if (oGps.reachedDest()) {
          cmdStopReason = FIN_DESTINATION;
          finishSub = true;
          subStop();
          return leftTime;
        }
      }
      if (seekerCheck && oSeeker.reachedDest()) {
        cmdStopReason = FIN_DESTINATION;
        finishSub = true;
        subStop();
        return leftTime;
      }
    }
  }

  if (cmdStopReason == FIN_NONE)
    cmdStopReason = FIN_TIME;
  if (leftTime >= 0)
    return leftTime;
  else
    return 0;
} // uint32_t subMove(uint16_t flags, uint32_t timeout, _dir dirMove, float angle, int32_t latitude, int32_t longitude)

uint32_t cmdStartTime = 0;

// Start new command
void startCommand(uint16_t cmd) {
  if (cmdRun != cmd) {
    cmdRun = cmdNextRun = cmd;
    finishSub = true;
    cmdStartTime = millis();
    cmdStopReason = FIN_NONE;
    // Disable perimeters calculations if needed
    if (cmdFlags & FL_DISABLE_WIRE)
      oPerim.disableThings();
    else
      oPerim.enableThings();
  }
} // void startCommand(uint16_t cmd)

// Modbus interface start new command (data has command code)
void startExtCommand(void *addr, uint32_t data, uint8_t size) {
  startCommand((uint16_t) data);
  // cmdFlags |= FL_SIMPLE_MOVE;
} // void startExtCommand(void *addr, uint32_t data, uint8_t size)

// Yield function with WFI to save power at delays
void yield() {
  asm("wfi");
} // void yield()

// Main loop
void loop() {
  int32_t sLat, sLon;
  int32_t newStartLat, newStartLon;
  int32_t tmpSize;
  int8_t signOffset;
  uint32_t endMowingTime;
  float setLeftRPM, setRightRPM;
  
  if ((millis() - lastDisplay) > 5000) {
    lastDisplay = millis();
    displayStuff();
  }

  subMustCheck();
  finishSub = false;
  // Switch to next command
  cmdRun = cmdNextRun;
  cmdNextRun = CMD_STOP;

  // debug(L_NOTICE, (char *) F("Main loop cycle (%hu:%hx)\n"), cmdRun, cmdFlags);
  switch (cmdRun) {
    case CMD_STOP:
      if ((oMotors.curPWM(0) != 0) || (oMotors.curPWM(1) != 0)) {
        subStop();
      }
      oMow.maxSpeed = 0;
      // Check if mowing height was changed by pfod and set it
      if (((millis() / 1000) % 10 == 0) && (optMowHeight != oMow.heightMow))
          oMow.setHeight(optMowHeight);
      cmdStopReason = FIN_TIME;
      break;

    case CMD_MOVE:
      subMove(cmdFlags, 180000, _dir::FORWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
      subStop();
      if (cmdRun == CMD_MOVE)
        cmdRun = CMD_STOP;
      cmdStopReason = FIN_TIME;
      break;

    case CMD_VIRT_LAWN:
      // Mowed the lawn by virtual perimeter (with emergency return when out of perimeter)
      endMowingTime = millis() + cmdTime;

      while ((cmdRun == CMD_VIRT_LAWN) && (millis() < endMowingTime)) {
        cmdStopReason = FIN_NONE;
        debug(L_INFO, (char *) F("Mowing by virtual perimeter (%d ms left)\n"), endMowingTime - millis());
        finishSub = false;
        subMove(cmdFlags, endMowingTime - millis(), _dir::FORWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        // Check if we're out of perimeter - return to emergency point
        if ((cmdStopReason == FIN_PERIMETER) || (cmdStopReason == FIN_TIME)) {
          finishSub = false;
          debug(L_INFO, (char *) F("Return to emergency point %ld, %ld\n"), virtRLat, virtRLon);
          subMove(cmdFlags | FL_IGNORE_VIRTUAL | FL_IGNORE_WIRE | FL_SAVE_POWER, 360000, _dir::STOP,
                  INVALID_ANGLE, virtRLat, virtRLon);
        }
      }
      debug(L_INFO, (char *) F("Finished mowing, stop reason %d\n"), cmdStopReason);
      subStop();
      if (cmdRun == CMD_VIRT_LAWN)
        cmdRun = CMD_STOP;
      cmdStopReason = FIN_TIME;
      break;

    case CMD_EXT_MOVE_ANGLE:
      if (currCmdAngle == INVALID_ANGLE) {
        debug(L_INFO, (char *) F("Moving forward for %lu ms\n"), cmdTime);
        subMove(cmdFlags, cmdTime, _dir::FORWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
      } else {
        debug(L_INFO, (char *) F("Moving to angle %hd for %lu ms\n"), currCmdAngle, cmdTime);
        subMove(cmdFlags, cmdTime, _dir::STOP, currCmdAngle, INVALID_COORD, INVALID_COORD);
      }
      subStop();
      if (cmdRun == CMD_EXT_MOVE_ANGLE)
        cmdRun = CMD_STOP;
      if (cmdStopReason != FIN_NONE)
        cmdStopReason = FIN_TIME;
      break;

    case CMD_ROS_CMDVEL:
      finishSub = false;
      setLeftRPM = 60 * (forwSpeed - (yawSpeed * oMotors.wheelBase / 2)) / oMotors.distRevolution;
      setRightRPM = 60 * (forwSpeed + (yawSpeed * oMotors.wheelBase / 2)) / oMotors.distRevolution;
      debug(L_INFO, "ros_cmdvel: forw: %f yaw: %f (%f %f)\n", forwSpeed, yawSpeed, setLeftRPM, setRightRPM);
      subSpeedSetup(cmdFlags);
      oMotors.setRPM(setLeftRPM, setRightRPM, ROS_CMDVEL_TIMEOUT);
      while (!finishSub && ((cmdStartTime + cmdTime) > millis())) {
        subMustCheck();
        if (subCheckMotors()) {
          subResetMotors();
          cmdStopReason = FIN_MOTORS;
          break;
        }
        if (subCheckMow()) {
          subResetMow();
          cmdStopReason = FIN_MOWER;
          break;
        }
      }
      if (cmdStopReason == FIN_NONE)
        cmdStopReason = FIN_TIME;
      if (cmdNextRun != CMD_ROS_CMDVEL) {
        cmdRun = CMD_STOP;
        subStop();
      }
      break;

    case CMD_EXT_MOVE_GPS:
    case CMD_ROS_MOVE_GPS:
      // TODO: add flags setting through ros
      cmdFlags |= FL_HIGH_PRECISION | FL_IGNORE_WIRE | FL_IGNORE_VIRTUAL;
      
      finishSub = false;
      if (cmdFlags & FL_HIGH_PRECISION) {
        // Delay until we get precision coordinates
        debug(L_INFO, (char *) F("Waiting for high precision coordinates\n"));
        if ((oGps.numSats < 128) && !finishSub) {
          subMustCheck();
          delayWithROS(300);
          return;
        }
      }
      if (!finishSub) {
        debug(L_INFO, (char *) F("Moving to point %ld %ld (from %ld %ld) for %lu ms\n"),
              currCmdLat, currCmdLon, oGps.latitude, oGps.longitude, cmdTime);
        subMove(cmdFlags, cmdTime, _dir::STOP, INVALID_ANGLE, currCmdLat, currCmdLon);
      }
      subStop();
      // At the end of CMD_ROS_MOVE_GPS - roll to needed angle
      if (cmdRun == CMD_ROS_MOVE_GPS) {
        debug(L_INFO, (char *) F("Rolling to angle %hd for %lu ms\n"), currCmdAngle, cmdTime);
        finishSub = false;
        subRoll(cmdFlags, cmdTime, _dir::STOP, currCmdAngle);
      }
      if ((cmdRun == CMD_EXT_MOVE_GPS) || (cmdRun == CMD_ROS_MOVE_GPS))
        cmdRun = CMD_STOP;
      if (cmdStopReason != FIN_NONE)
        cmdStopReason = FIN_TIME;
      break;

    // Not implemented yet
    case CMD_EXT_MOVE_TAGS:
      cmdRun = CMD_STOP;
      cmdStopReason = FIN_HARDWARE;
      break;

    case CMD_EXT_ROLL_ANGLE:
      debug(L_INFO, (char *) F("Rolling to angle %hd for %lu ms\n"), currCmdAngle, cmdTime);
      subRoll(cmdFlags, cmdTime, _dir::STOP, currCmdAngle);
      subStop();
      if (cmdRun == CMD_EXT_ROLL_ANGLE)
        cmdRun = CMD_STOP;
      if (cmdStopReason != FIN_NONE)
        cmdStopReason = FIN_TIME;
      break;

    case CMD_EXT_REVERSE:
      debug(L_INFO, (char *) F("Moving to backward for %lu ms\n"), cmdTime);
      subMove(cmdFlags, cmdTime, _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
      subStop();
      if (cmdRun == CMD_EXT_REVERSE)
        cmdRun = CMD_STOP;
      if (cmdStopReason != FIN_NONE)
        cmdStopReason = FIN_TIME;
      break;

    case CMD_EXT_ROLL_PERIMETER:
      debug(L_INFO, (char *) F("Rolling around to find perimeter for %lu ms\n"), cmdTime);
      subPerimSearch(cmdFlags, cmdTime);
      subStop();
      if (cmdRun == CMD_EXT_ROLL_PERIMETER)
        cmdRun = CMD_STOP;
      if (cmdStopReason != FIN_NONE)
        cmdStopReason = FIN_TIME;
      break;

    // Not implemented yet
    case CMD_EXT_MOVE_PERIMETER:
      subStop();
      if (cmdRun == CMD_EXT_MOVE_PERIMETER)
        cmdRun = CMD_STOP;
      if (cmdStopReason != FIN_NONE)
        cmdStopReason = FIN_TIME;
      break;

    case CMD_EXT_CALIB_START:
      resetCompassCalib();
      cmdRun = CMD_STOP;
      cmdStopReason = FIN_TIME;
      break;

    case CMD_EXT_CALIB_FINISH:
      stopCompassCalib();
      cmdRun = CMD_STOP;
      cmdStopReason = FIN_TIME;
      break;

    case CMD_EXT_SAVE_NVMEM:
      loadSaveVars(true);
      cmdRun = CMD_STOP;
      cmdStopReason = FIN_TIME;
      break;

    case CMD_NORTH:
    case CMD_SOUTH:
    case CMD_EAST:
    case CMD_WEST:
      int16_t angle;
      
      switch (cmdRun) {
        case CMD_NORTH:
          angle = 0;
          break;
        case CMD_SOUTH:
          angle = 180;
          break;
        case CMD_EAST:
          angle = 90;
          break;
        case CMD_WEST:
          angle = -90;
          break;
      }
      subMove(cmdFlags, 180000, _dir::STOP, angle, INVALID_COORD, INVALID_COORD);
      subStop();
      if ((cmdStopReason == FIN_TIME) || finishSub)
        cmdRun = CMD_STOP;
      break;

    case CMD_MOVE_SEEKER:
      debug(L_INFO, (char *) F("Moving to chessboard\n"));
      subMove(cmdFlags, 360000, _dir::STOP, -INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
      subStop();
      if (cmdRun == CMD_MOVE_SEEKER)
        cmdRun = CMD_STOP;
      cmdStopReason = FIN_TIME;
      break;

    case CMD_FIND_PERIMETER:
      // Find perimeter
      subMove(cmdFlags | FL_STOP_OUTSIDE, 180000, _dir::FORWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
      // Move by perimeter wire, ignoring sonar (only bumper)
      if (cmdRun == CMD_FIND_PERIMETER)
        subMove(cmdFlags | FL_IGNORE_SONAR, 1800000, _dir::STOP, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
      if (cmdRun == CMD_FIND_PERIMETER)
        cmdRun = CMD_STOP;
      break;

    case CMD_POINT:
      ledState = 2;
      if (cmdFlags & FL_HIGH_PRECISION) {
        // Delay until we get precision coordinates
        if (oGps.numSats < 128) {
          subMustCheck();
          delayWithROS(10);
          return;
        }
      }
      ledState = 1;
      debug(L_INFO, (char *) F("Moving to point %ld %ld (from %ld %ld)\n"), pointLat, pointLon, oGps.latitude, oGps.longitude);
      finishSub = false;
      subMove(cmdFlags | FL_IGNORE_WIRE, 100000, _dir::STOP, INVALID_ANGLE, pointLat, pointLon);
      subStop();
      if (cmdRun == CMD_POINT)
        cmdRun = CMD_STOP;
      cmdStopReason = FIN_TIME;
      break;

    case CMD_HOME:
      ledState = 2;
      if (cmdFlags & FL_HIGH_PRECISION) {
        // Delay until we get precision coordinates
        if (oGps.numSats < 128) {
          subMustCheck();
          delayWithROS(10);
          return;
        }
      }
      ledState = 1;
      debug(L_INFO, (char *) F("Home - moving to first %d %d (from %d %d)\n"), homeFLat, homeFLon, oGps.latitude, oGps.longitude);
      finishSub = false;
      // Move to first point
      subMove(cmdFlags | FL_IGNORE_WIRE | FL_SAVE_POWER, 4000000, _dir::STOP, INVALID_ANGLE, homeFLat, homeFLon);
      debug(L_INFO, (char *) F("Home - stop reason %d\n"), cmdStopReason);
      if (cmdStopReason != FIN_DESTINATION) {
        cmdRun = CMD_STOP;
        break;
      }
      debug(L_INFO, (char *) F("Home - moving to second %d %d (from %d %d)\n"), homeLLat, homeLLon, oGps.latitude, oGps.longitude);
      finishSub = false;
      // Move to second point (close to the base)
      subMove(cmdFlags | FL_IGNORE_WIRE | FL_SAVE_POWER, 4000000, _dir::STOP, INVALID_ANGLE, homeLLat, homeLLon);
      if (cmdStopReason != FIN_DESTINATION) {
        cmdRun = CMD_STOP;
        break;
      }
      debug(L_INFO, (char *) F("Home - turning to direction %d (from %d)\n"), (int) homeDir, (int) oImu.readCurDegree(-1));
      finishSub = false;
      subRoll(cmdFlags |FL_IGNORE_WIRE | FL_SAVE_POWER, 100000, _dir::STOP, homeDir);
      if (cmdStopReason != FIN_DESTINATION) {
        cmdRun = CMD_STOP;
        break;
      }
      debug(L_INFO, (char *) F("Home - going with seeker\n"));
      finishSub = false;
      subMove(cmdFlags | FL_IGNORE_WIRE | FL_SAVE_POWER | FL_IGNORE_SONAR | FL_IGNORE_BUMPER, 50000, _dir::STOP, -INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
      finishSub = false;
      // Check if we're at right angle for the station. If not, then go back
      if ((cmdStopReason != FIN_DESTINATION) || (abs(oSeeker.offsetAngle) > 12)) {
        debug(L_INFO, (char *) F("Home - wrong angle! Moving back."));
        subMove(cmdFlags | FL_IGNORE_WIRE | FL_SAVE_POWER, 10000, _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        finishSub = false;
        subMove(cmdFlags | FL_IGNORE_WIRE | FL_SAVE_POWER, 4000000, _dir::STOP, INVALID_ANGLE, homeLLat, homeLLon);
        finishSub = false;
        subMove(cmdFlags | FL_IGNORE_WIRE | FL_SAVE_POWER, 4000000, _dir::STOP, INVALID_ANGLE, homeFLat, homeFLon);
        cmdRun = CMD_STOP;
        break;
      }
      debug(L_INFO, (char *) F("Home - parking\n"));
      subMove(cmdFlags | FL_IGNORE_WIRE | FL_SAVE_POWER | FL_IGNORE_SONAR | FL_IGNORE_BUMPER, 20000, _dir::FORWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
      finishSub = false;
      if (cmdStopReason != FIN_CHARGE) {
        debug(L_INFO, (char *) F("Home - no charge voltage! Moving back."));
        subMove(cmdFlags | FL_IGNORE_WIRE | FL_SAVE_POWER, 10000, _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        finishSub = false;
        subMove(cmdFlags | FL_IGNORE_WIRE | FL_SAVE_POWER, 4000000, _dir::STOP, INVALID_ANGLE, homeLLat, homeLLon);
        finishSub = false;
        subMove(cmdFlags | FL_IGNORE_WIRE | FL_SAVE_POWER, 4000000, _dir::STOP, INVALID_ANGLE, homeFLat, homeFLon);
        cmdRun = CMD_STOP;
      }
      break;

    case CMD_ZIGZAG:
      // process square in zigzag pattern
      // Start coordinates.
      ledState = 2;
      // Delay until we get precision coordinates
      if (oGps.numSats < 128) {
        subMustCheck();
        delayWithROS(10);
        return;
      }
      ledState = 1;
      // If we have set coordinates, move to start point first. Otherwise - use current coordinates as start point
      if (currCmdLat != INVALID_COORD) {
        sLat = currCmdLat;
        sLon = currCmdLon;
        subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION, 180000, _dir::STOP, INVALID_ANGLE, sLat, sLon);
      } else {
        sLat = oGps.latitude;
        sLon = oGps.longitude;
      }

      // Choose direction
      if (currCmdAngle == 0)
        signOffset = 1;
      else
        signOffset = -1;

      debug(L_INFO, (char *) F("Square start, initial position %ld %ld\n"), sLat, sLon);
      for (uint16_t i = 1; i <= optSquareCycles; i++) {
        if (cmdRun != CMD_ZIGZAG)
          break;
        finishSub = false;
#ifdef ZIGZAG_EASTWEST
        if ((i % 2) == 1)
          subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION, 100000, _dir::STOP, INVALID_ANGLE, sLat + signOffset * i * optSquareInc + optSquareOffset, sLon + optSquareSize);
        else
          subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION, 100000, _dir::STOP, INVALID_ANGLE, sLat + signOffset * i * optSquareInc, sLon);
#else
        if ((i % 2) == 1)
          subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION, 100000, _dir::STOP, INVALID_ANGLE, sLat + optSquareSize, sLon + signOffset * i * optSquareInc + optSquareOffset);
        else
          subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION, 100000, _dir::STOP, INVALID_ANGLE, sLat, sLon + signOffset * i * optSquareInc);
#endif
      }
      finishSub = false;
      if (cmdRun == CMD_ZIGZAG) {
        debug(L_INFO, (char *) F("Square done, returning to starting position %ld %ld\n"), sLat, sLon);
        subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION | FL_SAVE_POWER, 100000, _dir::STOP, INVALID_ANGLE, sLat, sLon);
      }
      if (cmdRun == CMD_ZIGZAG)
        cmdRun = CMD_STOP;
      break;

    case CMD_SPIRAL:
      ledState = 2;
      // Delay until we get precision coordinates
      if (oGps.numSats < 128) {
        subMustCheck();
        delayWithROS(10);
      }
      ledState = 1;
      sLat = oGps.latitude;
      sLon = oGps.longitude;
      
      debug(L_INFO, (char *) F("Spiral start, initial position %ld %ld\n"), sLat, sLon);
      newStartLat = sLat;
      newStartLon = sLon;
      tmpSize = optSquareSize;
      
      while (tmpSize > 0) {
        for (uint16_t i = 1; i <= 4; i++) {
          if (cmdRun != CMD_SPIRAL)
            break;
          finishSub = false;
          switch(i) {
            case 1:
              subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION, 100000, _dir::STOP, INVALID_ANGLE, newStartLat + tmpSize, newStartLon);
              break;
            case 2:
              subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION, 100000, _dir::STOP, INVALID_ANGLE, newStartLat + tmpSize, newStartLon + tmpSize - optSquareOffset / 4);
              break;
            case 3:
              subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION, 100000, _dir::STOP, INVALID_ANGLE, newStartLat, newStartLon + tmpSize - optSquareOffset / 4);
              break;
            case 4:
              subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION, 100000, _dir::STOP, INVALID_ANGLE, newStartLat, newStartLon + optSquareOffset / 2);
              break;
          }
        }
        newStartLat += optSquareOffset / 2;
        newStartLon += optSquareOffset / 2;
        tmpSize -= optSquareOffset / 2;
      }
      finishSub = false;
      if (cmdRun == CMD_SPIRAL) {
        debug(L_INFO, (char *) F("Spiral done, returning to starting position %ld %ld\n"), sLat, sLon);
        subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION | FL_SAVE_POWER, 100000, _dir::STOP, INVALID_ANGLE, sLat, sLon);
      }
      if (cmdRun == CMD_SPIRAL)
        cmdRun = CMD_STOP;
      cmdStopReason = FIN_TIME;
      break;

    case CMD_MODE_POWERSAVE:
      if (!oSwitches.readSensor(T_EXTLED1)) {
        debug(L_INFO, (char *) F("Entering power save mode for %u (%u %u) ms\n"), cmdTime, cmdStartTime, millis());
        delayWithROS(500);
        oROS.pause();
        oMotors.disableThings();
        oSonars.disableThings();
        oPerim.disableThings();
        oPower.disableThings();
        oMow.disableThings();
        oSwitches.setLED(T_EXTLED1, true);
      }
      // Stop power save mode when time come
      while ((cmdStartTime + cmdTime) > millis()) {
        subMustCheck();
      }
      cmdRun = CMD_MODE_NORMAL;
      cmdStopReason = FIN_TIME;
      break;

    case CMD_MODE_NORMAL:
      oROS.unpause();
      delay(50);
      debug(L_INFO, (char *) F("Exiting power save mode\n"));
      oMotors.enableThings();
      oSonars.enableThings();
      oPerim.enableThings();
      oPower.enableThings();
      oMow.enableThings();
      oSwitches.setLED(T_EXTLED1, false);
      delayWithROS(50);
      cmdRun = CMD_STOP;
      cmdStopReason = FIN_TIME;
      break;

    case CMD_SHUTDOWN:
      // Check if time to shutdown came
      if ((cmdStartTime + cmdTime) < millis()) {
        // Sometimes relay do stuck, we have to pulse it few times
        for (uint8_t i = 0; i < 32; i++) {
          oPower.emergShutdown();
          delayWithROS(2000);
        }
        cmdRun = CMD_STOP;
      }
      delay(5);
      break;

    case CMD_ERROR:
      delayWithROS(100);
      cmdStopReason = FIN_TIME;
      break;
  }

  if (!finishSub && (cmdRun != CMD_STOP)) {
    debug(L_NOTICE, "out of main loop with command %d running!\n", cmdRun);
    oSwitches.setLED(T_BEEPER, true);
    oSwitches.setLED(T_EXTLED2, true);
    delayWithROS(1000);
    oSwitches.setLED(T_BEEPER, false);
    oSwitches.setLED(T_EXTLED2, false);
  }
} // loop()
