// OMower demo command functions
// $Id$

#include "OMower_Simple.h"
#include "robot.h"
#include "pfod.h"

// Commands labels
const char *cmdLabels[] = { "STOP", "MOVE", "FIND_PERIMETER", "ZIGZAG", "NORTH", "SOUTH", "EAST", "WEST", "SPIRAL", "ERROR" };

// Current command running
uint16_t cmdRun = CMD_STOP;

// Current command's flags
uint16_t cmdFlags = 0;

// Finish sub-command (new command received or serious hardware error)
boolean finishSub = false;

// Last turn direction at perimeter
_dir lastPerimTurn = _dir::LEFT;

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

uint32_t lastDisplay = 0;
uint32_t lastLoop = 0;

// Check important stuff (battery, commands received, GPS serial port), reset IMU if needed
void subMustCheck() {
  lastLoop = millis();
  subCheckConsole();
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
  return (oMotors.softError() == _hwstatus::ERROR);
} // boolean subCheckMotors()

// Check overload/fails on mowing motor
boolean subCheckMow() {
  return (oMow.softError() == _hwstatus::ERROR);
} // boolean subCheckMow()

// Reset mowing motor driver
void subResetMow() {
  debug(L_NOTICE, (char *) F("subResetMow: resetting mowing motor driver\n"));
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
  subWaitMotorsStop();
  return millis() - stopTime;
} // uint32_t subStop(uint16_t flags)

// Reset motors driver
void subResetMotors() {
  debug(L_NOTICE, (char *) F("subResetMotors: resetting motor drivers\n"));
  oMotors.emergStop();
  subWaitMotorsStop();
  oMotors.init();
  delay(100);  
} // void subResetMotors()



// Check rain sensor, switch to station return if detected
void subCheckRain() {

} // void subCheckRain()

// Check drop sensor, emergency stop if detected
void subCheckDrop() {
  
} // void subCheckDrop()

// Check sonars and return direction of which sonar hit
_dir subCheckSonar() {
  // Check center sonar if we do have it
  if (((oSonars.locThings() == _locationThings::JUSTONE)
       || (oSonars.locThings() == _locationThings::LEFT_CENTER_RIGHT))
      && ((oSonars.readSonar(1) > 0) && (oSonars.readSonar(1) < oSonars.triggerDist))) {
    debug(L_INFO, (char *) F("subCheckSonar: center %hu\n"), oSonars.readSonar(1));
    return _dir::FORWARD;
  }
  // Check left forward sonar
  if ((oSonars.readSonar(0) > 0) && (oSonars.readSonar(0) < oSonars.triggerDist)) {
    debug(L_INFO, (char *) F("subCheckSonar: left %hu\n"), oSonars.readSonar(0));
    return _dir::LEFT;
  }
  // Check right forward sonar
  if ((oSonars.readSonar(2) > 0) && (oSonars.readSonar(2) < oSonars.triggerDist)) {
    debug(L_INFO, (char *) F("subCheckSonar: right %hu\n"), oSonars.readSonar(2));
    return _dir::RIGHT;
  }
  return _dir::STOP;
} // _dir subCheckSonar()

// Check bumpers and return direction of which bumper hit
_dir subCheckBumper() {
  return _dir::STOP;
} // _dir subCheckBumper()

// Subcommand to roll, returns execution time in milliseconds
// dirMove == _dir::STOP - use angle
// angle == INVALID_ANGLE - use dirMove
uint32_t subRoll(uint16_t flags, uint32_t timeout, _dir dirMove, int16_t angle) {
  uint32_t subStart = millis();
  uint32_t subEnd = millis() + timeout;
  int32_t leftTime = timeout;

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
        oImu.setCourse(angle);
        oMotors.rollCourse((navThing *) &oImu, leftTime);
      } else
        return leftTime;
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
        // Move a bit backward in case if this is caused by an obstacle
        subMove(flags | FL_IGNORE_SONAR | FL_IGNORE_BUMPER, optBackwardTime, _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        break;
      }
      if (!(flags & FL_IGNORE_WIRE)) {
        if (oPerim.inside(-1)) {
          // Check special stop condition
          if (flags & FL_STOP_INSIDE) {
            subStop();
            return leftTime;
          }
        } else {
          // Check special stop condition
          if (flags & FL_STOP_OUTSIDE) {
            subStop();
            return leftTime;
          }
        }
      }
    }
    // Calculate time left
    leftTime = (int32_t) subEnd - (int32_t) millis();
  }

  if (leftTime >= 0)
    return leftTime;
  else
    return 0;
} // uint32_t subRoll(uint16_t flags, uint32_t timeout, _dir dirMove, int16_t angle)

// Roll to perimeter's maximum signal strength (first roll around, then roll to maximum of this)
uint32_t subPerimSearch(uint16_t flags, uint32_t timeout) {
  uint32_t subStart = millis();
  uint32_t subEnd = millis() + timeout;
  int32_t leftTime = timeout;
  uint32_t lastMove;

  debug(L_INFO, (char *) F("subPerimSearch: flags %hx, timeout %lu\n"),
        flags, timeout);
  while (!finishSub && (millis() < subEnd)) {
    // Set speed of wheel and mowing motors
    subSpeedSetup(flags);

    // Start rolling
    oPerim.setTracking(false);
    oMotors.rollCourse((navThing *) &oPerim, leftTime);
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
        // Move a bit backward in case if this is caused by an obstacle
        subMove(flags | FL_IGNORE_SONAR | FL_IGNORE_BUMPER, optBackwardTime, _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        break;
      }
      if (!(flags & FL_IGNORE_WIRE)) {
        if (oPerim.inside(-1)) {
          // Check special stop condition
          if (flags & FL_STOP_INSIDE) {
            subStop();
            return leftTime;
          }
        } else {
          // Check special stop condition
          if (flags & FL_STOP_OUTSIDE) {
            subStop();
            return leftTime;
          }
        }
      }
      // Check if we weren't move for 2 seconds
      if ((oMotors.curPWM(0) == 0) && (oMotors.curPWM(1) == 0)) {
        if ((millis() - lastMove) > 2000)
          return leftTime;
      } else
        lastMove = millis();
    }
    // Calculate time left
    leftTime = (int32_t) subEnd - (int32_t) millis();
  }

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
  int32_t latNext, lonNext;

  debug(L_INFO, (char *) F("subMove: flags %hx, timeout %lu, dirMove %hd, angle %hd, latitude %ld, longitude %ld\n"),
        flags, timeout, dirMove, angle, latitude, longitude);
  while (!finishSub && (millis() < subEnd)) {
    // Set speed of wheel and mowing motors
    subSpeedSetup(flags);
     
    // Start moving
    if (dirMove != _dir::STOP) {
      oMotors.move(dirMove, leftTime);
    } else {
      if (angle != INVALID_ANGLE) {
        // Move by an angle (IMU)
        if ((flags & FL_IGNORE_IMU) || (oImu.softError() != _hwstatus::ONLINE))
          // Well, we have to ignore IMU or it is not available, let's just stop
          return leftTime;
        // Initialize movement by an angle
        oImu.setCourse(angle);
        oMotors.moveCourse((navThing *) &oImu, leftTime);
      } else
        if (latitude != INVALID_COORD) {
          // Move to GPS/tags coordinates
          // GPS is not functioning, just stop
          if (oGps.softError() != _hwstatus::ONLINE)
            return leftTime;
          // Initialize movement by the GPS
          oGps.setTarget(latitude, longitude, (flags & FL_HIGH_PRECISION) > 0, true);
          oMotors.moveCourse((navThing *) &oGps, leftTime);
        } else {
          if ((flags & FL_IGNORE_WIRE) && (flags & FL_IGNORE_VIRTUAL))
            oMotors.move(_dir::STOP, leftTime);
          else {
            // Initialize movement by perimeter
            perimTrack = true;
            oPerim.setTracking(true);
            oMotors.moveCourse((navThing *) &oPerim, leftTime);
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
        // Move a bit backward in case if this is caused by an obstacle
        subMove(flags | FL_IGNORE_SONAR | FL_IGNORE_BUMPER, optBackwardTime, _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        break;
      }

      // Check and reset mowing motor if needed
      if (subCheckMow()) {
        subResetMow();
      }

      // Check sonar and bumper sensors
      if ((!(flags & FL_IGNORE_SONAR) && ((resSonar = subCheckSonar()) != _dir::STOP)) || (!(flags & FL_IGNORE_BUMPER) && ((resBumper = subCheckBumper()) != _dir::STOP))) {
        _dir dirRoll;

        debug(L_NOTICE, (char *) F("Found obstacle, moving around\n"));
        subStop();
        // Calculate coordinates of the next point after moving around the obstacle (2 meters away)
        if (latitude != INVALID_COORD)
          oGps.calcPoint(2.0, latNext, lonNext, latitude, longitude);
        subMove(flags | FL_STOP_OUTSIDE | FL_IGNORE_SONAR | FL_IGNORE_BUMPER, optBackwardTime, _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        subStop();
        // Try to roll in other direction than an obstacle
        if ((resSonar == _dir::LEFT) || (resBumper == _dir::LEFT))
          dirRoll = _dir::RIGHT;
        else {
          dirRoll = _dir::LEFT;
        }
        subRoll(flags | FL_STOP_OUTSIDE | FL_IGNORE_SONAR, getRandomRange(optRollTimeMin, optRollTimeMax), dirRoll, INVALID_ANGLE);
        subMove(flags | FL_STOP_OUTSIDE | FL_IGNORE_SONAR, getRandomRange(optRollTimeMin, optRollTimeMax), _dir::FORWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
        subStop();
        // Return to way point
        if (latitude != INVALID_COORD) {
          subMove(flags, 30, _dir::STOP, INVALID_ANGLE, latNext, lonNext);
          oGps.setTarget(latitude, longitude, (flags & FL_HIGH_PRECISION) > 0, true);
          oMotors.moveCourse((navThing *) &oGps, leftTime);
        }
        break;
      }

      // Check wire perimeter
      if (!(flags & FL_IGNORE_WIRE)) {
        if (oPerim.inside(-1)) {
          // Check special stop condition
          if (flags & FL_STOP_INSIDE) {
            debug(L_NOTICE, (char *) F("subMove: Inside perimeter, stop\n"));
            subStop();
            return leftTime;
          }
        } else {
          // We know that we're outside, continue
          if ((flags & FL_STOP_INSIDE) || perimTrack)
            continue;
          // Perform stop when reached out of perimeter
          debug(L_NOTICE, (char *) F("subMove: Out of perimeter %hd (%f) %hd (%f), stop\n"), oPerim.magnitude(0), oPerim.quality(0), oPerim.magnitude(1), oPerim.quality(1));
          subStop();
          // Check special stop condition
          if (flags & FL_STOP_OUTSIDE)
            return leftTime;
            
          // Well, we've got out of perimeter, let's move inside
          // Save time and count for crude detection "end of field" without mapping
          if ((millis() - lastPerimOut) > 20000)
            lastOutCnt++;
          else
            lastOutCnt = 0;
          lastPerimOut = millis();

          // Backward direction
          subMove(FL_SAVE_POWER | FL_STOP_INSIDE | FL_IGNORE_SONAR, optBackwardTime, _dir::BACKWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
          subStop();
            
          if (oPerim.inside(-1)) {
            // Ok, we're inside after moving back, let's roll and continue
            // Switching turn direction
            if (lastPerimTurn == _dir::LEFT)
              lastPerimTurn = _dir::RIGHT;
            else
              lastPerimTurn = _dir::LEFT;
            subRoll(FL_SAVE_POWER | FL_IGNORE_WIRE, getRandomRange(optRollTimeMin, optRollTimeMax), lastPerimTurn, INVALID_ANGLE);
            subStop();
          }
          if (!oPerim.inside(-1)) {
            lastOutCnt++;
            lastPerimOut = millis();
            
            // We are not inside. Looks like we have to roll around to find the perimeter and move a bit forward
            subPerimSearch(FL_SAVE_POWER | FL_STOP_INSIDE, 40000);
            subMove(FL_SAVE_POWER | FL_STOP_INSIDE, optBackwardTime * 3, _dir::FORWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
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
          finishSub = true;
          subStop();
          break;
        }
      }
    }
  }

  if (leftTime >= 0)
    return leftTime;
  else
    return 0;
} // uint32_t subMove(uint16_t flags, uint32_t timeout, _dir dirMove, float angle, int32_t latitude, int32_t longitude)

uint32_t cmdStartTime = 0;

// Start new command
void startCommand(uint16_t cmd) {
  if (cmdRun != cmd) {
    cmdRun = cmd;
    finishSub = true;
    cmdStartTime = millis();
  }
} // void startCommand(uint16_t cmd)

// Main loop
void loop() {
  int32_t sLat, sLon;
  int32_t newStartLat, newStartLon;
  int32_t tmpSize;
  
  if ((millis() - lastDisplay) > 5000) {
    lastDisplay = millis();
    displayStuff();
  }

  subMustCheck();
  finishSub = false;

  // debug(L_DEBUG, (char *) F("Main loop cycle (%hu:%hx)\n"), cmdRun, cmdFlags);
  switch (cmdRun) {
    case CMD_STOP:
      if ((oMotors.curPWM(0) != 0) || (oMotors.curPWM(1) != 0)) {
        subStop();
      }
      oMow.maxSpeed = 0;
      // Check if mowing height was changed by pfod and set it
      if (((millis() / 1000) % 10 == 0) && (optMowHeight != oMow.heightMow))
          oMow.setHeight(optMowHeight);
      break;
    case CMD_MOVE:
      if (cmdFlags & FL_IGNORE_IMU)
        subMove(cmdFlags, 180000, _dir::FORWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
      else
        subMove(cmdFlags, 180000, _dir::STOP, oImu.readCurDegree(0), INVALID_COORD, INVALID_COORD);
      subStop();
      cmdRun = CMD_STOP;
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
      cmdRun = CMD_STOP;
      break;
    case CMD_FIND_PERIMETER:
      // Find perimeter
      subMove(cmdFlags | FL_STOP_OUTSIDE, 180000, _dir::FORWARD, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
      // Move by perimeter wire, ignoring sonar (only bumper)
      if (cmdRun == CMD_FIND_PERIMETER)
        subMove(cmdFlags | FL_IGNORE_SONAR, 1800000, _dir::STOP, INVALID_ANGLE, INVALID_COORD, INVALID_COORD);
      break;
    case CMD_POINT:
      ledState = 2;
      if (cmdFlags & FL_HIGH_PRECISION) {
        // Delay until we get precision coordinates
        if (oGps.numSats < 128) {
          subMustCheck();
          delay(10);
          return;
        }
      }
      ledState = 1;
      debug(L_INFO, (char *) F("Moving to point %ld %ld (from %ld %ld)\n"), pointLat, pointLon, oGps.latitude, oGps.longitude);
      finishSub = false;
      subMove(cmdFlags | FL_IGNORE_WIRE, 100000, _dir::STOP, INVALID_ANGLE, pointLat, pointLon);
      subStop();
      cmdRun = CMD_STOP;
      break;
    case CMD_ZIGZAG:
      // process square in zigzag pattern
      // Start coordinates.
      ledState = 2;
      // Delay until we get precision coordinates
      if (oGps.numSats < 128) {
        subMustCheck();
        delay(10);
        return;
      }
      ledState = 1;
      sLat = oGps.latitude;
      sLon = oGps.longitude;

      debug(L_INFO, (char *) F("Square start, initial position %ld %ld\n"), sLat, sLon);
      for (uint16_t i = 1; i <= optSquareCycles; i++) {
        if (cmdRun != CMD_ZIGZAG)
          break;
        finishSub = false;
        if ((i % 2) == 1)
          subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION, 100000, _dir::STOP, INVALID_ANGLE, sLat + i * optSquareInc + optSquareOffset, sLon + optSquareSize);
        else
          subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION, 100000, _dir::STOP, INVALID_ANGLE, sLat + i * optSquareInc, sLon);
      }
      finishSub = false;
      if (cmdRun == CMD_ZIGZAG) {
        debug(L_INFO, (char *) F("Square done, returning to starting position %ld %ld\n"), sLat, sLon);
        subMove(cmdFlags | FL_IGNORE_WIRE | FL_HIGH_PRECISION | FL_SAVE_POWER, 100000, _dir::STOP, INVALID_ANGLE, sLat, sLon);
      }
      break;
    case CMD_SPIRAL:
      ledState = 2;
      // Delay until we get precision coordinates
      if (oGps.numSats < 128) {
        subMustCheck();
        delay(10);
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
      break;
    case CMD_ERROR:
      delay(100);
      break;
  }

  if (cmdRun != CMD_STOP) {
    oSwitches.setLED(T_BEEPER, true);
    oSwitches.setLED(T_EXTLED2, true);
    delay(1000);
    oSwitches.setLED(T_BEEPER, false);
    oSwitches.setLED(T_EXTLED2, false);
  }
} // loop()