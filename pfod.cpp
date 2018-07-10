// OMower simple pfodApp interface
// $Id$

#include "OMower_Simple.h"
#include "robot.h"
#include "pfod.h"
#include "modbus.h"
#include <string.h>

// Receive buffer for pfod command
uint8_t pfodRcvBuf[64];
uint32_t pfodLen = 0;

// current menu
uint16_t currentMenu = 0;

// last menu update
uint32_t lastPfodUpdate = 0;

// test points coordinates
int32_t tLat[4] = {INVALID_COORD, INVALID_COORD, INVALID_COORD, INVALID_COORD};
int32_t tLon[4] = {INVALID_COORD, INVALID_COORD, INVALID_COORD, INVALID_COORD};

void sendSlider(String cmd, String title, float value, String unit, float scale, float maxValue, float minValue) {
  oSerial.printf(PFODAPP_PORT, (char *) F("|%s`%d~%s ~%s`%d`%d~%f~%f"), cmd.c_str(), (int) (value / scale), title.c_str(), unit.c_str(), 
        (int) (maxValue / scale), (int) (minValue / scale), maxValue, minValue);
} // void sendSlider(String cmd, String title, float value, String unit, float scale, float maxValue, float minValue)

void sendLabel(String cmd, String title) {
  oSerial.printf(PFODAPP_PORT, (char *) F("|!%s~%s"), cmd.c_str(), title.c_str());
} // void sendLabel(String title)

void sendFloat(String cmd, String title, float value) {
  oSerial.printf(PFODAPP_PORT, (char *) F("|!%s~%s=%f"), cmd.c_str(), title.c_str(), value);
} // void sendFloat(String title, float value)

void sendLong(String cmd, String title, int32_t value) {
  oSerial.printf(PFODAPP_PORT, (char *) F("|!%s~%s=%ld"), cmd.c_str(), title.c_str(), value);
} // void sendLong(String cmd, String title, int32_t value)

void sendButton(String cmd, String title) {
  oSerial.printf(PFODAPP_PORT, (char *) F("|%s~%s"), cmd.c_str(), title.c_str());
}

void sendSwitch(String cmd, String title, boolean state) {
  oSerial.printf(PFODAPP_PORT, (char *) F("|%s`%d~%s ~~OFF\\ON"), cmd.c_str(), state ? 1 : 0, title.c_str());
} // void sendSwitch(String cmd, String title, boolean state)

void sendMenuStart(boolean update, String title) {
  oSerial.printf(PFODAPP_PORT, "{%c%s`1000", update ? ',' : ',', title.c_str());
} // void sendMenuStart(boolean update, String title)

void sendMenuFinish() {
  oSerial.printf(PFODAPP_PORT, "}\n");
} // void sendMenuFinish()

// Display settings menu
void sendSettingsMenu(boolean update) {
  debug(L_INFO, (char *) F("pfod.cpp: sending settings menu\n"));
  sendMenuStart(update, F("Settings"));
  
  sendButton("m00", F("Main menu"));
  sendButton("r10", F("Save settings"));
  sendButton("r11", F("Load settings"));
  sendButton("m03", F("Motors settings"));
  sendButton("m04", F("Mowing settings"));
  sendButton("m07", F("Navigation settings"));
  sendButton("m05", F("Other settings"));
  sendMenuFinish();
} // void sendSettingsMenu(boolean update)

// Display wheel motors settings
void sendMotorsSettingsMenu(boolean update) {
  debug(L_INFO, (char *) F("pfod.cpp: sending motors settings menu\n"));
  sendMenuStart(update, F("Motors settings"));
  sendButton("m00", F("Main menu"));
  sendSlider("s01", F("Wheel acceleration"), oMotors.accel, "", 1, SPEED_MAX, 1);
  sendSlider("s02", F("Wheel stop deaccel."), oMotors.accelStop, "", 1, SPEED_MAX, 1);
  sendSlider("s03", F("Wheel maximum PWM"), oMotors.maxPWM, "", 1, SPEED_MAX, 1);
  sendSlider("s04", F("Wheel minimum PWM"), oMotors.minPWM, "", 1, SPEED_MAX, 1);
  sendSlider("s05", F("Wheel roll P"), oMotors.pidRollP, "", 0.1, 64, 0);
  sendSlider("s06", F("Wheel roll I"), oMotors.pidRollI, "", 0.1, 16, 0);
  sendSlider("s07", F("Wheel roll D"), oMotors.pidRollD, "", 0.1, 16, 0);
  sendSlider("s08", F("Wheel move P"), oMotors.pidMoveP, "", 0.1, 64, 0);
  sendSlider("s09", F("Wheel move I"), oMotors.pidMoveI, "", 0.1, 16, 0);
  sendSlider("s10", F("Wheel move D"), oMotors.pidMoveD, "", 0.1, 16, 0);
  sendSlider("s11", F("Wheel current max"), oCrrMotors.currentMax, "A", 0.1, 8, 0.1);
  sendMenuFinish();
} // void sendMotorsSettingsMenu(boolean update)

void sendMowingSettingsMenu(boolean update) {
  debug(L_INFO, (char *) F("pfod.cpp: sending mowing settings menu\n"));
  sendMenuStart(update, F("Mowing settings"));
  sendButton("m00", F("Main menu"));
  sendSlider("s12", F("Mowing current max"), oCrrMow.currentMax, "A", 0.1, 20, 0.1);
  sendSlider("s13", F("Mowing acceleration"), oMow.accel, "", 1, SPEED_MAX, 1);
  sendSlider("s14", F("Mowing stop deaccel."), oMow.accelStop, "", 1, SPEED_MAX, 1);
  sendSlider("s15", F("Mowing modulate speed"), oMow.modulateSpeed, "", 1, SPEED_MAX, 0);
  sendSlider("s16", F("Mowing modulate period"), oMow.modulatePeriod, "ms", 100, 5000, 500);
  sendSlider("s17", F("Mowing min height"), oMow.heightMowMin, "", 1, 100, 1);
  sendSlider("s18", F("Mowing maximum PWM"), oMow.maxPWM, "", 1, SPEED_MAX, 1);
  sendSlider("s54", F("Mowing height"), optMowHeight, "mm", 1, oMow.heightMowMin, 1);
  sendMenuFinish();
} // void sendMowingSettingsMenu(boolean update)

void sendOtherSettingsMenu(boolean update) {
  debug(L_INFO, (char *) F("pfod.cpp: sending other settings menu\n"));
  sendMenuStart(update, F("Other settings"));
  sendButton("m00", F("Main menu"));
  sendSlider("s19", F("Perimeter minimum quality"), oPerim.minQuality, "", 0.1, 2.0, 1.0);
  sendSlider("s20", F("Perimeter timeout"), oPerim.errorTimeout, "", 1, 200, 0);
  sendSlider("s21", F("Sonars trigger distance"), oSonars.triggerDist, "", 1, 200, 1);
  sendSwitch("s22", F("Perimeter invert"), oPerim.invertMag);
  sendSlider("s50", F("Wheel slow ratio"), optSlowRatio, "", 0.1, 2.0, 1.0);
  sendSlider("s51", F("Backward time"), optBackwardTime, "ms", 200, 5000, 200);
  sendSlider("s52", F("Minimum roll time"), optRollTimeMin, "ms", 100, 4000, 100);
  sendSlider("s53", F("Maximum roll time"), optRollTimeMax, "ms", 100, 9000, 200);
  sendSlider("s64", F("Square size"), optSquareSize, "kCm", 1, 2000, 10);
  sendSlider("s65", F("Square offset"), optSquareOffset, "kCm", 1, 200, 1);
  sendSlider("s66", F("Square increment"), optSquareInc, "kCm", 1, 100, 1);
  sendSlider("s67", F("Square cycles"), optSquareCycles, "n", 1, 2000, 1);
  sendMenuFinish();
} // void sendOtherSettingsMenu(boolean update)

void sendNavSettingsMenu(boolean update) {
  debug(L_INFO, (char *) F("pfod.cpp: sending navigation settings menu\n"));
  sendMenuStart(update, F("Navigation settings"));
  sendButton("m00", F("Main menu"));
  sendSlider("s62", F("GPS receiver offset"), oGps.distCenter, "cm", 0, 100, 1);
  sendSlider("s63", F("GPS receiver angle"), oGps.angleCenter, "deg", -179, 180, 1);
  sendSlider("s55", F("Magnetic inclination"), oImu.inclMag, "deg", 0.1, 20, -20);
  sendSlider("s56", F("Stop distance (non-precision)"), oGps.stopDistance, "m", 0.1, 5, 0.1);
  sendSlider("s57", F("Stop distance (precision)"), oGps.stopDistancePrecision, "m", 0.1, 5, 0.1);
  sendMenuFinish();
} // void sendNavSettingsMenu(boolean update)

void sendMainMenu(boolean update) {
  debug(L_INFO, (char *) F("pfod.cpp: sending main menu\n"));
  sendMenuStart(update, F("Main menu"));
  // Send name of current command
  sendLabel("l00", String(cmdLabels[cmdRun]));
  
  sendButton("c00", F("Cmd STOP"));
  sendButton("c01", F("Cmd MOVE"));
  sendButton("c02", F("Cmd HOME"));
  sendButton("c03", F("Cmd ZIGZAG"));
  sendButton("c08", F("Cmd SPIRAL"));
  sendButton("c04", F("Cmd NORTH"));
  sendButton("c05", F("Cmd SOUTH"));
  sendButton("c06", F("Cmd EAST"));
  sendButton("c07", F("Cmd WEST"));
  sendButton("c21", F("Cmd MODE POWERSAVE"));
  sendButton("c22", F("Cmd MODE NORMAL"));
  sendButton("c23", F("Cmd SHUTDOWN"));
  sendButton("m01", F("Settings menu"));
  sendButton("m02", F("Info menu"));
  sendButton("m06", F("Test point moveme"));
  sendButton("r01", F("Compass calib restart"));
  sendButton("r02", F("Compass calib stop"));
  sendSwitch("b01", F("Ignore wire"), (cmdFlags & FL_IGNORE_WIRE) > 0);
  sendSwitch("b02", F("Ignore virtual"), (cmdFlags & FL_IGNORE_VIRTUAL) > 0);
  sendSwitch("b03", F("Ignore sonar"), (cmdFlags & FL_IGNORE_SONAR) > 0);
  sendSwitch("b04", F("Ignore bumper"), (cmdFlags & FL_IGNORE_BUMPER) > 0);
  sendSwitch("b05", F("Ignore drop"), (cmdFlags & FL_IGNORE_DROP) > 0);
  sendSwitch("b06", F("Ignore IMU"), (cmdFlags & FL_IGNORE_IMU) > 0);
  sendSwitch("b07", F("Save power"), (cmdFlags & FL_SAVE_POWER) > 0);
  sendSwitch("b08", F("Stop inside"), (cmdFlags & FL_STOP_INSIDE) > 0);
  sendSwitch("b09", F("Stop outside"), (cmdFlags & FL_STOP_OUTSIDE) > 0);
  sendSwitch("b10", F("Stop after"), (cmdFlags & FL_STOP_AFTER) > 0);
  sendSwitch("b11", F("High precision"), (cmdFlags & FL_HIGH_PRECISION) > 0);
  sendMenuFinish();
} // void sendMainMenu(boolean update)

void sendPointsMenu(boolean update) {
  debug(L_INFO, (char *) F("pfod.cpp: sending info menu\n"));
  sendMenuStart(update, F("Info"));
  sendButton("m00", F("Main menu"));
  sendButton("p01", F("Set point 1"));
  sendButton("p02", F("Set point 2"));
  sendButton("p03", F("Set point 3"));
  sendButton("p04", F("Set point 4"));
  sendButton("p05", F("Move to point 1"));
  sendButton("p06", F("Move to point 2"));
  sendButton("p07", F("Move to point 3"));
  sendButton("p08", F("Move to point 4"));
  sendMenuFinish();
} // void sendPointsMenu(boolean update)

void sendInfoMenu(boolean update) {
  debug(L_INFO, (char *) F("pfod.cpp: sending info menu\n"));
  sendMenuStart(update, F("Info"));
  sendButton("m00", F("Main menu"));
  sendSwitch("s90", F("Debug output "), debugLevel == L_DEBUG);
  sendFloat("f00", F("Battery "), oPower.readSensor(P_BATTERY));
  sendFloat("f01", F("Current "), oCrrPower.readCurrent(P_BATTERY));
  sendFloat("f02", F("Sonar left "), oSonars.readSonar(0));
  sendFloat("f03", F("Sonar right "), oSonars.readSonar(2));
  sendFloat("f04", F("Perim left "), oPerim.magnitudeSmooth(0));
  sendFloat("f05", F("Perim right "), oPerim.magnitudeSmooth(1));
  sendFloat("f06", F("Perim inside "), oPerim.inside(-1));
  sendFloat("f07", F("Pitch "), oImu.readCurPitchDegree(0));
  sendFloat("f08", F("Roll "), oImu.readCurRollDegree(0));
  sendFloat("f09", F("Yaw "), oImu.readCurDegree(0));
  sendLong("f10", F("Lat "), oGps.latitude);
  sendLong("f11", F("Lon "), oGps.longitude);
  sendLong("f12", F("numSats "), oGps.numSats);
#ifdef USE_DW1000
  sendLong("f13", F("coordX "), oTag.coordX);
  sendLong("f14", F("coordY "), oTag.coordY);
#endif
  sendMenuFinish();
} // void sendInfoMenu(boolean update)

void processSwitchBit(char *rest, uint16_t bitMask, uint16_t *bits) {
  char *p = strchr(rest, '`');

  if (p) {
    if (*(p + 1) == '0')
      *bits &= ~bitMask;
    else
      *bits |= bitMask;
  }
} // void processSwitchBit(char *rest, uint16_t *bits)

boolean processSwitch(char *rest) {
  char *p = strchr(rest, '`');

  if (p) {
    if (*(p + 1) == '0')
      return false;
    else
      return true;
  }
  return false;
} // boolean processSwitch(char *rest)

float processSlider(char *rest) {
  char *p = strchr(rest, '`');

  if (p)
    return atof(p + 1);
  else
    return 0;
} // float processSlider(char *rest)

// Process external GPS data
void processGPSData(char *rest) {
  int32_t lat, lon;
  uint16_t numSats, year, month, day, hour, minute, second;

  sscanf(rest+1, "%ld,%ld,%hu,%hu,%hu,%hu,%hu,%hu,%hu",
         &lat, &lon, &numSats, &year, &month, &day,
         &hour, &minute, &second);
  oGps.setCoords(lat, lon, numSats, year, month, day, hour, minute, second);
  debug(L_INFO, (char *) F("ext.GPS recv: %ld, %ld (%hu), %04hu-%02hu-%02hu %02hu:%02hu:%02hu\n"),
                 lat, lon, numSats, year, month, day, hour, minute, second);
} // void processGPSData(char *rest)

void processSetting(uint16_t cmdNum, char *rest) {
  switch (cmdNum) {
    case 1:
      oMotors.accel = processSlider(rest);
      break;
    case 2:
      oMotors.accelStop = processSlider(rest);
      break;
    case 3:
      oMotors.maxPWM = processSlider(rest);
      break;
    case 4:
      oMotors.minPWM = processSlider(rest);
      break;
    case 5:
      oMotors.pidRollP = processSlider(rest) / 10;
      break;
    case 6:
      oMotors.pidRollI = processSlider(rest) / 10;
      break;
    case 7:
      oMotors.pidRollD = processSlider(rest) / 10;
      break;
    case 8:
      oMotors.pidMoveP = processSlider(rest) / 10;
      break;
    case 9:
      oMotors.pidMoveI = processSlider(rest) / 10;
      break;
    case 10:
      oMotors.pidMoveD = processSlider(rest) / 10;
      break;
    case 11:
      oCrrMotors.currentMax = processSlider(rest);
      break;
    case 12:
      oCrrMow.currentMax = processSlider(rest);
      break;
    case 13:
      oMow.accel = processSlider(rest);
      break;
    case 14:
      oMow.accelStop = processSlider(rest);
      break;
    case 15:
      oMow.modulateSpeed = processSlider(rest);
      break;
    case 16:
      oMow.modulatePeriod = processSlider(rest) * 100;
      break;
    case 17:
      oMow.heightMowMin = processSlider(rest);
      break;
    case 18:
      oMow.maxPWM = processSlider(rest);
      break;
    case 19:
      oPerim.minQuality = processSlider(rest) / 10;
      break;
    case 20:
      oPerim.errorTimeout = processSlider(rest);
      break;
    case 21:
      oSonars.triggerDist = processSlider(rest);
      break;
    case 22:
      oPerim.invertMag = processSwitch(rest);
      break;
    case 50:
      optSlowRatio = processSlider(rest) / 10;
      break;
    case 51:
      optBackwardTime = processSlider(rest) * 200;
      break;
    case 52:
      optRollTimeMin = processSlider(rest) * 100;
      break;
    case 53:
      optRollTimeMax = processSlider(rest) * 100;
      break;
    case 54:
      optMowHeight = processSlider(rest);
      break;
    case 60:
      optSquareSize = processSlider(rest) * 100;
      break;
    case 61:
      oPower.minSolarVoltage = (float) processSlider(rest) * 0.1;
      break;
    case 55:
      oImu.inclMag = (float) processSlider(rest) * 0.1;
      break;
    case 56:
      oGps.stopDistance = (float) processSlider(rest) * 0.1;
      break;
    case 57:
      oGps.stopDistancePrecision = (float) processSlider(rest) * 0.1;
      break;
    case 62:
      oGps.distCenter = processSlider(rest);
      break;
    case 63:
      oGps.angleCenter = processSlider(rest);
      break;
    case 64:
      optSquareSize = processSlider(rest);
      break;
    case 65:
      optSquareOffset = processSlider(rest);
      break;
    case 66:
      optSquareInc = processSlider(rest);
      break;
    case 67:
      optSquareCycles = processSlider(rest);
      break;
    case 90:
      debugLevel = processSwitch(rest) ? L_NOTICE : L_WARNING;
      break;
    case 91:
      debugLevel = processSwitch(rest) ? L_DEBUG : L_WARNING;
      break;
    default:
      break;
  }
} // void processSetting(uint16_t cmdNum, char *rest)

void processMenu(uint16_t cmdNum, char *rest) {
  currentMenu = cmdNum;
  lastPfodUpdate = 0;
} // void processMenu(uint16_t cmdNum, char *rest)

void processCmdBits(uint16_t cmdNum, char *rest) {
  switch (cmdNum) {
    case 1:
      processSwitchBit(rest, FL_IGNORE_WIRE, &cmdFlags);
      break;
    case 2:
      processSwitchBit(rest, FL_IGNORE_VIRTUAL, &cmdFlags);
      break;
    case 3:
      processSwitchBit(rest, FL_IGNORE_SONAR, &cmdFlags);
      break;
    case 4:
      processSwitchBit(rest, FL_IGNORE_BUMPER, &cmdFlags);
      break;
    case 5:
      processSwitchBit(rest, FL_IGNORE_DROP, &cmdFlags);
      break;
    case 6:
      processSwitchBit(rest, FL_IGNORE_IMU, &cmdFlags);
      break;
    case 7:
      processSwitchBit(rest, FL_SAVE_POWER, &cmdFlags);
      break;
    case 8:
      processSwitchBit(rest, FL_STOP_INSIDE, &cmdFlags);
      break;
    case 9:
      processSwitchBit(rest, FL_STOP_OUTSIDE, &cmdFlags);
      break;
    case 10:
      processSwitchBit(rest, FL_STOP_AFTER, &cmdFlags);
      break;
    case 11:
      processSwitchBit(rest, FL_HIGH_PRECISION, &cmdFlags);
      break;
    default:
      break;
  }
} // void processCmdBits(uint16_t cmdNum, char *rest)

void processCommand(uint16_t cmdNum, char *rest) {
  switch (cmdNum) {
    case 0:
      startCommand(CMD_STOP);
      break;
    case 1:
      startCommand(CMD_MOVE);
      break;
    case 2:
      startCommand(CMD_FIND_PERIMETER);
      break;
    case 3:
      currCmdAngle = INVALID_ANGLE;
      currCmdLat = INVALID_COORD;
      currCmdLon = INVALID_COORD;
      startCommand(CMD_ZIGZAG);
      break;
    case 4:
      startCommand(CMD_NORTH);
      break;
    case 5:
      startCommand(CMD_SOUTH);
      break;
    case 6:
      startCommand(CMD_EAST);
      break;
    case 7:
      startCommand(CMD_WEST);
      break;
    case 8:
      startCommand(CMD_SPIRAL);
      break;
    case 21:
      cmdTime = 180;
      startCommand(CMD_MODE_POWERSAVE);
      break;
    case 22:
      startCommand(CMD_MODE_NORMAL);
      break;
    case 23:
      cmdTime = 0;
      startCommand(CMD_SHUTDOWN);
      break;
    default:
      break;
  }
}// void processCommand(uint16_t cmdNum, char *rest)

void processPoint(uint16_t cmdNum, char *rest) {
  switch (cmdNum) {
    case 1:
    case 2:
    case 3:
    case 4:
      tLat[cmdNum - 1] = oGps.latitude;
      tLon[cmdNum - 1] = oGps.longitude;
      break;
    case 5:
    case 6:
    case 7:
    case 8:
      pointLat = tLat[cmdNum - 5];
      pointLon = tLon[cmdNum - 5];
      startCommand(CMD_POINT);
      break;
    default:
      break;
  }
}

void processRoutine(uint16_t cmdNum, char *rest) {
  switch (cmdNum) {
    case 1:
      resetCompassCalib();
      break;
    case 2:
      stopCompassCalib();
      break;
    case 10:
      loadSaveVars(true);
      break;
    case 11:
      loadSaveVars(false);
      break;
    default:
      break;
  }
} // void processRoutine(uint16_t cmdNum, char *rest)

// Special stuff (like external GPS data)
void processSpecial(uint16_t cmdNum, char *rest) {
  switch (cmdNum) {
    case 0:
      processGPSData(rest);
      break;
    default:
      break;
  }
} // void processSpecial(uint16_t cmdNum, char *rest)

// Process command in the buffer
boolean processPfodCmd(char *cmd) {
  char num[3];
  uint16_t cmdNum;

  debug(L_INFO, (char *) F("pfod command: %s\n"), cmd);
  if (cmd[1] == '.') {
    lastPfodUpdate = 0;
    return true;
  }

  // Convert two-chars number to int
  num[2] = '\0';
  num[0] = cmd[2];
  num[1] = cmd[3];
  cmdNum = atoi(num);
  switch (cmd[1]) {
    case 's':
      processSetting(cmdNum, cmd + 4);
      return true;
    case 'b':
      processCmdBits(cmdNum, cmd + 4);
      return true;
    case 'm':
      processMenu(cmdNum, cmd + 4);
      return true;
    case 'c':
      processCommand(cmdNum, cmd + 4);
      return true;
    case 'r':
      processRoutine(cmdNum, cmd + 4);
      return true;
    case 'p':
      processPoint(cmdNum, cmd + 4);
      break;
    case 'g':
      processSpecial(cmdNum, cmd + 4);
      return false;
    default:
      return false;
  }
} // void processPfodCmd(char *cmd)

void sendCurrentMenu(boolean update) {
  switch (currentMenu) {
    case 0:
      sendMainMenu(update);
      break;
    case 1:
      sendSettingsMenu(update);
      break;
    case 2:
      sendInfoMenu(update);
      break;
    case 3:
      sendMotorsSettingsMenu(update);
      break;
    case 4:
      sendMowingSettingsMenu(update);
      break;
    case 5:
      sendOtherSettingsMenu(update);
      break;
    case 6:
      sendPointsMenu(update);
      break;
    case 7:
      sendNavSettingsMenu(update);
      break;
  }
} // void sendCurrentMenu(boolean update)

uint32_t lastRecv = 0;

// Check console input
void subCheckConsole() {
  uint8_t c;
  uint8_t *p, *next;
  boolean needUpdate = false;

  // Redisplay menu at every second
  if ((millis() - lastPfodUpdate) > 3000) {
    lastPfodUpdate = millis();
    sendCurrentMenu(lastPfodUpdate != 0);
  }

  while (oSerial.available(PFODAPP_PORT)) {
    c = oSerial.read(PFODAPP_PORT);
    // Check if we did receive our modbus ID, reset the buffer
    pfodRcvBuf[pfodLen] = c;
    pfodLen++;
    // Check if we have modbus packet
    if (pfodRcvBuf[0] == MODBUS_ID) {
      // Check if we have full packet in the buffer
      if (checkModbusPacket((uint8_t *) pfodRcvBuf, pfodLen)) {
        // Ignore if we have something else in input buffer (stalled packets)
        if (!oSerial.available(PFODAPP_PORT))
          // Process packet, clear buffer and delay pfod menu update
          parseModbus((uint8_t *) pfodRcvBuf, pfodLen);
        pfodLen = 0;
        lastPfodUpdate = millis();
      } else {
        // Check if we have stalled data in the buffer
        if ((millis() - lastRecv) > 50) {
          // Clear buffer and place new character at the start
          pfodRcvBuf[0] = c;
          pfodLen = 1;
        }
      }
      lastRecv = millis();
    } else {
      // Check if we have full pfod command
      if ((pfodLen == (sizeof(pfodRcvBuf) - 1)) || (c == '}')) {
        pfodRcvBuf[pfodLen] = '\0';
        next = pfodRcvBuf;
        while (p = (uint8_t *) memchr((void *) next, '{', pfodLen + pfodRcvBuf - next)) {
          if (processPfodCmd((char *) p))
            needUpdate = true;
          next = p + 1;
        }
        pfodLen = 0;
        if (needUpdate)
          lastPfodUpdate = millis() - 4000;
      } else {
        // Check if we have received our modbus ID - discard, switch to modbus
        if (c == MODBUS_ID) {
          pfodRcvBuf[0] = c;
          pfodLen = 1;
        }
      }
    }
    // Strip leading \r or \n
    if ((pfodLen == 1) && ((pfodRcvBuf[0] == '\r') || (pfodRcvBuf[0] == '\n')))
      pfodLen = 0;
  }
} // void subCheckConsole()

