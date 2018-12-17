// OMower modbus protocol
// $Id$

#include "OMower_Simple.h"
#include "robot.h"
#include "modbus.h"



// Modbus registers map (all values are 32bit and use two modbus "registers")
// Number of register is array's element number * 2 (and +1)
// typedef struct { void *, uint8_t sizeVal, uint32_t (readFunc)(void *, uint8_t), void (*saveFunc)(void *, uint32_t, uint8_t)} modbusMap_t;

modbusMap_t modbusMap[] = {
// Command control
{&cmdFlags, 2, &readVal, &writeVal}, // U:CURR_COMMAND_FLAGS
{&cmdTime, 4, &readVal, &writeVal}, // U:CURR_COMMAND_MILLIS
{&cmdRun, 2, &readVal, &startExtCommand}, // U:CURR_COMMAND
{&cmdStartTime, 4, &readVal, NULL}, // U:CURR_COMMAND_START
{NULL, 4, &readMcuMillis, NULL}, // U:MCU_MILLIS
{&currCmdAngle, 2, &readVal, &writeVal}, // I:CURR_COMMAND_ANGLE
{&currCmdLat, 4, &readVal, &writeVal}, // I:CURR_COMMAND_LAT
{&currCmdLon, 4, &readVal, &writeVal}, // I:CURR_COMMAND_LON
{&currCmdX, 4, &readVal, &writeVal}, // I:CURR_COMMAND_X
{&currCmdY, 4, &readVal, &writeVal}, // I:CURR_COMMAND_Y
{&currCmdRoom, 2, &readVal, &writeVal}, // U:CURR_COMMAND_ROOM
{&cmdStopReason, 2, &readVal, NULL}, // U:CURR_COMMAND_STOP_REASON
// Data from robot objects
// GPS
{&oGps.speed, 2, &readVal, NULL}, // U:GPS_APPROX_SPEED
{&oGps.degree, 2, &readVal, NULL}, // I:GPS_APPROX_DEGREE
{&oGps.latitudeRaw, 4, &readVal, NULL}, // I:GPS_COORD_RAW_LAT
{&oGps.longitudeRaw, 4, &readVal, NULL}, // I:GPS_COORD_RAW_LON
{&oGps.year, 2, &readVal, &writeVal}, // U:GPS_YEAR
{&oGps.month, 2, &readVal, &writeVal}, // U:GPS_MONTH
{&oGps.day, 2, &readVal, &writeVal}, // U:GPS_DAY
{&oGps.hour, 2, &readVal, &writeVal}, // U:GPS_HOUR
{&oGps.minute, 2, &readVal, &writeVal}, // U:GPS_MINUTE
{&oGps.second, 2, &readVal, &writeVal}, // U:GPS_SECOND
{&oGps.latitude, 4, &readVal, &writeGpsLat}, // I:GPS_COORD_LAT
{&oGps.longitude, 4, &readVal, &writeGpsLon}, // I:GPS_COORD_LON
{&oGps.numSats, 1, &readVal, &writeGpsSats}, // U:GPS_COORD_SATS
{&oGps.latitudeTarg, 4, &readVal, NULL}, // I:GPS_COORD_TARG_LAT
{&oGps.longitudeTarg, 4, &readVal, NULL}, // I:GPS_COORD_TARG_LON
{(void *) &oGps, 2, &readSoftError, NULL}, // I:GPS_ERROR
// TAGS
{&oTag.speed, 2, &readVal, NULL}, // U:TAGS_APPROX_SPEED
{&oTag.degree, 2, &readVal, NULL}, // I:TAGS_APPROX_DEGREE
{&oTag.validCoords, 1, &readVal, NULL}, // U:TAGS_COORD_VALID
{&oTag.coordX, 4, &readVal, NULL}, // I:TAGS_COORD_X
{&oTag.coordY, 4, &readVal, NULL}, // I:TAGS_COORD_Y
{&oTag.coordRoom, 2, &readVal, &writeVal}, // U:TAGS_COORD_ROOM
{(void *) &oTag, 2, &readSoftError, NULL}, // I:TAGS_ERROR
// IMU
{NULL, 2, &readIMUDegree, NULL}, // I:IMU_YAW
{NULL, 2, &readIMUPitch, NULL}, // I:IMU_PITCH
{NULL, 2, &readIMURoll, NULL}, // I:IMU_ROLL
{NULL, 2, &readIMUTemp, NULL}, // I:IMU_TEMPERATURE
{NULL, 2, &readIMUPress, NULL}, // I:IMU_PRESSURE
{(void *) &oImu, 2, &readSoftError, NULL}, // I:IMU_ERROR
// Perimeter
{(void *) 0, 1, &readPerimInside, NULL}, // U:PERIMETER_INSIDE_LEFT_FORW
{(void *) 1, 1, &readPerimInside, NULL}, // U:PERIMETER_INSIDE_RIGHT_FORW
{(void *) 2, 1, &readPerimInside, NULL}, // U:PERIMETER_INSIDE_LEFT_BACK
{(void *) 3, 1, &readPerimInside, NULL}, // U:PERIMETER_INSIDE_RIGHT_BACK
{(void *) 0, 2, &readPerimMag, NULL}, // I:PERIMETER_LEFT_FORW
{(void *) 1, 2, &readPerimMag, NULL}, // I:PERIMETER_RIGHT_FORW
{(void *) 2, 2, &readPerimMag, NULL}, // I:PERIMETER_LEFT_BACK
{(void *) 3, 2, &readPerimMag, NULL}, // I:PERIMETER_RIGHT_BACK
{(void *) &oPerim, 2, &readSoftError, NULL}, // I:PERIMETER_ERROR
// Drop sensors
{(void *) 0, 2, &readDrop, NULL}, // U:DROP_FORW
{(void *) 1, 2, &readDrop, NULL}, // U:DROP_BACK
{(void *) &oDrop, 2, &readSoftError, NULL}, // I:DROP_ERROR
// Bumper sensors
{(void *) 0, 1, &readBumper, NULL}, // U:BUMPER_LEFT_FORW
{(void *) 1, 1, &readBumper, NULL}, // U:BUMPER_RIGHT_FORW
{(void *) 2, 1, &readBumper, NULL}, // U:BUMPER_LEFT_BACK
{(void *) 3, 1, &readBumper, NULL}, // U:BUMPER_RIGHT_BACK
{(void *) &oBumper, 2, &readSoftError, NULL}, // I:BUMPER_ERROR
// Rain sensor
{NULL, 1, &readRain, NULL}, // U:RAIN_STATUS
{(void *) &oRain, 2, &readSoftError, NULL}, // I:RAIN_ERROR
// Sonar sensors
{(void *) 0, 2, &readSonar, NULL}, // U:SONAR_LEFT_FORW
{(void *) 1, 2, &readSonar, NULL}, // U:SONAR_CENTER_FORW
{(void *) 2, 2, &readSonar, NULL}, // U:SONAR_RIGHT_FORW
{(void *) 3, 2, &readSonar, NULL}, // U:SONAR_LEFT_BACK
{(void *) 4, 2, &readSonar, NULL}, // U:SONAR_CENTER_BACK
{(void *) 5, 2, &readSonar, NULL}, // U:SONAR_RIGHT_BACK
{(void *) &oSonars, 2, &readSoftError, NULL}, // I:SONAR_ERROR
// Lawn sensors
{(void *) 0, 2, &readLawn, NULL}, // U:LAWN_FORW
{(void *) 1, 2, &readLawn, NULL}, // U:LAWN_BACK
{(void *) &oLawn, 2, &readSoftError, NULL}, // I:LAWN_ERROR
// RTC
{NULL, 4, &readRtc, &writeRtc}, // U:RTC_UNIXTIME
{(void *) &oRtc, 2, &readSoftError, NULL}, // I:RTC_ERROR
// Motors
{(void *) 0, 2, &readPWM, NULL}, // I:MOTORS_LEFT_PWM
{(void *) 1, 2, &readPWM, NULL}, // I:MOTORS_RIGHT_PWM
{(void *) &oMotors, 2, &readSoftError, NULL}, // I:MOTORS_ERROR
// Mowing motor
{&oMow.maxSpeed, 1, &readVal, &writeVal}, // U:MOWER_MAXSPEED
{&oMow.heightMow, 1, &readVal, NULL}, // U:MOWER_HEIGHT
{(void *) &oMow, 2, &readSoftError, NULL}, // I:MOWER_ERROR
// Button&LEDs
{NULL, 2, &readSwitches, &writeSwitches}, // U:BUTTONSLEDS_STATUS
// Odometry sensors (wheel motors)
{(void *) 0, 4, &readOdometry, &writeOdometry}, // I:ODOMETRY_LEFT_FORW
{(void *) 1, 4, &readOdometry, &writeOdometry}, // I:ODOMETRY_RIGHT_FORW
{(void *) 2, 4, &readOdometry, &writeOdometry}, // I:ODOMETRY_LEFT_BACK
{(void *) 3, 4, &readOdometry, &writeOdometry}, // I:ODOMETRY_RIGHT_BACK
{(void *) &oOdoMotors, 2, &readSoftError, NULL}, // I:ODOMETRY_ERROR
// Current sensors (wheel motors)
{(void *) 0, 4, &readCurrent, NULL}, // F:CURRENT_MOTORS_LEFT_FORW
{(void *) 1, 4, &readCurrent, NULL}, // F:CURRENT_MOTORS_RIGHT_FORW
{(void *) 2, 4, &readCurrent, NULL}, // F:CURRENT_MOTORS_LEFT_BACK
{(void *) 3, 4, &readCurrent, NULL}, // F:CURRENT_MOTORS_RIGHT_BACK
// Current sensors (mowing motor)
{(void *) 4, 4, &readCurrent, NULL}, // F:CURRENT_MOWER
// Current sensors (chassis)
{(void *) 0, 4, &readCurrentPow, NULL}, // F:CURRENT_BATTERY
{(void *) 3, 4, &readCurrentPow, NULL}, // F:CURRENT_CHARGE
// Voltage sensors (chassis)
{(void *) 0, 4, &readVoltagePow, NULL}, // F:VOLTAGE_BATTERY
{(void *) 1, 4, &readVoltagePow, NULL}, // F:VOLTAGE_SOLAR
{(void *) 2, 4, &readVoltagePow, NULL}, // F:VOLTAGE_CHARGE
{(void *) 3, 4, &readVoltagePow, NULL}, // F:VOLTAGE_BOOST
{(void *) 4, 4, &readVoltagePow, NULL}, // F:VOLTAGE_3V3
{(void *) 5, 4, &readVoltagePow, NULL}, // F:VOLTAGE_5V
{(void *) 6, 4, &readVoltagePow, NULL}, // F:VOLTAGE_5VSEC
// PWM servos control (except fan, boost and charger regulators)
{(void *) 0, 2, NULL, &writePwmServo}, // U:PWMSERVO_A
{(void *) 1, 2, NULL, &writePwmServo}, // U:PWMSERVO_B
{(void *) 2, 2, NULL, &writePwmServo}, // U:PWMSERVO_C
{(void *) 3, 2, NULL, &writePwmServo}, // U:PWMSERVO_D
{(void *) 4, 2, NULL, &writePwmServo}, // U:PWMSERVO_E
{(void *) 6, 2, NULL, &writePwmServo}, // U:PWMSERVO_F
{(void *) 7, 2, NULL, &writePwmServo}, // U:PWMSERVO_G
// Settings
{(void *) &oMotors.accel, 1, &readVal, &writeVal}, // U:SMOTORS_ACCEL
{(void *) &oMotors.accelStop, 1, &readVal, &writeVal}, // U:SMOTORS_ACCELSTOP
{(void *) &oMotors.maxPWM, 1, &readVal, &writeVal}, // U:SMOTORS_MAXPWM
{(void *) &oMotors.minPWM, 1, &readVal, &writeVal}, // U:SMOTORS_MINPWM
{(void *) &oMotors.maxSpeed, 1, &readVal, &writeVal}, // U:SMOTORS_MAXSPEED
{(void *) &oMotors.pidRollP, 4, &readVal, &writeVal}, // F:SMOTORS_PIDROLL_P
{(void *) &oMotors.pidRollI, 4, &readVal, &writeVal}, // F:SMOTORS_PIDROLL_I
{(void *) &oMotors.pidRollD, 4, &readVal, &writeVal}, // F:SMOTORS_PIDROLL_D
{(void *) &oMotors.pidMoveP, 4, &readVal, &writeVal}, // F:SMOTORS_PIDMOVE_P
{(void *) &oMotors.pidMoveI, 4, &readVal, &writeVal}, // F:SMOTORS_PIDMOVE_I
{(void *) &oMotors.pidMoveD, 4, &readVal, &writeVal}, // F:SMOTORS_PIDMOVE_D
{(void *) &oCrrMotors.currentMax, 4, &readVal, &writeVal}, // F:SMOTORS_CURRENTMAX
{(void *) &oCrrMow.currentMax, 4, &readVal, &writeVal}, // F:SMOWER_CURRENTMAX
{(void *) &oMow.accel, 1, &readVal, &writeVal}, // U:SMOWER_ACCEL
{(void *) &oMow.accelStop, 1, &readVal, &writeVal}, // U:SMOWER_ACCELSTOP
{(void *) &oMow.modulateSpeed, 1, &readVal, &writeVal}, // U:SMOWER_MODULATESPEED
{(void *) &oMow.modulatePeriod, 2, &readVal, &writeVal}, // U:SMOWER_MODULATEPERIOD
{(void *) &oMow.heightMowMin, 1, &readVal, &writeVal}, // U:SMOWER_HEIGHTMOWMIN
{(void *) &oMow.maxPWM, 1, &readVal, &writeVal}, // U:SMOWER_MAXPWM
{(void *) &oImu.inclMag, 4, &readVal, &writeVal}, // F:SIMU_INCLMAG
{(void *) &oPerim.minQuality, 4, &readVal, &writeVal}, // F:SPERIM_MINQUALITY
{(void *) &oPerim.errorTimeout, 1, &readVal, &writeVal}, // U:SPERIM_ERRORTIMEOUT
{(void *) &oPerim.invertMag, 1, &readVal, &writeVal}, // U:SPERIM_INVERTMAG
{(void *) &oGps.maxTimeout, 2, &readVal, &writeVal}, // U:SGPS_MAXTIMEOUT
{(void *) &oGps.angleCenter, 2, &readVal, &writeVal}, // I:SGPS_ANGLECENTER
{(void *) &oGps.distCenter, 2, &readVal, &writeVal}, // I:SGPS_DISTCENTER
{(void *) &oGps.stopDistance, 4, &readVal, &writeVal}, // F:SGPS_STOPDISTANCE
{(void *) &oGps.stopDistancePrecision, 4, &readVal, &writeVal}, // F:SGPS_STOPDISTANCEPRECISION
{(void *) &oSonars.triggerDist, 2, &readVal, &writeVal}, // U:SSONARS_TRIGGERDIST
{(void *) &oPower.maxBatteryVoltage, 4, &readVal, &writeVal}, // F:SPOWER_MAXBATTERYVOLTAGE
{(void *) &oPower.minBatteryVoltage, 4, &readVal, &writeVal}, // F:SPOWER_MINBATTERYVOLTAGE
{(void *) &oPower.maxBatteryChargeStart, 4, &readVal, &writeVal}, // F:SPOWER_MAXBATTERYCHARGESTART
{(void *) &oPower.maxChargeCurrent, 4, &readVal, &writeVal}, // F:SPOWER_MAXCHARGECURRENT
{(void *) &oTag.maxTimeout, 2, &readVal, &writeVal}, // U:STAGS_MAXTIMEOUT
{(void *) &optSlowRatio, 4, &readVal, &writeVal}, // F:SOPT_SLOWRATIO
{(void *) &optBackwardTime, 4, &readVal, &writeVal}, // U:SOPT_BACKWARDTIME
{(void *) &optRollTimeMin, 4, &readVal, &writeVal}, // U:SOPT_ROLLTIMEMIN
{(void *) &optRollTimeMax, 4, &readVal, &writeVal}, // U:SOPT_ROLLTIMEMAX
{(void *) &optMowHeight,1, &readVal, &writeVal}, // U:SOPT_MOWHEIGHT
{(void *) &optSquareSize, 2, &readVal, &writeVal}, // U:SOPT_SQUARESIZE
{(void *) &optSquareInc, 2, &readVal, &writeVal}, // U:SOPT_SQUAREINC
{(void *) &optSquareOffset, 2, &readVal, &writeVal}, // I:SOPT_SQUAREOFFSET
};

#ifdef USE_ROS
#define MODBUS_WRITE(msg,len) oROS.publishCmd(msg, len)
#else
#define MODBUS_WRITE(msg,len) oSerial.write(MODBUS_PORT, msg, len)
#endif

// Output modbus packet buffer
uint8_t bufModbusOut[256];

// Create modbus output packet with data requested (returns output packet size)
uint8_t createModbusOut(uint8_t address, uint8_t func, uint16_t regStart, uint8_t numReg) {
  uint8_t *p = bufModbusOut;
  uint16_t i, n;
  modbusMap_t val;
  uint16_t crc;
  uint32_t data;

  // Restriction on maximum packet size
  if (numReg > 120)
    numReg = 120;

  debug(L_DEBUG, (char *) F("createModbusOut: %hd %hd\n"), regStart, numReg);
  
  // Create modbus header with answer to function 3
  *p = address;
  p++;
  *p = func;
  p++;
  *p = numReg * 2;
  p++;

  for (i = regStart / 2; i < ((regStart + numReg) / 2); i++) {
    // Check in modbusMap array
    if ((i < (sizeof(modbusMap) / sizeof(modbusMap[0]))) && (modbusMap[i].readFunc != NULL)) {
      val = modbusMap[i];
      data = val.readFunc(val.addrVal, val.sizeVal);
      debug(L_DEBUG, (char *) F("readVal: %08X (%hu, %08X, %hu)\n"), data, i, val.addrVal, val.sizeVal);
      putModbus32(p, data);
      p += 4;
    } else {
      // No read function or bad register number, just fill zero
      *(uint32_t *) p = 0;
      p += 4;
    }
  }

  // Add CRC to the output frame
  crc = modbusCrc(bufModbusOut, p - bufModbusOut);
  putModbus16(p, crc);
  p += 2;
  return p - bufModbusOut;
} // uint8_t createModbusOut(uint16_t regStart, uint8_t numReg)

// Process data from function 16 (write multiple holding registers packet)
uint8_t writeRegs(uint16_t regStart, uint8_t *data, uint8_t size) {
  uint16_t i;
  modbusMap_t val;
  uint32_t tmp = 0;
  uint8_t *p = data;
  uint8_t writes = 0;

  debug(L_DEBUG, (char *) F("writeRegs: %hd %hd\n"), regStart, size);
  for (i = 0; i < size / 4; i++) {
    val = modbusMap[regStart / 2 + i];
    if (val.writeFunc != NULL) {
      // Read data with swapped bytes from packet
      tmp = getModbus32(p);
      // Call update function
      val.writeFunc(val.addrVal, tmp, val.sizeVal);
    }
    p += 4;
    writes++;
  }
  return writes;
} // void writeRegs(uint8_t data, uint8_t size)

// Returns true if we have full modbus packet in buffer (without checking CRC)
boolean checkModbusPacket(uint8_t *buf, uint16_t length) {
  uint8_t bytes;

  // Check minimum size
  if (length < 8)
    return false;
  // Check slave ID
  if (buf[0] != MODBUS_ID)
    return false;
  // Check function code
  if ((buf[1] != 0x03) && (buf[1] != 0x10))
    return false;
  if (buf[1] == 0x03) {
    if (length >= 8)
      return true;
    else
      return false;
  }
  // Length of 0x10 function packet would be more than 11
  if (length < 11)
    return false;
  // Check total packet length (with registers values)
  bytes = buf[6];
  if (length >= (bytes + 9))
    return true;
  return false;
} // boolean checkModbusPacket(uint8_t *buf, uint16_t length)

// Returns true if CRC in modbus packet is correct
boolean checkModbusCrc(uint8_t *packet, uint8_t length) {
  uint16_t crc;

  crc = getModbus16(packet + length - 2);
  return (crc == modbusCrc(packet, length - 2));
} // boolean checkModbusCrc(uint8_t *packet, uint8_t length)

// Parse incoming packet and send a response
void parseModbus(uint8_t *packet, uint8_t length) {
  uint16_t crc;
  uint16_t lengthOut;
  uint16_t regStart;
  uint16_t regNum;

  debug(L_DEBUG, (char *) F("Parsing modbus %hd\n"), length);
  // Check id
  if (packet[0] != MODBUS_ID)
    return;
  // Check CRC
  if (!checkModbusCrc(packet, length))
    return;
  // Check function code
  if ((packet[1] != 0x03) && (packet[1] != 0x10)) {
    // Send failure packet
    bufModbusOut[0] = MODBUS_ID;
    bufModbusOut[1] = 0x80 | packet[1];
    bufModbusOut[2] = 0x01;
    crc = modbusCrc(bufModbusOut, 3);
    putModbus16(bufModbusOut + 3, crc);
    MODBUS_WRITE(bufModbusOut, 5);
    return;
  }
  debug(L_INFO, (char *) F("Modbus packet, function code %hd, length %hd\n"), packet[1], length);
  switch (packet[1]) {
    // Read holding registers
    case 0x03:
      regStart = packet[3] | (packet[2] << 8);
      regNum = packet[5] | (packet[4] << 8);
      lengthOut = createModbusOut(MODBUS_ID, packet[1], regStart, regNum);
      MODBUS_WRITE(bufModbusOut, lengthOut);
      return;
    // Write holding registers
    case 0x10:
      regStart = packet[3] | (packet[2] << 8);
      regNum = packet[5] | (packet[4] << 8);
      regNum = writeRegs(regStart, packet + 7, packet[6]);
      bufModbusOut[0] = MODBUS_ID;
      bufModbusOut[1] = packet[1];
      bufModbusOut[2] = packet[2];
      bufModbusOut[3] = packet[3];
      bufModbusOut[4] = 0;
      bufModbusOut[5] = regNum * 2;
      crc = modbusCrc(bufModbusOut, 6);
      putModbus16(bufModbusOut + 6, crc);
      MODBUS_WRITE(bufModbusOut, 8);
      return;
    default:
      break;
  }
  // Send failure packet
  bufModbusOut[0] = MODBUS_ID;
  bufModbusOut[1] = 0x80 | packet[1];
  crc = modbusCrc(bufModbusOut, 2);
  putModbus16(bufModbusOut + 2, crc);
  MODBUS_WRITE(bufModbusOut, 4);
} // void parseModbus(uint8_t *packet, uint8_t length)

// Read size'th bytes from param address
uint32_t readVal(void *param, uint8_t size) {
  uint32_t tmp = 0;
  
  switch (size) {
    case 4:
      tmp = *(uint32_t *) param;
      break;
    case 2:
      tmp = *(uint16_t *) param;
      break;
    case 1:
      tmp = *(uint8_t *) param;
      break;
    default:
      break;
  }
  return tmp;
} // uint32_t readVal(void *param, uint8_t size)

// Write size'th bytes to param address
void writeVal(void *param, uint32_t data, uint8_t size) {
  switch (size) {
    case 4:
      *(uint32_t *) param = data;
      return;
    case 2:
      *(uint16_t *) param = data;
      return;
    case 1:
      *(uint8_t *) param = data;
      return;
  }
} // void writeVal(void *param, uint32_t data, uint8_t size)

// Returns number of milliseconds from start
uint32_t readMcuMillis(void *param, uint8_t size) {
  return millis();
} // uint32_t readMcuMillis(void *param, uint8_t size)

// Returns soft error status (calls specified softError() function)
uint32_t readSoftError(void *param, uint8_t size) {
  thing *curThing = (thing *) param;
  int32_t tmp;

  tmp = (int32_t) curThing->softError();
  return *(uint32_t *) &tmp;
} // uint32_t readSoftError(void *param, uint8_t size)

uint32_t tmpLat, tmpLon;
uint8_t tmpSats;

// Write GPS coordinates in buffer to write it to oGps object when finished
void writeGpsLat(void *param, uint32_t data, uint8_t size) {
  tmpLat = *(uint32_t *) &data;
} // void writeGpsLat(void *param, uint32_t data, uint8_t size)

void writeGpsLon(void *param, uint32_t data, uint8_t size) {
  tmpLon = *(uint32_t *) &data;
} // void writeGpsLon(void *param, uint32_t data, uint8_t size)

// When this function called - transfer this and coordinates through setCoords
void writeGpsSats(void *param, uint32_t data, uint8_t size) {
  tmpSats = data;

  oGps.setCoords(tmpLat, tmpLon, tmpSats, 0, 0, 0, 0, 0, 0);
} // void writeGpsSats(void *param, uint32_t data, uint8_t size)

// Read IMU stuff
uint32_t readIMUDegree(void *param, uint8_t size) {
  int32_t tmp;

  // Convert signed int to 4 bytes 
  tmp = oImu.readCurDegree(-1);
  return *(uint32_t *) &tmp;
} // uint32_t readIMUDegree(void *param, uint8_t size)

uint32_t readIMUPitch(void *param, uint8_t size) {
  int32_t tmp;

  // Convert signed int to 4 bytes 
  tmp = oImu.readCurPitchDegree(-1);
  return *(uint32_t *) &tmp;
} // uint32_t readIMUPitch(void *param, uint8_t size) 

uint32_t readIMURoll(void *param, uint8_t size) {
  int32_t tmp;

  // Convert signed int to 4 bytes 
  tmp = oImu.readCurRollDegree(-1);
  return *(uint32_t *) &tmp;
} // uint32_t readIMURoll(void *param, uint8_t size)

uint32_t readIMUTemp(void *param, uint8_t size) {
  int32_t tmp;

  // Convert signed int to 4 bytes 
  tmp = oImu.readCurTemperature(-1);
  return *(uint32_t *) &tmp;
} // uint32_t readIMUTemp(void *param, uint8_t size)

uint32_t readIMUPress(void *param, uint8_t size) {
  int32_t tmp;

  // Convert signed int to 4 bytes 
  tmp = oImu.readCurPressure(-1);
  return *(uint32_t *) &tmp;
} // uint32_t readIMUPress(void *param, uint8_t size)

// Read perimeter sensors (inside/outside only)
uint32_t readPerimInside(void *param, uint8_t size) {
  return (uint32_t) oPerim.inside((uint32_t) param);
} // uint32_t readPerimInside(void *param, uint8_t size)

// Read perimeter sensors
uint32_t readPerimMag(void *param, uint8_t size) {
  int32_t tmp;

  tmp = oPerim.magnitude((uint32_t) param);
  return *(uint32_t *) &tmp;
} // uint32_t readPerimMag(void *param, uint8_t size)

// Read drop sensors
uint32_t readDrop(void *param, uint8_t size) {
  return oDrop.readSensor((uint32_t) param);
} // uint32_t readDrop(void *param, uint8_t size)

// Read bumper sensors
uint32_t readBumper(void *param, uint8_t size) {
  return oBumper.readSensor((uint32_t) param);
} // uint32_t readBumper(void *param, uint8_t size)

// Read rain sensor
uint32_t readRain(void *param, uint8_t size) {
  return oRain.readSensor(-1);
} // uint32_t readRain(void *param, uint8_t size)

// Read sonars
uint32_t readSonar(void *param, uint8_t size) {
  return oSonars.readSonar((uint32_t) param);
} // uint32_t readSonar(void *param, uint8_t size)

// Read lawn sensors
uint32_t readLawn(void *param, uint8_t size) {
  return oLawn.readSensor((uint32_t) param);
} // uint32_t readLawn(void *param, uint8_t size)

// Set mowing motor height
void writeMowHeight(void *param, uint32_t data, uint8_t size) {
  oMow.setHeight(data);
} // void writeMowHeight(void *param, uint32_t data, uint8_t size)

// Read buttons/LEDs sate (bitmap)
uint32_t readSwitches(void *param, uint8_t size) {
  return (oSwitches.readSensor(5) ? 0x20 : 0)
        | (oSwitches.readSensor(4) ? 0x10 : 0)
        | (oSwitches.readSensor(3) ? 0x08 : 0)
        | (oSwitches.readSensor(2) ? 0x04 : 0)
        | (oSwitches.readSensor(1) ? 0x02 : 0)
        | (oSwitches.readSensor(0) ? 0x01 : 0);
} // uint32_t readSwitches(void *param, uint8_t size)

// Write buttons/LEDs state (bitmap)
void writeSwitches(void *param, uint32_t data, uint8_t size) {
  // T_EXTLED3
  oSwitches.setLED(5, (data & 0x20 != 0));
  // T_EXTLED2
  oSwitches.setLED(4, (data & 0x10 != 0));
  // T_EXTLED1
  oSwitches.setLED(3, (data & 0x08 != 0));
  // T_BOARDLED
  oSwitches.setLED(2, (data & 0x04 != 0));
  // T_BEEPER
  oSwitches.setLED(0, (data & 0x01 != 0));

  // Solar panel enable
  oPower.setSolarCharge((data & 0x2000 != 0));
  // Main charge
  oPower.setMainCharge((data & 0x4000 != 0));
  // Shutdown
  if (data & 0x8000)
    oPower.emergShutdown();
} // void writeSwitches(void *param, uint32_t data, uint8_t size)

// Read odometry sensor
uint32_t readOdometry(void *param, uint8_t size) {
  int32_t tmp;

  tmp = oOdoMotors.readTicks((uint32_t) param);
  return *(uint32_t *) &tmp;
} // uint32_t readOdometry(void *param, uint8_t size)

// Reset odometry sensor
void writeOdometry(void *param, uint32_t data, uint8_t size) {
  oOdoMotors.resetTicks((uint32_t) param);
} // void writeOdometry(void *param, uint32_t data, uint8_t size)

// Read current sensor (wheel/mowing motors)
uint32_t readCurrent(void *param, uint8_t size) {
  float tmp;

  switch ((uint32_t) param) {
    case 0:
    case 1:
    case 2:
    case 3:
      tmp = oCrrMotors.readCurrent((uint32_t) param);
      break;
    case 4:
      tmp = oCrrMow.readCurrent(-1);
      break;
  }
  return *(uint32_t *) &tmp;
} // uint32_t readCurrent(void *param, uint8_t size)

// Read current sensor (power class)
uint32_t readCurrentPow(void *param, uint8_t size) {
  float tmp;

  tmp = oCrrPower.readCurrent((uint32_t) param);
  return *(uint32_t *) &tmp;
} // uint32_t readCurrentPow(void *param, uint8_t size)

// Read voltage sensor
uint32_t readVoltagePow(void *param, uint8_t size) {
  float tmp;

  tmp = oPower.readSensor((uint32_t) param);
  return *(uint32_t *) &tmp;
} // uint32_t readVoltagePow(void *param, uint8_t size)

// PWM servos control
void writePwmServo(void *param, uint32_t data, uint8_t size) {
  oPwm.servoSet((uint32_t) param, data);
} // void writePwmServo(void *param, uint32_t data, uint8_t size)

// Read RTC (unixtime)
uint32_t readRtc(void *param, uint8_t size) {
  return oRtc.getUnixTime();
} // uint32_t readRtc(void *param, uint8_t size)

// Write RTC (unixtime)
void writeRtc(void *param, uint32_t data, uint8_t size) {
  oRtc.setUnixTime(data);
} // void writeRtc(void *param, uint32_t data, uint8_t size)

// Read motors PWM values
uint32_t readPWM(void *param, uint8_t size) {
  int32_t tmp;

  tmp = oMotors.curPWM((uint32_t) param);
   return *(uint32_t *) &tmp;
} // uint32_t readPWM(void *param, uint8_t size)

