// OMower simple - simple automatic lawn mowing with OMower SDK
// Inspired by ardumower project - http://www.ardumower.de/
// $Id$

#include "OMower_Simple.h"
#include <U8g2lib.h>
#include <due-adc-scan.h>
#include <max11617-adc-scan.h>
#include "robot.h"
#include "pfod.h"
#include <DueTimer.h>

#ifdef USE_OLED_DISPLAY
// Demo I2C display connected to software I2C on pins 74, 75
// ("SPI" connector as we don't use any SPI device)
// We use software I2C as hardware I2C function don't have any
// transaction system yet and work only for poll50() level
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 74, 75, U8X8_PIN_NONE);
#endif

// OMower SDK objects
chassis oChassis;
motors oMotors;
imu oImu;
perimeter oPerim;
gps oGps;
seeker oSeeker;
#ifdef USE_DW1000
radiotag oTag;
#endif
virtPerim oVPerim;
buttonsLeds oSwitches;
motorMow oMow;
pwmServo oPwm;
sonars oSonars;
rain oRain;
drop oDrop;
bumper oBumper;
lawn oLawn;
currentMow oCrrMow;
currentPow oCrrPower;
currentMotors oCrrMotors;
odometryMotors oOdoMotors;
power oPower;
rtc oRtc;
serial oSerial;
nvmem oSave;

// NVMEM address for load/save settings
uint32_t nvmemAddr;

// Onboard LED state
volatile uint8_t ledState = 1;

uint32_t cntPoll50 = 0;
uint32_t cntPoll20 = 0;
uint32_t cntPoll10 = 0;

// Put all poll*() functions here
// Poll hook for 50Hz
void poll50() {
  cntPoll50++;
  oImu.poll50();
  oPower.poll50();
} // void poll50()

// Poll hook for 20Hz
void poll20() {
  cntPoll20++;
  oPerim.poll20();
  oSonars.poll20();
  oSeeker.poll20();
} // poll20()

// Poll hook for 10Hz
void poll10() {
  cntPoll10++;
  oGps.poll10();
#ifdef USE_DW1000
  oTag.poll10();
#endif
  oMotors.poll10();
  oMow.poll10();
  oOdoMotors.poll10();
  oBumper.poll10();
  oVPerim.poll10();

  // Blink led
  if (ledState) {
    if ((ledState == 1) || ((cyclesPollNum % 50) == 0))
      oSwitches.setLED(T_BOARDLED, !oSwitches.readSensor(T_BOARDLED));
  } else
     oSwitches.setLED(T_BOARDLED, false);
} // void poll10()

// Set default values for all object's configuration variables
// OMower PCB v3,  4-cell LiFePo4 battery (14.8V max)
void setDefaultVars() {
  oMotors.accel = 128;
  oMotors.accelStop = 128; 
  oMotors.maxPWM = 255;
  oMotors.minPWM = 64;
  oMotors.maxSpeed = 255;
  oMotors.pidRollP = 10.0;
  oMotors.pidRollI = 0.1;
  oMotors.pidRollD = 0.5;
  oMotors.pidMoveP = 15.0;
  oMotors.pidMoveI = 0.1;
  oMotors.pidMoveD = 0.1;
  oCrrMotors.currentMax = 8.0;
  oCrrMow.currentMax = 15.0;
  oCrrPower.zeroOffset = 2059;
  oMow.accel = 32;
  oMow.accelStop = 64;
  oMow.modulateSpeed = 0;
  oMow.modulatePeriod = 2000;
  oMow.heightMowMin = 55;
  oMow.heightMow = 28;
  oMow.maxPWM = 1;
  oImu.inclMag = -11.5;
  oPerim.minQuality = 1.1;
  oPerim.errorTimeout = 120;
  oPerim.invertMag = true;
  oVPerim.errorTimeout = 30;
  oGps.maxTimeout = 3000;
  oGps.angleCenter = -158;
  oGps.distCenter = 42;
  oGps.stopDistance = 3.0;
  oGps.stopDistancePrecision = 0.2;
  oSonars.triggerDist = 20;
  oPower.maxBatteryVoltage = 14.6;
  oPower.minBatteryVoltage = 11;
  oPower.maxBatteryChargeStart = 14.2;
  oPower.maxChargeCurrent = 5.0;
  oPower.minSolarVoltage = 15.0;

  optSlowRatio = 1.0;
  optBackwardTime = 2500;
  optRollTimeMin = 2000;
  optRollTimeMax = 4000;
  optMowHeight = 0;
  optSquareSize = 1054;
  optSquareCycles = 104;
  optSquareInc = 15;
  optSquareOffset = 100;
#ifdef USE_DW1000
  oTag.maxTimeout = 400;
#endif
  oSeeker.maxTimeout = 500;
} // void setDefaultVars()

// Load/save variable template function
template<typename T> void loadSave(T &data, boolean save);

template<typename T> void loadSave(T &data, boolean save) {
  T tmp;
  
  if (save)
    oSave.writeMem(data);
  else {
    if (oSave.haveValid)
      oSave.readMem(data);
    else
      oSave.readMem(tmp);
  }
} // void loadSave(T data, boolean save)

// Load or save settings from NVMEM
void loadSaveVars(boolean save) {
  // use stored address in NVMEM to load/save settings from
  oSave.curAddr = nvmemAddr;
  // Load stuff
  debug(L_NOTICE, (char *) F("load/save to address %08x %d %d\n"), oSave.curAddr, save, (save || oSave.haveValid));
  loadSave(oMotors.accel, save);
  loadSave(oMotors.accelStop, save);
  loadSave(oMotors.maxPWM, save);
  loadSave(oMotors.minPWM, save);
  loadSave(oMotors.maxSpeed, save);
  loadSave(oMotors.pidRollP, save);
  loadSave(oMotors.pidRollI, save);
  loadSave(oMotors.pidRollD, save);
  loadSave(oMotors.pidMoveP, save);
  loadSave(oMotors.pidMoveI, save);
  loadSave(oMotors.pidMoveD, save);
  loadSave(oCrrMotors.currentMax, save);
  loadSave(oCrrMow.currentMax, save);
  loadSave(oMow.accel, save);
  loadSave(oMow.accelStop, save);
  loadSave(oMow.modulateSpeed, save);
  loadSave(oMow.modulatePeriod, save);
  loadSave(oMow.heightMowMin, save);
  loadSave(oMow.maxPWM, save);
  loadSave(oImu.inclMag, save);
  loadSave(oPerim.minQuality, save);
  loadSave(oPerim.errorTimeout, save);
  loadSave(oPerim.invertMag, save);
  loadSave(oVPerim.errorTimeout, save);
  loadSave(oGps.maxTimeout, save);
  loadSave(oGps.angleCenter, save);
  loadSave(oGps.distCenter, save);
  loadSave(oGps.stopDistance, save);
  loadSave(oGps.stopDistancePrecision, save);
  loadSave(oSonars.triggerDist, save);
  loadSave(oPower.maxBatteryVoltage, save);
  loadSave(oPower.minBatteryVoltage, save);
  loadSave(oPower.maxBatteryChargeStart, save);
  loadSave(oPower.maxChargeCurrent, save);
  loadSave(oPower.minSolarVoltage, save);  
  loadSave(optSlowRatio, save);
  loadSave(optBackwardTime, save);
  loadSave(optRollTimeMin, save);
  loadSave(optRollTimeMax, save);
  loadSave(optMowHeight, save);
  loadSave(optSquareSize, save);
  loadSave(optSquareCycles, save);
  loadSave(optSquareInc, save);
  loadSave(optSquareOffset, save);
  loadSave(virtRLat, save);
  loadSave(virtRLon, save);
  for (int i = 0; i < (sizeof(tLat) / sizeof(tLat[0])); i++) {
    loadSave(tLat[i], save);
    loadSave(tLon[i], save);
  }
#ifdef USE_DW1000                                                                                             
  loadSave(oTag.maxTimeout, save);
#endif
  loadSave(oSeeker.maxTimeout, save);
  if (save) {
    oImu.saveCalib();
    oVPerim.savePerimeter();
    oSave.commit();
  } else {
    oImu.loadCalib();
    oVPerim.loadPerimeter();
  }
} // void loadSaveVars()

void delayWithROS(int pause) {
  unsigned long endPause = millis() + pause;
  
  while (millis() < endPause) {
    oROS.spinOnce();
    delay(1);
  }
} // void delayWithROS(int pause)


void setup() {
  // Set speed for console UART
  consInit(460800);
  oSerial.printf(CONSOLE_PORT, "OMower by Vasim V. (C) 2016-2018\n");
  
  oROS.advertiseSubscribe();

  // Set debug level for hw init
  debugLevel = L_DEBUG;

  // set default values
  setDefaultVars();

  // Link sensor objects
  oMotors.currentSens = &oCrrMotors;
  oMotors.odometrySens = &oOdoMotors;
  oMow.currentSens = &oCrrMow;
  oPower.currentSens = &oCrrPower;
  oGps.imuSens = &oImu;
  oGps.odoSens = &oOdoMotors;
  oSeeker.imuSens = &oImu;
  oVPerim.oGps = &oGps;
  oVPerim.imuSens = &oImu;
  oImu.bus = 0;

#ifdef USE_OLED_DISPLAY
  // start demo display
  u8g2.begin();
  u8g2.setPowerSave(0);
  u8g2.setFont(u8g2_font_inb16_mn);
#endif

  debug(L_WARNING, (char *) F("Hardware init start\n"));
  oSave.revision = REVISION;
  // Hardware init, chassis must be first
  oChassis.begin();
  oSave.begin();
  oMotors.begin();  
  oSerial.begin();
  oPower.begin();
  oImu.begin();
  oVPerim.begin();
  oPerim.begin();
  oGps.begin();
  oRtc.begin();
  oSwitches.begin();
  oMow.begin();
  oOdoMotors.begin();
  oTag.begin();
  oSeeker.begin();

  // Remember nvmem current address to load/save values
  nvmemAddr = oSave.curAddr;

  // Load variables from NVMEM
  loadSaveVars(false);
  
  debug(L_WARNING, (char *) F("init()'s start\n"));

  // Software inits (these can be called later to reset)
  oMotors.init();
  oSeeker.init();
  oImu.init(&oSave);
  oVPerim.init(&oSave);
  oPerim.init();
  oGps.init();
  oMow.init();
  oSonars.init();
  oOdoMotors.init();
  oBumper.init();
  
#ifdef USE_DW1000
  oTag.imuSens = &oImu;
  oTag.serialPorts = &oSerial;
  oTag.tagPort = SERIAL3_PORT;
  // Pass-through mode (coordinates from radiotag module will be feed to the GPS module)
  oTag.passGps = &oGps;

  oTag.init();
#endif
  debug(L_WARNING, (char *) F("Hardware init complete\n"));
  oSerial.setBaud(1, 115200);
  // Set GPS port (SERIAL-2) speed
  oSerial.setBaud(2, 9600);
  oSerial.setBaud(3, 115200);

  // Start poll routines
  oChassis.setPollHooks(&poll50, &poll20, &poll10);

  #ifdef USE_ROS
  debugLevel = L_NOTICE;
  #else
  debugLevel = L_WARNING;
  #endif

  // Give enough time for settle up
  delayWithROS(oChassis.pauseTime);

  debug(L_WARNING, (char *) F("Init done\n"));

  // Make small beep
  oSwitches.setLED(T_BEEPER, true);
  oSwitches.setLED(T_EXTLED2, true);
  delayWithROS(2000);
  oSwitches.setLED(T_BEEPER, false);
  oSwitches.setLED(T_EXTLED2, false);
  delayWithROS(1000);
  oSwitches.setLED(T_BEEPER, true);
  oSwitches.setLED(T_EXTLED2, true);
  delayWithROS(2000);
  oSwitches.setLED(T_BEEPER, false);
  oSwitches.setLED(T_EXTLED2, false);

  oPower.setMainCharge(true);
  oPower.setSolarCharge(true);
} // void setup()

// Print info on OLED screen
void displayStuff() {
  char tmpS[32];
  
#ifdef USE_OLED_DISPLAY
  // Display info (since it takes long time - only when CMD_STOP
  if (((cmdRun == CMD_STOP) || (cmdRun == CMD_MODE_POWERSAVE)) && ((millis() - cmdStartTime) > 5000)) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_profont22_mr);
    rpl_snprintf(tmpS, sizeof(tmpS), "%.2f %.1f", oPower.readSensor(P_BATTERY), oCrrPower.readCurrent(P_BATTERY));
    u8g2.drawStr(0,15, tmpS);
    rpl_snprintf(tmpS, sizeof(tmpS), "%hucm %hucm", oSonars.readSonar(0), oSonars.readSonar(2));
    u8g2.drawStr(0,36, tmpS);
    rpl_snprintf(tmpS, sizeof(tmpS), "P:%.1f %.1f", oPerim.quality(0), oPerim.quality(1));
    u8g2.drawStr(0,57, tmpS);
    u8g2.sendBuffer();
  }
#endif

  debug(L_NOTICE, (char *) F("int %lu %lu %lu %lu, p50/20/10 %lu %lu %lu\n"),
        millis(), micros(), adcCounter, cyclesPollNum, cntPoll50, cntPoll20, cntPoll10);
  debug(L_DEBUG, (char *) F("adc scan: "));
  for (int i=0; i < _MAX_ADC_CHANNELS_NUM; i++)
    debug(L_DEBUG, "%hu, ", adcArr[i].lastRead);
  debug(L_DEBUG, "\n");
  debug(L_DEBUG, (char *) F("adc max11617 scan: "));
  for (int i=0; i < _ADC_11617_CHANNELS_NUM; i++)
    debug(L_DEBUG, "%hu, ", adc11617Arr[i]);
  debug(L_DEBUG, (char *) F("perim: %hd/%hd (%f) %hd/%hd (%f)\n"),
        oPerim.magnitude(0), oPerim.magnitudeSmooth(0), oPerim.quality(0),
        oPerim.magnitude(1), oPerim.magnitudeSmooth(1), oPerim.quality(1));
  debug(L_NOTICE, (char *) F("virtPerim: %d\n"), (int) oVPerim.inside());
} // void displayStuff()
