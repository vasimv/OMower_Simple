// OMower modbus control stuff
// $Id$

#include <stdlib.h>
#include <omower-modbus.h>

// Slave ID of OMower
#define MODBUS_ID 0xA5

// MODBUS serial port
#define MODBUS_PORT CONSOLE_PORT

// Returns true if we have full modbus packet in buffer (without checking CRC)
boolean checkModbusPacket(uint8_t *buf, uint16_t length);

// Parse incoming packet and send a response                                                                  
void parseModbus(uint8_t *packet, uint8_t length);

// Element type of registers map
typedef struct {
                void *addrVal;
                uint8_t sizeVal;
                uint32_t (*readFunc)(void *, uint8_t);
                void (*writeFunc)(void *, uint32_t, uint8_t);
              } modbusMap_t;


// Read size'th bytes from param address
uint32_t readVal(void *param, uint8_t size);

// Write size'th bytes to param address
void writeVal(void *param, uint32_t data, uint8_t size);

// Some specific functions for reading/writing registers

// Returns number of milliseconds from start
uint32_t readMcuMillis(void *param, uint8_t size);

// Returns soft error status (calls specified softError() function)
uint32_t readSoftError(void *param, uint8_t size);

// Write GPS coordinates in buffer to write it to oGps object when finished
void writeGpsLat(void *param, uint32_t data, uint8_t size);
void writeGpsLon(void *param, uint32_t data, uint8_t size);
// When this function called - transfer this and coordinates through setCoords
void writeGpsSats(void *param, uint32_t data, uint8_t size);

// Read IMU stuff
uint32_t readIMUDegree(void *param, uint8_t size);
uint32_t readIMUPitch(void *param, uint8_t size);
uint32_t readIMURoll(void *param, uint8_t size);
uint32_t readIMUTemp(void *param, uint8_t size);
uint32_t readIMUPress(void *param, uint8_t size);

// Read perimeter sensors (inside/outside only)
uint32_t readPerimInside(void *param, uint8_t size);

// Read perimeter sensors (magnitude)
uint32_t readPerimMag(void *param, uint8_t size);

// Read drop sensors
uint32_t readDrop(void *param, uint8_t size);

// Read bumper sensors
uint32_t readBumper(void *param, uint8_t size);

// Read rain sensor
uint32_t readRain(void *param, uint8_t size);

// Read sonars
uint32_t readSonar(void *param, uint8_t size);

// Read lawn sensors
uint32_t readLawn(void *param, uint8_t size);

// Set mowing motor height
void writeMowHeight(void *param, uint32_t data, uint8_t size);

// Read buttons/LEDs sate (bitmap)
uint32_t readSwitches(void *param, uint8_t size);
// Write buttons/LEDs state (bitmap)
void writeSwitches(void *param, uint32_t data, uint8_t size);

// Read odometry sensor
uint32_t readOdometry(void *param, uint8_t size);
// Reset odometry sensor
void writeOdometry(void *param, uint32_t data, uint8_t size);

// Read current sensor (wheel/mowing motors)
uint32_t readCurrent(void *param, uint8_t size);

// Read current sensor (power class)
uint32_t readCurrentPow(void *param, uint8_t size);

// Read voltage sensor
uint32_t readVoltagePow(void *param, uint8_t size);

// PWM servos control
void writePwmServo(void *param, uint32_t data, uint8_t size);

// Read/write RTC
uint32_t readRtc(void *param, uint8_t size);
void writeRtc(void *param, uint32_t data, uint8_t size);

// Read motors PWM values
uint32_t readPWM(void *param, uint8_t size);
