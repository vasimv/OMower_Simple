// OMower demo command functions and defines
// $Id$

#include "OMower_Simple.h"
#include <stdint.h>

// command flags (bits array)
#define FL_IGNORE_WIRE 0x0001
#define FL_IGNORE_VIRTUAL 0x0002
#define FL_IGNORE_SONAR 0x0004
#define FL_IGNORE_BUMPER 0x0008
#define FL_IGNORE_DROP 0x0010
#define FL_IGNORE_IMU 0x0020
#define FL_SAVE_POWER 0x0040
#define FL_STOP_INSIDE 0x0080
#define FL_STOP_OUTSIDE 0x0100
#define FL_STOP_AFTER 0x0200
#define FL_HIGH_PRECISION 0x0400
#define FL_STOP_CHARGE 0x0800
#define FL_STOP_LAWN 0x1000
#define FL_IGNORE_RAIN 0x2000
#define FL_DISABLE_WIRE 0x4000
#define FL_SIMPLE_MOVE 0x8000

#define CMD_STOP 0
#define CMD_MOVE 1
#define CMD_FIND_PERIMETER 2
#define CMD_ZIGZAG 3
#define CMD_NORTH 4
#define CMD_SOUTH 5
#define CMD_EAST 6
#define CMD_WEST 7
#define CMD_SPIRAL 8
#define CMD_POINT 9
#define CMD_ERROR 10
// Modbus interface commands (also CMD_STOP)
#define CMD_EXT_MOVE_ANGLE 11
#define CMD_EXT_MOVE_GPS 12
#define CMD_EXT_MOVE_TAGS 13
#define CMD_EXT_ROLL_ANGLE 14
#define CMD_EXT_REVERSE 15
#define CMD_EXT_ROLL_PERIMETER 16
#define CMD_EXT_MOVE_PERIMETER 17
#define CMD_EXT_CALIB_START 18
#define CMD_EXT_CALIB_FINISH 19
#define CMD_EXT_SAVE_NVMEM 20
// Powersave commands (robot will stop moving and disable some sensors)
#define CMD_MODE_POWERSAVE 21
#define CMD_MODE_NORMAL 22
#define CMD_SHUTDOWN 23

// Stop reasons
// Command is executing
#define FIN_NONE 0
// Time out
#define FIN_TIME 1
// General hardware problem
#define FIN_HARDWARE 2
// Sonar detection
#define FIN_SONAR 3
// Bumper detection
#define FIN_BUMPER 4
// Drop detection
#define FIN_DROP 5
// No lawn to brush
#define FIN_LAWN 6
// Rain detection
#define FIN_RAIN 7
// High pitch/roll angle (IMU)
#define FIN_FALLING 8
// Wire perimeter detection
#define FIN_PERIMETER 9
// Wheel motors problem (overcurrent/driver fault)
#define FIN_MOTORS 10
// Mowing motor problem (overcurrent/driver fault)
#define FIN_MOWER 11
// Charging voltage detection
#define FIN_CHARGE 12
// Destination reached
#define FIN_DESTINATION 13

// Labels for commands
extern const char *cmdLabels[];

// Current command running (see CMD_* defines)
extern uint16_t cmdRun;

// Current command's flags
extern uint16_t cmdFlags;

// Time when command was started
extern uint32_t cmdStartTime;

// For modbus interface commands
// Time of execution (in milliseconds)
extern uint32_t cmdTime;

// Angle for movement
extern int16_t currCmdAngle;

// Coordinates for GPS/RTK GPS movement
extern int32_t currCmdLat;
extern int32_t currCmdLon;

// Coordinates for radiotag movement
extern int32_t currCmdX;
extern int32_t currCmdY;
extern uint16_t currCmdRoom;

// Command end reason
extern uint16_t cmdStopReason;

// Finish sub-command (new command received or serious hardware error)
extern boolean finishSub;

// Configuration variables
// How much we have to slow speed in "save power" mode
extern float optSlowRatio;

// How much we have to move backward when hitting obstacle or perimeter
extern uint32_t optBackwardTime;

// Minimum roll time for random rolls
extern uint32_t optRollTimeMin;

// Maximum roll time for random rolls
extern uint32_t optRollTimeMax;

// Height of mowing motor (changes every 10 seconds when STOP only)
extern uint8_t optMowHeight;

// Square movement variables
// width of the square (step is about 1.1cm)
extern uint16_t optSquareSize;
// Number of latitude steps
extern uint16_t optSquareCycles;
// Latitude step (is about 1.1cm)
extern uint16_t optSquareInc;
// Latitude offset for eastern side
extern int16_t optSquareOffset;

// target coordinates (form CMD_POINT)
extern int32_t pointLat, pointLon;


// Change current command
void startCommand(uint16_t cmd);

// Modbus interface start new command (data has command code)
void startExtCommand(void *addr, uint32_t data, uint8_t size);

// Reset(restart) compass calibration
void resetCompassCalib();

// Stop compass calibration
void stopCompassCalib();

// Subcommand to move, returns execution time in milliseconds
// dirMove == _dir::STOP - use angle or coordinates
// angle == INVALID_ANGLE - use dirMove or coordinates
// latitude == INVALID_COORD - use angle or dirMove
uint32_t subMove(uint16_t flags, uint32_t timeout, _dir dirMove, int16_t angle, int32_t latitude, int32_t longitude);
