// OMower demo command functions and defines
// $Id$

#include "OMower_Simple.h"
#include <stdint.h>

// command flags (bits array)
#define FL_IGNORE_WIRE 1
#define FL_IGNORE_VIRTUAL 2
#define FL_IGNORE_SONAR 4
#define FL_IGNORE_BUMPER 8
#define FL_IGNORE_DROP 16
#define FL_IGNORE_IMU 32
#define FL_SAVE_POWER 64
#define FL_STOP_INSIDE 128
#define FL_STOP_OUTSIDE 256
#define FL_STOP_AFTER 512
#define FL_HIGH_PRECISION 1024

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

// Labels for commands
extern const char *cmdLabels[];

// Current command running (see CMD_* defines)
extern uint16_t cmdRun;

// Current command's flags
extern uint16_t cmdFlags;

// Time when command was started
extern uint32_t cmdStartTime;

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

// Reset(restart) compass calibration
void resetCompassCalib();

// Stop compass calibration
void stopCompassCalib();

// Subcommand to move, returns execution time in milliseconds
// dirMove == _dir::STOP - use angle or coordinates
// angle == INVALID_ANGLE - use dirMove or coordinates
// latitude == INVALID_COORD - use angle or dirMove
uint32_t subMove(uint16_t flags, uint32_t timeout, _dir dirMove, int16_t angle, int32_t latitude, int32_t longitude);
