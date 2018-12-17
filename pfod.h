// OMower simple pfodApp interface
// $Id$

#define PFODAPP_PORT CONSOLE_PORT
//#define PFODAPP_PORT SERIAL1_PORT

#include <stdint.h>

extern int32_t tLat[4];
extern int32_t tLon[4];

// Check and read console input
void subCheckConsole();
