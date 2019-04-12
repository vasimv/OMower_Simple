This software contains demo firmware for OMower SDK with all basic stuff initialization, etc. It can do some simple commands:

Move inside wire perimeter

Follow wire perimeter

Move by GPS or RTK GPS (with RTK GPS you can use "zigzag" or "spiral" patterns)

Move by compass


The firmware is controlled by "pfodApp" program. It can be built with ROS support also (sensors data in internal format and /cmd_vel command supported).

For OMower SDK running on OMower v3/v4 platform - you will need Orange PI Zero or wifi/bluetooth adapter.

For OMower SDK running on Arduino Due+IHM12A1 motor controllers you will need Wifi/Bluetooth adapter connected to serial port 1, modify omower-defs.h and uncomment line in pfod.h:

//#define PFODAPP_PORT SERIAL1_PORT 

