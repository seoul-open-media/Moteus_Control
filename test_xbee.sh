#!/bin/bash
# Test XBee communication by sending commands

XBEE_PORT="/dev/ttyUSB_Xbee"

echo "Testing XBee communication on $XBEE_PORT"
echo "Sending: STATUS"
echo -e "STATUS\n" > $XBEE_PORT
sleep 1

echo "Sending: M1 0"
echo -e "M1 0\n" > $XBEE_PORT
sleep 1

echo "Sending: STOP"
echo -e "STOP\n" > $XBEE_PORT

echo "Done! Check Serial Monitor on Teensy USB port for output"
