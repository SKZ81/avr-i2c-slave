USER_LIB_PATH = ../../../../libs
ARDUINO_LIBS  = avr-i2c-slave

# BOARD_TAG     = nano
# BOARD_SUB     = atmega328
# MONITOR_PORT  = /dev/ttyUSB0
# Needed with older bootloader
# AVRDUDE_ARD_BAUDRATE = 57600
BOARD_TAG     = uno
MONITOR_PORT  = /dev/ttyACM0
# AVRDUDE_SPEED = 115200
# MCU           = atmega328p

# Note : COMPILE FLAGS (defines)
# -DDEBUG=1 activate traces (as for now, enabled by default, use 0 to disable)
CFLAGS        = -g -Os -std=gnu99 -DDEBUG=1
LDFLAGS       = -Wl,-u,vfprintf -lprintf_flt

ARCHITECTURE  = avr
ARDMK_VENDOR  = archlinux-arduino
include /usr/share/arduino/Arduino.mk



