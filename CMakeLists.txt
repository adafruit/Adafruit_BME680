# Adafruit BME680 Library
# https://github.com/adafruit/Adafruit_BME680
# BSD license

cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS "Adafruit_BME680.cpp" "bme68x.c" "Adafruit_BME680.h" "bme68x_defs.h" "bme68x.h"
                       INCLUDE_DIRS "."
                       REQUIRES arduino Adafruit_BusIO Adafruit_Sensor Adafruit-GFX-Library Adafruit_SSD1306)

project(Adafruit_BME680)
