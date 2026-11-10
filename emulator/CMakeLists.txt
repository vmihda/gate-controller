# Bench emulator of the Nice SNA2 control board for an ESP32-WROOM-32.
# Build: idf.py set-target esp32 build   (its own build directory, separate from firmware/)
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(nice_emulator)
