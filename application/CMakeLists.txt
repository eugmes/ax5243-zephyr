cmake_minimum_required(VERSION 3.13.1)

set(BOARD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(BOARD nucleo_l432kc)
set(SHIELD ax5243_nano)

find_package(Zephyr REQUIRED HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../zephyr")

project(ax5243_test)

target_sources(app PRIVATE
  src/main.c
  src/ax5x43.c)
