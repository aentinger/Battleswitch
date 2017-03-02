##########################################################################

cmake_minimum_required(VERSION 2.8)

##########################################################################

project(battleswitch)

##########################################################################

SET(CMAKE_C_COMPILER avr-gcc)
SET(CMAKE_CXX_COMPILER avr-g++)

##########################################################################

set(AVR_MCU_TYPE  atmega328p)
set(AVR_MCU_SPEED 16000000UL)

##########################################################################

SET(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "-Wl,--relax,--gc-sections")

##########################################################################

set(TARGET battleswitch)  

add_executable(
  ${TARGET}
  led_control.c
  main.c  
  rc_input.c  
  switch_control.c
)

set_target_properties(
  ${TARGET}

  PROPERTIES
    COMPILE_FLAGS "-mmcu=${AVR_MCU_TYPE} -DF_CPU=${AVR_MCU_SPEED} -std=c99 -O1 -Wall -pedantic -pedantic-errors -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -fno-exceptions"
    LINK_FLAGS "-mmcu=${AVR_MCU_TYPE}"
)

##########################################################################

