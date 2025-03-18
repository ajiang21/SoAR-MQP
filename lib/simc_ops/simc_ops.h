#ifndef simc_ops_h
    #define simc_ops_h

    #include <Arduino.h>

    #define SET_ADDRESS (uint8_t) 0x00
    #define SET_GEAR_RATIO (uint8_t) 0x01
    #define TUNE_POS_PID (uint8_t) 0x02
    #define TUNE_VEL_PID (uint8_t) 0x03
    #define SET_PID_DIRECTION (uint8_t) 0x04
    #define SET_TARGET_POS (uint8_t) 0x05
    #define SET_TARGET_VEL (uint8_t) 0x06
    #define RESET (uint8_t) 0x07
#endif