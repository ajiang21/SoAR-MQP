#ifndef SmartMotor_h
#define SmartMotor_h

#include <Arduino.h>
#include <simc_read.h>

#define DIRECT (uint8_t) 0x00
#define REVERSE (uint8_t) 0x01

class SmartMotor {
  public:
    SmartMotor(uint8_t addr) : _addr(addr) {}
    uint8_t set_address(uint8_t);
    uint8_t set_gear_ratio(uint16_t);
    uint8_t tune_pos_pid(float, float, float);
    uint8_t tune_vel_pid(float, float, float);
    uint8_t set_direction(uint8_t);
    uint8_t set_position(int32_t);
    uint8_t set_rpm(float);
    uint8_t reset();
    uint8_t get_address() { return _addr; }
    int32_t get_position() { return simc_read(_addr) -> position; }
    float get_rpm() { return simc_read(_addr) -> rpm; }
    int32_t get_current() { return simc_read(_addr) -> current; }

  private:
    uint8_t _addr; // MOTOR ADDRESS

    uint8_t _tune_pid(uint8_t, float, float, float);
    uint8_t _set_pid_target(uint8_t, void *, uint8_t);
};
#endif