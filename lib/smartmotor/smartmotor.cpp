#include "smartmotor.h"
#include <header.h>
#include <simc_write.h>
#include <simc_ops.h>
#include <pid_gains.h>
#include <sensor_data.h>

uint8_t SmartMotor::set_address(uint8_t new_addr) {
    Header h{SET_ADDRESS,sizeof(new_addr)}; // SET HEADER
    uint8_t status= simc_write(_addr,&h,&new_addr); // WRITE MSG
    _addr=new_addr;
    return status;
}

uint8_t SmartMotor::set_gear_ratio(uint16_t gear_ratio) {
    Header h{SET_GEAR_RATIO,sizeof(gear_ratio)}; // SET HEADER
    return simc_write(_addr,&h,&gear_ratio); // WRITE MSG
}

uint8_t SmartMotor::_tune_pid(uint8_t op, float k_p, float k_i, float k_d) {
    PIDGains gains{k_p,k_i,k_d}; // SET PID GAINS
    Header h{op,sizeof(gains)}; // SET HEADER
    return simc_write(_addr,&h,&gains); // WRITE MSG
}

uint8_t SmartMotor::tune_pos_pid(float k_p, float k_i, float k_d) {
    return _tune_pid(TUNE_POS_PID,k_p,k_i,k_d);
}

uint8_t SmartMotor::tune_vel_pid(float k_p, float k_i, float k_d) {
    return _tune_pid(TUNE_VEL_PID,k_p,k_i,k_d);
}

uint8_t SmartMotor::set_direction(uint8_t dir) {
    Header h{SET_PID_DIRECTION,sizeof(dir)}; // SET HEADER
    return simc_write(_addr,&h,&dir); // WRITE MSG
}

uint8_t SmartMotor::_set_pid_target(uint8_t op, void* target_ptr, uint8_t target_sz) {
    Header h{op,target_sz}; // SET HEADER
    return simc_write(_addr,&h,target_ptr); // SEND MSG
}

uint8_t SmartMotor::set_position(int32_t pos) {
    return _set_pid_target(SET_TARGET_POS,&pos,sizeof(pos));
}

uint8_t SmartMotor::set_rpm(float vel) {
    return _set_pid_target(SET_TARGET_VEL,&vel,sizeof(vel));
}

uint8_t SmartMotor::reset() {
    Header h{RESET,0}; // SET HEADER
    return simc_write(_addr,&h,nullptr); // SEND MSG
}