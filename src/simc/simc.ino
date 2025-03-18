#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <PID_v1.h>
#include <header.h>
#include <simc_ops.h>
#include <pid_gains.h>
#include <sensor_data.h>

#define M1 9  // MD9927 MOTOR DRIVER PINS
#define M2 10
#define NSLEEP 5
#define MAX_PWM 255
#define ENCODER_PIN_1 2  // ENCODER PINS
#define ENCODER_PIN_2 3
#define ENCODER_TICKS_PER_REV 12
#define CURRENT_SENSOR_PIN 17  // CURRENT SENSOR PIN
#define SAMPLE_PERIOD 10       // SAMPLE PERIOD (IN MS)

// RPM MULTIPLIER MACRO
#define calc_rpm_mult(gear_ratio) ((1000.0 * 60.0) / double(ENCODER_TICKS_PER_REV * (gear_ratio)))

// DEFAULT CONFIGS
#define DEFAULT_I2C_ADDRESS 0x0A  // DEFAULT I2C ADDRESS
#define DEFAULT_GEAR_RATIO 150    // DEFAULT GEAR RATIO
#define DEFAULT_KP 0.5            // DEFAULT PID GAINS
#define DEFAULT_KI 0.005
#define DEFAULT_KD 0.005

// CONFIG EEPROM ADDRS
#define ADDR_FLAG_IDX 0
#define ADDR_IDX 1
#define DIR_FLAG_IDX 2
#define DIR_IDX 3
#define TPMS_FLAG_IDX 4
#define TPMS_IDX 5

// EEPROM DATA CHECK MACROS
#define valid_i2c_address(addr) ((addr) > 0x00 && (addr) < 0x7F)
#define valid_tpms(tpms) ((tpms) > 0)
#define valid_direction(dir) ((dir) == 0 || (dir) == 1)

// PID CONTROL MODES
#define POSITION_CONTROL 0x00
#define VELOCITY_CONTROL 0x01
#define NONE 0x02

// SIMC OPERATIONS
//TODO: ADD A CHECK TO ENSURE ALL OP FUNCTIONS ARE RECEIVING THE CORRECT NUMBER OF ARGUMENTS
void _set_address(uint8_t*, uint8_t);
void _set_gear_ratio(uint8_t*, uint8_t);
void _tune_pos_pid(uint8_t*, uint8_t);
void _tune_vel_pid(uint8_t*, uint8_t);
void _set_pid_direction(uint8_t*, uint8_t);
void _set_target_pos(uint8_t*, uint8_t);
void _set_target_vel(uint8_t*, uint8_t);
void _reset(uint8_t*, uint8_t);

void (*simc_ops[])(uint8_t*, uint8_t) = {
  _set_address,
  _set_gear_ratio,
  _tune_pos_pid,
  _tune_vel_pid,
  _set_pid_direction,
  _set_target_pos,
  _set_target_vel,
  _reset
};

// OUTPUT PROCESSING FUNCS
void process_pos_output();
void process_vel_output();

void (*process_output[])() = {
  process_pos_output,
  process_vel_output
};

// DRIVER VARIABLES
double cur_enc_count = 0.0;
double prev_enc_count = 0.0;
double rpm = 0.0;
long prev_sample_time = 0;
double pwm = 0.0;
double delta_pwm = 0.0;
double target_pos = 0.0;
double target_vel = 0.0;
bool joined_i2c_bus = false;
uint8_t control_mode = NONE;
SensorData sensor_data;

// SIMC CONFIGS
struct {
  uint8_t address, direction;
  double ticks_per_ms_to_rpm;
} simc_configs;

// PIDs
PID pid[] = {
  { &cur_enc_count, &pwm, &target_pos, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT },
  { &rpm, &delta_pwm, &target_vel, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT },
};

void setup() {
  load_configs();  // LOAD SIMC CONFIGS

  // INIT MOTOR DRIVER
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(NSLEEP, OUTPUT);
  digitalWrite(NSLEEP, HIGH);  //NOTE: NSLEEP MUST BE HIGH TO OPERATE THE MD9927 MOTOR DRIVER

  // INIT ENCODER
  pinMode(ENCODER_PIN_1, INPUT_PULLUP);
  pinMode(ENCODER_PIN_2, INPUT_PULLUP);

  // ATTACH ENCODER INTERRUPTS
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_1), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_2), isr2, CHANGE);

  // INIT CURRENT SENSOR
  pinMode(CURRENT_SENSOR_PIN, INPUT);

  // INIT PIDs
  for (int mode = 0; mode < 2; mode++) {
    pid[mode].SetOutputLimits(-MAX_PWM, MAX_PWM);     // CONSTRAIN POSITION PID OUTPUT
    pid[mode].SetSampleTime(SAMPLE_PERIOD);           // SET POSITION PID SAMPLING TIME
    set_pid_direction(mode, simc_configs.direction);  // SET PID DIRECTION
  }

  join_i2c_bus(simc_configs.address);  // INIT I2C
}

void loop() {
  long now = millis();
  int elapsed_time = (now - prev_sample_time);

  if (elapsed_time >= SAMPLE_PERIOD) {
    update_rpm(elapsed_time);

    if (control_mode != NONE) {
      pid[control_mode].Compute();     // COMPUTE OUTPUT
      process_output[control_mode]();  // PROCESS OUTPUT
      set_driver(pwm);                 // SET DRIVER PWM
    }

    prev_sample_time = now;
  }
}

void load_configs() {
  // SET ADDRESS
  bool addr_loaded = EEPROM[ADDR_FLAG_IDX] == 0x01;
  uint8_t temp_addr= EEPROM.read(ADDR_IDX);
  simc_configs.address = uint8_t((addr_loaded && valid_i2c_address(temp_addr)) ? temp_addr : DEFAULT_I2C_ADDRESS);

  // SET DIRECTION
  bool dir_loaded = EEPROM[DIR_FLAG_IDX] == 0x01;
  uint8_t temp_dir= EEPROM.read(DIR_IDX);
  simc_configs.direction = uint8_t((dir_loaded && valid_direction(temp_dir)) ? temp_dir : DIRECT);

  // SET RPM MULTIPLIER
  bool tpms_loaded = EEPROM[TPMS_FLAG_IDX] == 0x01;
  double temp_tpms= 0.0;
  EEPROM.get(TPMS_IDX, temp_tpms); 
  if (tpms_loaded && valid_tpms(temp_tpms)) simc_configs.ticks_per_ms_to_rpm = temp_tpms;
  else simc_configs.ticks_per_ms_to_rpm = calc_rpm_mult(DEFAULT_GEAR_RATIO);
}

void join_i2c_bus(uint8_t addr) {
  if (joined_i2c_bus) { Wire.end(); }

  Wire.begin(addr);

  // SET I2C CALLBACKS
  Wire.onRequest(on_request);
  Wire.onReceive(on_recv);

  joined_i2c_bus = true;
}

void process_pos_output() {
  pwm = round(pwm);
}

void process_vel_output() {
  pwm = constrain(round(pwm + delta_pwm), -MAX_PWM, MAX_PWM);
}

void isr1() {
  cur_enc_count += ((PIND >> 2 & 0x01) == (PIND >> 3 & 0x01)) ? 1 : -1;
}

void isr2() {
  cur_enc_count += ((PIND >> 2 & 0x01) != (PIND >> 3 & 0x01)) ? 1 : -1;
}

void update_rpm(int elapsed_time) {
  double ticks_per_ms = (cur_enc_count - prev_enc_count) / elapsed_time;
  rpm = ticks_per_ms * simc_configs.ticks_per_ms_to_rpm;  // CALCULATE MOTOR RPM
  prev_enc_count = cur_enc_count;
}

int16_t read_current_sensor() {
  return analogRead(CURRENT_SENSOR_PIN);
}

void set_driver(int16_t driver_pwm) {
  uint8_t pin1, pin2;

  if (driver_pwm > 0) {  // FORWARD
    pin1 = 0;
    pin2 = driver_pwm;
  } else if (driver_pwm < 0) {  // REVERSE
    pin1 = 0 - driver_pwm;
    pin2 = 0;
  } else {  // BRAKE
    pin1 = 0;
    pin2 = 0;
  }

  analogWrite(M1, pin1);
  analogWrite(M2, pin2);
}

void _set_address(uint8_t* p_ptr, uint8_t p_sz) {
  uint8_t new_address = *p_ptr;  // SET ADDRESS

  if (new_address != simc_configs.address && valid_i2c_address(new_address)) {
    join_i2c_bus(new_address);  // REJOIN I2C BUS W/ NEW ADDR
    simc_configs.address = new_address;
    EEPROM.update(ADDR_FLAG_IDX, 0x01);    // WRITE ADDRESS FLAG TO EEPROM
    EEPROM.update(ADDR_IDX, new_address);  // WRITE ADDRESS TO EEPROM
  }
}

void _set_gear_ratio(uint8_t* p_ptr, uint8_t p_sz) {
  uint16_t gear_ratio = *(uint16_t*)p_ptr;  // SET GEAR RATIO
  double new_ticks_per_ms_to_rpm = calc_rpm_mult(gear_ratio);

  if (new_ticks_per_ms_to_rpm != simc_configs.ticks_per_ms_to_rpm && valid_tpms(new_ticks_per_ms_to_rpm)) {
    simc_configs.ticks_per_ms_to_rpm = new_ticks_per_ms_to_rpm;
    EEPROM.update(TPMS_FLAG_IDX, 0x01);             // WRITE ADDRESS FLAG TO EEPROM
    EEPROM.put(TPMS_IDX, new_ticks_per_ms_to_rpm);  // WRITE ADDRESS TO EEPROM
  }
}

void tune_pid(uint8_t gain_type, uint8_t* p_ptr) {
  if (gain_type != NONE) {
    PIDGains* pid_gains = (PIDGains*)p_ptr;
    pid[int(gain_type)].SetTunings(
      pid_gains->k_p,
      pid_gains->k_i,
      pid_gains->k_d);
  }
}

void _tune_pos_pid(uint8_t* p_ptr, uint8_t p_sz) {
  tune_pid(POSITION_CONTROL, p_ptr);
}

void _tune_vel_pid(uint8_t* p_ptr, uint8_t p_sz) {
  tune_pid(VELOCITY_CONTROL, p_ptr);
}

void set_pid_direction(uint8_t mode, uint8_t new_direction) {
  if (mode != control_mode) pid[mode].SetMode(AUTOMATIC);  // ENABLE PID
  pid[mode].SetControllerDirection(int(new_direction));    // SET PID DIRECTION
  if (mode != control_mode) pid[mode].SetMode(MANUAL);     // DISABLE PID
}

void _set_pid_direction(uint8_t* p_ptr, uint8_t p_sz) {
  uint8_t new_direction = *p_ptr;

  if (new_direction != simc_configs.direction && valid_direction(new_direction)) {
    set_driver(0);
    reset_driver_variables();

    // SET PID DIRECTION
    for (int mode = 0; mode < 2; mode++) {
      set_pid_direction(mode, new_direction);
    }

    simc_configs.direction = new_direction;
    EEPROM.update(DIR_FLAG_IDX, 0x01);      // WRITE DIRECTION FLAG TO EEPROM
    EEPROM.update(DIR_IDX, new_direction);  // WRITE DIRECTION TO EEPROM
  }
}

void reset_driver_variables() {
  target_pos = 0.0;
  target_vel = 0.0;
  cur_enc_count = 0.0;
  prev_enc_count = 0.0;
  rpm = 0.0;
  pwm = 0.0;
  delta_pwm = 0.0;
}

void set_control_mode(uint8_t new_control_mode) {
  if (new_control_mode != control_mode) {
    set_driver(0);
    reset_driver_variables();

    if (control_mode != NONE) pid[control_mode].SetMode(MANUAL);             // DISABLE CURRENT PID
    if (new_control_mode != NONE) pid[new_control_mode].SetMode(AUTOMATIC);  // ENABLE NEXT PID

    control_mode = new_control_mode;  // SET CONTROL MODE
  }
}

void _set_target_pos(uint8_t* p_ptr, uint8_t p_sz) {
  set_control_mode(POSITION_CONTROL);       // SET CONTROL MODE
  target_pos = double(*((int32_t*)p_ptr));  // SET PID SETPOINT
}

void _set_target_vel(uint8_t* p_ptr, uint8_t p_sz) {
  set_control_mode(VELOCITY_CONTROL);   // SET CONTROL MODE
  target_vel = double(*(float*)p_ptr);  // SET PID SETPOINT
}

void _reset(uint8_t* p_ptr, uint8_t p_sz) {
  set_control_mode(NONE);
}

void on_request() {
  // LOAD DATA INTO SENSORDATA
  sensor_data.position = int32_t(cur_enc_count);
  sensor_data.rpm = double(rpm);
  sensor_data.current = read_current_sensor();

  Wire.write((uint8_t*)&sensor_data, sizeof(SensorData));
}

void on_recv(int pckt_sz) {
  // LOAD NEW MESSAGE INTO MEMORY
  uint8_t pckt[pckt_sz];
  for (int i = 0; i < pckt_sz; i++) {
    pckt[i] = uint8_t(Wire.read());
  }

  // EXEC OP
  Header* h_ptr = (Header*)&pckt;
  uint8_t* pyld_ptr = &pckt[0] + sizeof(Header);
  simc_ops[int(h_ptr->op)](pyld_ptr, h_ptr->payload_size);
}