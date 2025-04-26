#include <Arduino.h>
#include <Wire.h>
#include <smartmotor.h>

#define HORZ_INPUT_PIN 4  // Connect the PWM signal, 2 for xiao
#define VERT_INPUT_PIN 3  // Connect the PWM signal
#define SQUZ_INPUT_PIN 1  // Connect the PWM signal

#define SPINE_LENGTH_MAX 9 // Spine max length
#define SPINE_LENGTH_MIN 7 // Spine min length
float SPINE_WIDTH = 2.5; // Spine width
float SPINE_HEIGHT = 2.16; // Spine height
float SPINE_R = 1.44; // Spine Radius
int INCH_TICS = 2140; // 1 inch to encoder ticks
float MAX_HORZ = (SPINE_LENGTH_MAX - SPINE_LENGTH_MIN)/SPINE_WIDTH - 0.05; // Maximum bending angle in R for horizontal bend
float MAX_VERT = (SPINE_LENGTH_MAX - SPINE_LENGTH_MIN)/SPINE_HEIGHT; // Maximum bending angle in R for vertical bend
float MAX_SQUZ = SPINE_LENGTH_MAX - SPINE_LENGTH_MIN; // Maximum contraction length for squeezing

#define DELAY_PERIOD 500
#define GEAR_RATIO 150           // MOTOR GEAR RATIO
#define ENCODER_TICKS_PER_REV 12 // NO. OF HIGH PULSES PER ROTATION
const int32_t ENCODER_TICKS_PER_SHAFT_REV = ENCODER_TICKS_PER_REV * GEAR_RATIO;

// INIT SMART MOTORS
SmartMotor motors[] = {0x05, 0x06, 0x04}; // INIT MOTOR 
const int MOTOR_NUM = sizeof(motors) / sizeof(motors[0]);

int high_time_lower = 980; // low: 879 high: 2139 for high time
int high_time_upper = 2020;

// Struct for storing the Motor Values
struct MValues {
  float M1;
  float M2;
  float M3;
};

void setup() {
  Serial.begin(115200);
  Wire.begin(); // INIT DEVICE AS I2C CONTROLLER
  pinMode(HORZ_INPUT_PIN, INPUT);
  //pinMode(VERT_INPUT_PIN, INPUT);
  //pinMode(SQUZ_INPUT_PIN, INPUT);
  motors[0].tune_pos_pid(0.9,0.01,0.005);
  motors[1].tune_pos_pid(0.9,0.01,0.005);
  motors[2].tune_pos_pid(0.9,0.01,0.005);
  motors[0].reset();
  motors[1].reset();
  motors[2].reset();
  // TODO: Initi function
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readPWMValue(int PWM_INPUT_PIN, float map_val_lower, float map_val_higher) {
  unsigned long high_time = pulseIn(PWM_INPUT_PIN, HIGH);  // Measure HIGH duration
  unsigned long low_time = pulseIn(PWM_INPUT_PIN, LOW);    // Measure LOW duration
  unsigned long period = high_time + low_time;             // Total period

  //float frequency = period > 0 ? 1.0e6 / period : 0;      // Convert to Hz
  //float duty_cycle = period > 0 ? (high_time * 100.0) / period : 0;  // Duty cycle %
  
  // Calculate mapped value using the provided lower and upper bounds
  Serial.printf("High Time: %d \n", high_time);
  float val = 0;
  if (high_time > high_time_lower && high_time < high_time_upper){
    val = fmap(high_time, high_time_lower, high_time_upper, map_val_lower, map_val_higher);
  }
  return val;
}

MValues CheckMin(MValues mval, int minval){
  if (mval.M1 < minval){mval.M1 = 0;}
  if (mval.M2 < minval){mval.M2 = 0;}
  if (mval.M3 < minval){mval.M3 = 0;}
  return mval;
}

MValues BendDirection(float bend_val, int bend_dir) {
  MValues result;
  float beta = 0; // Default initialization to prevent uninitialized use

  if (bend_dir == 0) {
     // Bending UP
     beta = 0;
  } else if (bend_dir == 1) {
     // Bending RIGHT
     beta = PI / 2;
  } else if (bend_dir == 2) {
     // Bending LEFT
     beta = 3 * PI / 2;
  } else {
     // TODO: set error flag
     return result;
  }

  result.M1 = abs(bend_val) * SPINE_R * cos(2 * PI / 6 - beta) * INCH_TICS;
  result.M2 = abs(bend_val) * SPINE_R * cos(beta) * INCH_TICS;
  result.M3 = abs(bend_val) * SPINE_R * cos(2 * PI / 3 - beta) * INCH_TICS;

  result = CheckMin(result, 120);
  return result;
}




MValues HorizontalBend(float bend_val) {
  MValues result;
  if (bend_val > 0){
    result.M1 = abs(bend_val) * (SPINE_WIDTH) * INCH_TICS;
    result.M2 = abs(bend_val) * (SPINE_WIDTH/2) * INCH_TICS;;
    result.M3 = 0;
  }else{
    result.M1 = 0;
    result.M2 = abs(bend_val) * (SPINE_WIDTH/2) * INCH_TICS;
    result.M3 = abs(bend_val) * (SPINE_WIDTH) * INCH_TICS;
  }
  result = CheckMin(result,100);
  return result;
}

MValues VerticalBend(float bend_val){
  MValues result = {0,0,0};
  if (bend_val > 0){
    result.M1 = 0;
    result.M2 = abs(bend_val) * SPINE_HEIGHT * INCH_TICS;
    result.M3 = 0;
  }else{
    result.M1 = abs(bend_val) * SPINE_HEIGHT * INCH_TICS;
    result.M2 = 0;
    result.M3 = abs(bend_val) * SPINE_HEIGHT * INCH_TICS;
  }
  result = CheckMin(result,120);
  return result;
}

MValues Squeeze(float squeeze_val){
  int tics = 0;
  if (squeeze_val > 1){
    tics = squeeze_val * INCH_TICS;
  }
  MValues result = {tics, tics, tics};
  result = CheckMin(result,100);
  return result;
}

void controlMotor(int motorIndex, int32_t targetPos) {

  Serial.print("Motor: ");
  Serial.print(motors[motorIndex].get_address());

  Serial.print(" Target position: ");
  Serial.print(targetPos);

  uint8_t status = motors[motorIndex].set_position(targetPos);

  //Serial.print("Status: ");
  //Serial.println(status);

  // Check if the command was successful
  if (status < 1) {
    // Print current position
    Serial.print(" Position: ");
    Serial.println(motors[motorIndex].get_position());
  }
  Serial.println();
}

int mix_values(float v1, float v2, float v3) {
  // If all are non-zero, average all three
  if (v1 != 0 && v2 != 0 && v3 != 0) {
    return (int)((v1 + v2 + v3) / 3.0);
  }
  // If two are non-zero, average those two
  else if (v1 != 0 && v2 != 0) {
    return (int)((v1 + v2) / 2.0);
  } else if (v1 != 0 && v3 != 0) {
    return (int)((v1 + v3) / 2.0);
  } else if (v2 != 0 && v3 != 0) {
    return (int)((v2 + v3) / 2.0);
  }
  // If only one is non-zero, return that one
  else {
    return (int)(v1 + v2 + v3);
  }
}

void spine_control(MValues mval1, MValues mval2, MValues mval3){
  int M1_tics = mix_values(mval1.M1, mval2.M1, mval3.M1);
  int M2_tics = mix_values(mval1.M2, mval2.M2, mval3.M2);
  int M3_tics = mix_values(mval1.M3, mval2.M3, mval3.M3);
  
  Serial.print(M1_tics);
  Serial.print(" ");  // Add space between values
  Serial.print(M2_tics);
  Serial.print(" ");
  Serial.println(M3_tics); 
  
  controlMotor(0, -M1_tics);
  controlMotor(1, -M2_tics);
  controlMotor(2, -M3_tics);
}

void loop() {
  float hori_val = readPWMValue(HORZ_INPUT_PIN, -MAX_HORZ, MAX_HORZ);
  //vert_val = readPWMValue(VERT_INPUT_PIN, -MAX_VERT, MAX_VERT);
  //float squz_val = readPWMValue(SQUZ_INPUT_PIN, -MAX_SQUZ, MAX_SQUZ);
  float vert_val = 0;
  float squz_val = 0;

  Serial.print("hori_val");
  Serial.println(hori_val);
  
  MValues hori_mval = HorizontalBend(hori_val);
//  MValues vert_mval = VerticalBend(vert_val);
//  MValues squz_mval = Squeeze(squz_val);

  //MValues hori_mval = {0, 0, 0};
  MValues vert_mval = {0, 0, 0};
  MValues squz_mval = {0, 0, 0};
  
  Serial.print("Hori Val ");
  Serial.print(hori_mval.M1);
  Serial.print(" "); 
  Serial.print(hori_mval.M2);
  Serial.print(" ");
  Serial.println(hori_mval.M3);

//  Serial.print("Vert Val ");
//  Serial.print(vert_mval.M1);
//  Serial.print(" "); 
//  Serial.print(vert_mval.M2);
//  Serial.print(" ");
//  Serial.println(vert_mval.M3);

//  Serial.print("Squz Val ");
//  Serial.print(squz_mval.M1);
//  Serial.print(" "); 
//  Serial.print(squz_mval.M2);
//  Serial.print(" ");
//  Serial.println(squz_mval.M3);
//  
  spine_control(hori_mval,vert_mval,squz_mval);
}
