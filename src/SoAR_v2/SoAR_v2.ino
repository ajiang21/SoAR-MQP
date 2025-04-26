#include <Arduino.h>
#include <Wire.h>
#include <smartmotor.h>

#define HORZ_INPUT_PIN 3 // Connect the PWM signal 3 for xiao
#define VERT_INPUT_PIN 2 // Connect the PWM signal
#define SQUZ_INPUT_PIN 1 // Connect the PWM signal

#define SPINE_LENGTH_MAX 9 // Spine max length
#define SPINE_LENGTH_MIN 7 // Spine min length
#define SPINE_LENGTH 8 // Spine min length

float SPINE_WIDTH = 2.5;                                               // Spine width
float SPINE_HEIGHT = 2.16;                                             // Spine height
float SPINE_R = 1.44;                                                  // Spine Radius
int INCH_TICS = 2140;                                                  // 1 inch to encoder ticks
float MAX_HORZ = (SPINE_LENGTH_MAX - SPINE_LENGTH_MIN) / SPINE_WIDTH;  // Maximum bending angle in R for horizontal bend
float MAX_VERT = (SPINE_LENGTH_MAX - SPINE_LENGTH_MIN) / SPINE_HEIGHT; // Maximum bending angle in R for vertical bend
float MAX_SQUZ = SPINE_LENGTH_MAX - SPINE_LENGTH_MIN;                  // Maximum contraction length for squeezing

#define DELAY_PERIOD 500
#define GEAR_RATIO 150           // MOTOR GEAR RATIO
#define ENCODER_TICKS_PER_REV 12 // NO. OF HIGH PULSES PER ROTATION
const int32_t ENCODER_TICKS_PER_SHAFT_REV = ENCODER_TICKS_PER_REV * GEAR_RATIO;

// INIT SMART MOTORS
SmartMotor motors[] = {0x05, 0x06, 0x04}; // INIT MOTOR
const int MOTOR_NUM = sizeof(motors) / sizeof(motors[0]);

int high_time_lower = 980; // low: 879 high: 2139 for high time
int high_time_upper = 2020;

float last_val = 0; // Stored last PWM high time

// Struct for storing the Motor Values
struct MValues
{
  float M1;
  float M2;
  float M3;
};

void setup()
{
  Serial.begin(115200);
  Wire.begin(); // INIT DEVICE AS I2C CONTROLLER
  pinMode(HORZ_INPUT_PIN, INPUT);
  pinMode(VERT_INPUT_PIN, INPUT);
  pinMode(SQUZ_INPUT_PIN, INPUT);
  motors[0].tune_pos_pid(0.9, 0.01, 0.005);
  motors[1].tune_pos_pid(0.9, 0.01, 0.005);
  motors[2].tune_pos_pid(0.9, 0.01, 0.005);
  motors[0].reset();
  motors[1].reset();
  motors[2].reset();
  // TODO: Initi function
  controlMotor(0, -INCH_TICS);
  controlMotor(1, -INCH_TICS);
  controlMotor(2, -INCH_TICS);
  motors[0].reset();
  motors[1].reset();
  motors[2].reset();
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readPWMValue(int PWM_INPUT_PIN, float map_val_lower, float map_val_higher)
{
  unsigned long high_time = pulseIn(PWM_INPUT_PIN, HIGH); // Measure HIGH duration
  unsigned long low_time = pulseIn(PWM_INPUT_PIN, LOW);   // Measure LOW duration
  unsigned long period = high_time + low_time;            // Total period

  // float frequency = period > 0 ? 1.0e6 / period : 0;      // Convert to Hz
  // float duty_cycle = period > 0 ? (high_time * 100.0) / period : 0;  // Duty cycle %

  // Calculate mapped value using the provided lower and upper bounds
  Serial.printf("High Time: %d \n", high_time);
  float val = 0;
  if (high_time > high_time_lower && high_time < high_time_upper)
  {
    val = fmap(high_time, high_time_lower, high_time_upper, map_val_lower, map_val_higher);
  }
  return val;
}

MValues CheckMin(MValues mval, int minval)
{
  if (abs(mval.M1) < minval)
  {
    mval.M1 = 0;
  }
  if (abs(mval.M2) < minval)
  {
    mval.M2 = 0;
  }
  if (abs(mval.M3) < minval)
  {
    mval.M3 = 0;
  }
  return mval;
}

// bend_val refers to alpha angle
// bend_dir refers to beta angle
MValues BendDirection(float bend_val, float bend_dir)
{
  MValues result;
  //S1 = (SPINE_LENGTH + (bend_val) * SPINE_R * cos(2 * PI / 6 - bend_dir));
  //S2 = (SPINE_LENGTH - (bend_val) * SPINE_R * cos(bend_dir));
  //S3 = (SPINE_LENGTH - (bend_val) * SPINE_R * cos(2 * PI / 3 - bend_dir));
  result.M1 = bend_val * SPINE_R * cos(2 * PI / 6 - bend_dir) * INCH_TICS;
  result.M2 = - bend_val * SPINE_R * cos(bend_dir) * INCH_TICS;
  result.M3 = - bend_val * SPINE_R * cos(2 * PI / 3 - bend_dir) * INCH_TICS;
  result = CheckMin(result, 100); // remove minimum margin
  return result;
}

MValues HorizontalBend(float bend_val)
{
  MValues result;
  float beta = 0; // Default initialization to prevent uninitialized use

  if (bend_val > 0)
  {
    // Bending Right
    beta = PI / 2;
  }
  else if (bend_val < 0)
  {
    // Bending Left
    beta = 3 * PI / 2;
  }
  else
  {
    // TODO: set error flag
    return result;
  }
  result = BendDirection(abs(bend_val), beta);
  return result;
}

MValues Squeeze(float squeeze_val)
{
  int tics = 0;
  if (squeeze_val > 1)
  {
    tics = squeeze_val * INCH_TICS;
  }
  MValues result = {tics, tics, tics};
  result = CheckMin(result, 100);
  return result;
}

void controlMotor(int motorIndex, int32_t targetPos)
{

  Serial.print("Motor: ");
  Serial.print(motors[motorIndex].get_address());

  Serial.print(" Target position: ");
  Serial.println(targetPos);

  uint8_t status = motors[motorIndex].set_position(targetPos);

  Serial.print(" Status: ");
  Serial.print(status);

  // Check if the command was successful
  if (status < 1)
  {
    // Print current position
    Serial.print(" Position: ");
    Serial.println(motors[motorIndex].get_position());
  }
  Serial.println();
}

void anti_stall(int motorIndex, int32_t targetPos, int MIN)
{
  Serial.print("Motor: ");
  Serial.print(motors[motorIndex].get_address());

  Serial.print(" Target position: ");
  Serial.print(targetPos);

  int32_t current_pos = motors[motorIndex].get_position();
  Serial.print(" Position: ");
  Serial.println(current_pos);

  if (abs(targetPos - current_pos) < MIN)
  {
    Serial.println("Set motor stop ");
    uint8_t status = motors[motorIndex].set_position(current_pos);
    Serial.print("Status: ");
    Serial.println(status);
  }
}

int mix_values(float v1, float v2, float v3)
{
  // If all are non-zero, average all three
  if (v1 != 0 && v2 != 0 && v3 != 0)
  {
    return (int)((v1 + v2 + v3) / 3.0);
  }
  // If two are non-zero, average those two
  else if (v1 != 0 && v2 != 0)
  {
    return (int)((v1 + v2) / 2.0);
  }
  else if (v1 != 0 && v3 != 0)
  {
    return (int)((v1 + v3) / 2.0);
  }
  else if (v2 != 0 && v3 != 0)
  {
    return (int)((v2 + v3) / 2.0);
  }
  // If only one is non-zero, return that one
  else
  {
    return (int)(v1 + v2 + v3);
  }
}

void spine_control_mix(MValues mval1, MValues mval2, MValues mval3)
{
  int M1_tics = mix_values(mval1.M1, mval2.M1, mval3.M1);
  int M2_tics = mix_values(mval1.M2, mval2.M2, mval3.M2);
  int M3_tics = mix_values(mval1.M3, mval2.M3, mval3.M3);

  Serial.print(M1_tics);
  Serial.print(" ");
  Serial.print(M2_tics);
  Serial.print(" ");
  Serial.println(M3_tics);

  controlMotor(0, -M1_tics);
  controlMotor(1, -M2_tics);
  controlMotor(2, -M3_tics);
}

void spine_control(MValues mval1)
{
  Serial.print(mval1.M1);
  Serial.print(" ");
  Serial.print(mval1.M2);
  Serial.print(" ");
  Serial.println(mval1.M3);

  controlMotor(0, -mval1.M1);
  //controlMotor(1, -mval1.M2);
  controlMotor(2, -mval1.M3);
}

void loop()
{
 
  float hori_val = readPWMValue(HORZ_INPUT_PIN, -MAX_HORZ, MAX_HORZ);
  Serial.print("hori_val");
  Serial.print(hori_val);
  Serial.print("   last val");
  Serial.println(last_val);

  MValues hori_mval = HorizontalBend(hori_val);

  Serial.print("Hori Val ");
  Serial.print(hori_mval.M1);
  Serial.print(" ");
  Serial.print(hori_mval.M2);
  Serial.print(" ");
  Serial.println(hori_mval.M3);

  if (abs(hori_val - last_val) < 0.02)
  {
    anti_stall(0, -hori_mval.M1, 80);
    anti_stall(1, -hori_mval.M2, 80);
    anti_stall(2, -hori_mval.M3, 80);
    Serial.println("im here");
  }
  else
  {
    spine_control(hori_mval);
    last_val = hori_val;
    Serial.println("turning motors");
  }
}
