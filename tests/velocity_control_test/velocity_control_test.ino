#include <Arduino.h>
#include <Wire.h>
#include <smartmotor.h>

// MOTOR PROPERTIES
#define GEAR_RATIO 150           // MOTOR GEAR RATIO
#define ENCODER_TICKS_PER_REV 12 // NO. OF HIGH PULSES PER ROTATION
const int32_t ENCODER_TICKS_PER_SHAFT_REV= ENCODER_TICKS_PER_REV * GEAR_RATIO;
#define DELAY_PERIOD 7000

// INIT SMART MOTORS
SmartMotor motors[] = {0x05,0x06,0x07}; // INIT MOTORS
const int MOTOR_NUM = sizeof(motors)/sizeof(motors[0]);

void setup() {
    Serial.begin(115200);
    Wire.begin(); // INIT DEVICE AS I2C CONTROLLER
}

float target_vel= 40.0; // INIT TARGET VELOCITY
void loop() {
    // FOR EACH MOTOR ADDRESS
    for (int i = 0; i < MOTOR_NUM; i++) {
        Serial.print("Writing to motor: ");
        Serial.println(motors[i].get_address());

        Serial.print("Target position: ");
        Serial.println(target_vel);

        uint8_t status= motors[i].set_rpm(target_vel);

        Serial.print("Status: ");
        Serial.println(status);
 
        // READ MOTOR VARIABLES IF TRANSMISSION IS SUCCESSFUL
        if (status < 1) {
            delay(DELAY_PERIOD);

            // PRINT CURRENT POSITION
            Serial.print("RPM: ");
            Serial.println(motors[i].get_rpm());
        }
        Serial.println();
    }
    
    delay(DELAY_PERIOD);
}