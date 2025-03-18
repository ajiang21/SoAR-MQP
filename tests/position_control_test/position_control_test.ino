#include <Arduino.h>
#include <Wire.h>
#include <smartmotor.h>

// MOTOR PROPERTIES
#define GEAR_RATIO 298           // MOTOR GEAR RATIO
#define ENCODER_TICKS_PER_REV 12 // NO. OF HIGH PULSES PER ROTATION
// #define ENCODER_TICKS_PER_REV 12 // NO. OF HIGH PULSES PER ROTATION
const int32_t ENCODER_TICKS_PER_SHAFT_REV= ENCODER_TICKS_PER_REV * GEAR_RATIO;
#define DELAY_PERIOD 5000

// INIT SMART MOTORS
SmartMotor motors[] = {0x04,0x05,0x06}; // INIT MOTOR W/ DEFAULT ADDRESS
const int MOTOR_NUM = sizeof(motors)/sizeof(motors[0]);

void setup() {
    Serial.begin(115200);
    Wire.begin(); // INIT DEVICE AS I2C CONTROLLER
}

// int32_t target_pos= ENCODER_TICKS_PER_SHAFT_REV; // INIT TARGET DISPLACEMENT
int32_t target_pos= 0; // INIT TARGET DISPLACEMENT
void loop() {
    // FOR EACH MOTOR ADDRESS
    for (int i = 0; i < MOTOR_NUM; i++) {
        Serial.print("Writing to motor: ");
        Serial.println(motors[i].get_address());

        Serial.print("Target position: ");
        Serial.println(target_pos);

        uint8_t status= motors[i].set_position(target_pos);

        Serial.print("Status: ");
        Serial.println(status);
 
        // READ MOTOR VARIABLES IF TRANSMISSION IS SUCCESSFUL
        if (status < 1) {
            delay(DELAY_PERIOD);

            // PRINT CURRENT POSITION
            Serial.print("Position: ");
            Serial.println(motors[i].get_position());
        }
        Serial.println();
    }
    target_pos-= ENCODER_TICKS_PER_SHAFT_REV;
    
    delay(DELAY_PERIOD);
} 
