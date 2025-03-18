#include <Arduino.h>
#include <Wire.h>
#include <smartmotor.h>

void setup() {
    Serial.begin(115200);

    // READ CURRENT ADDRESS
    Serial.println("ENTER CURRENT ADDRESS (1-127)...");
    block();
    String addr_str= Serial.readString();
    uint8_t address= (uint8_t)addr_str.toInt();

    // READ NEW ADDRESS
    Serial.println("ENTER NEW ADDRESS (1-127)...");
    block();
    String new_addr_str= Serial.readString();
    uint8_t new_address= (uint8_t)new_addr_str.toInt();

    // READ NEW GEAR RATIO
    Serial.println("ENTER GEAR RATIO...");
    block();
    String gear_ratio_str= Serial.readString();
    uint16_t new_gear_ratio= (uint16_t)gear_ratio_str.toInt();

    // READ NEW DIRECTION (0 OR 1)? 
    Serial.println("ENTER DIRECTION (0: DIRECT, 1: REVERSE)... ");
    block();
    String new_direction_str= Serial.readString();
    uint8_t new_direction= (uint8_t)new_direction_str.toInt();

    // CONFIGURE MOTOR
    Wire.begin(); // INIT DEVICE AS I2C CONTROLLER
    SmartMotor motor(address); // INIT MOTOR W/ DEFAULT ADDRESS
    Serial.println("CONFIGURING MOTOR...");
    motor.set_address(new_address);
    delay(1000);
    motor.set_gear_ratio(new_gear_ratio);
    delay(1000);
    motor.set_direction(new_direction);

    delay(1000);
    Serial.println("DONE...");
}

void block() { while(Serial.available() == 0) { delay(500); } }

void loop() { }
