#ifndef sensor_data_h
    #define sensor_data_h

    # include <Arduino.h>

    struct SensorData {
        int32_t position;
        float rpm;
        int16_t current;
    } __attribute__((packed));
#endif