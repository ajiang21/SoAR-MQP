#ifndef simc_read_h
    #define simc_read_h
    
    #include <Arduino.h>
    #include <Wire.h>
    #include <sensor_data.h>

    inline SensorData* simc_read(uint8_t addr) {
        int sensor_data_sz = sizeof(SensorData);
        Wire.requestFrom(addr,sensor_data_sz); // REQUEST DATA FROM ADDRESS

        uint8_t* data= new uint8_t[sensor_data_sz];
        for (int i=0; i < sensor_data_sz; i++) { data[i]= uint8_t(Wire.read()); }
        
        return (SensorData*)data;
    }
#endif