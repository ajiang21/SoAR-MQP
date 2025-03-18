#ifndef simc_write_h
    #define simc_write_h
    
    #include <Arduino.h>
    #include <Wire.h>
    #include <Header.h>

    inline uint8_t simc_write(uint8_t addr, const Header* h_ptr, const void* p_ptr) {
        Wire.beginTransmission(addr);
        Wire.write((uint8_t*)h_ptr,sizeof(Header));
        Wire.write((uint8_t*)p_ptr,h_ptr -> payload_size);
        return Wire.endTransmission();
    }

    inline uint8_t simc_write(uint8_t addr, const uint8_t* msg_ptr, int msg_sz) {
        Wire.beginTransmission(addr);
        Wire.write(msg_ptr,msg_sz);
        return Wire.endTransmission();
    }
#endif