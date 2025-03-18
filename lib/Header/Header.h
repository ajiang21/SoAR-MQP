#ifndef header_h
    #define header_h

    #include <Arduino.h>

    struct Header {
        uint8_t op, payload_size;
    } __attribute__((packed));
#endif