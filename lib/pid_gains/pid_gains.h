#ifndef pid_gains_h
    #define pid_gains_h

    #include <Arduino.h>

    struct PIDGains {
        float k_p, k_i, k_d;
    } __attribute__((packed));
#endif