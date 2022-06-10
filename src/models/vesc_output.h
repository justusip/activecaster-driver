#pragma once

#include <cstdint>

class MotorOutput {
public:
    enum MotorOutputMode {
        MODE_CURRENT = 0,
        MODE_HARDBRAKE = 1,
        MODE_RPM = 2,
        MODE_POS = 3,
    } mode;
    uint8_t value[4];
};