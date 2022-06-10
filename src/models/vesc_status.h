#pragma once

#include <cstdint>

class VescStatus {
public:
    int32_t rpm;
    int32_t pos;
    explicit VescStatus(const uint8_t *ptr);
};