#include "vesc_status.h"
#include <cstring>
VescStatus::VescStatus(const uint8_t *ptr) {
        memcpy(&rpm, ptr, sizeof(rpm));
        memcpy(&pos, ptr + 4, sizeof(pos));
}