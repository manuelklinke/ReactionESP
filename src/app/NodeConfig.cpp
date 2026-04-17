#include "app/NodeConfig.h"
#include <string.h>

uint8_t MasterAddress[6] = {0xCC, 0x7B, 0x5C, 0x4F, 0xD2, 0x84};

uint8_t SlaveAddress[STATIC_SLAVE_COUNT][6] = {
    {0xD8, 0xBF, 0xC0, 0x17, 0xA6, 0x9A},
    {0xCC, 0x7B, 0x5C, 0x50, 0x25, 0x1D},
    {0xCC, 0x7B, 0x5C, 0x4F, 0xDE, 0x3B}
};

uint8_t roleFromMasterSwitch(uint8_t switchHigh) {
    return switchHigh ? ROLE_MASTER : ROLE_LIGHT;
}

uint8_t lightIdByMacAddress(const uint8_t* macAddr) {
    if (memcmp(macAddr, MasterAddress, 6) == 0) {
        return 0;
    }

    for (uint8_t i = 0; i < STATIC_SLAVE_COUNT; i++) {
        if (memcmp(macAddr, SlaveAddress[i], 6) == 0) {
            return i + 1;
        }
    }

    return UNKNOWN_LIGHT_ID;
}

bool copyPeerAddressForLight(uint8_t lightId, uint8_t* destination) {
    if (destination == nullptr) {
        return false;
    }

    if (lightId == 0) {
        memcpy(destination, MasterAddress, 6);
        return true;
    }

    if (lightId >= 1 && lightId <= STATIC_SLAVE_COUNT) {
        memcpy(destination, SlaveAddress[lightId - 1], 6);
        return true;
    }

    return false;
}
