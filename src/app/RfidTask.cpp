#include "app/RfidTask.h"

uint8_t stateIdForRfidUid(const uint8_t* uid, uint8_t uidLength) {
    if (uid == nullptr || uidLength == 0) {
        return STATE_INIT;
    }

    return STATE_CLASSIC_1_OF_N;
}

bool pollRfidForState(Adafruit_PN532* nfc, uint8_t* stateId) {
    if (nfc == nullptr || stateId == nullptr) {
        return false;
    }

    uint8_t uid[7] = {};
    uint8_t uidLength = 0;
    bool success = nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
    if (!success) {
        return false;
    }

    *stateId = stateIdForRfidUid(uid, uidLength);
    return true;
}
