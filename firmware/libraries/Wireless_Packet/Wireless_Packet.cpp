#include "Wireless_Packet.h"

Wireless_Packet::Wireless_Packet() {
    // any initialization code goes here (if needed)
}

void Wireless_Packet::sendPacket(const Packet &packet) {
    rfWrite((uint8_t *)&packet, sizeof(Packet));
}

bool Wireless_Packet::receivePacket(Packet &packet, bool isRemote) {
    bool verify = false;
    if (rfAvailable() >= sizeof(Packet)) {
        rfRead((uint8_t *)&packet, sizeof(Packet));
        uint8_t checksum = calculateChecksum(packet);
        uint8_t magicNumber = packet.magicNumber;
        if (isRemote)
            verify =  (checksum == packet.checksum) && (magicNumber == MAGIC_NUMBER_QUAD);
        else
            verify =  (checksum == packet.checksum) && (magicNumber == MAGIC_NUMBER_REMOTE);
    }
    if (!verify) {
        rfFlush();
    }
    return verify;
}

uint8_t Wireless_Packet::calculateChecksum(const Packet &packet) {
    uint8_t checksum = 0;
    const byte *bytePtr = (uint8_t *)&packet;
    for (size_t i = 0; i < sizeof(Packet) - 1; i++) {
        checksum ^= bytePtr[i];
    }
    return checksum;
}
