#ifndef Wireless_Packet_h
#define Wireless_Packet_h
#define MAGIC_NUMBER_REMOTE 223
#define MAGIC_NUMBER_QUAD 154
#define RADIO_CHANNEL 20
#include "Arduino.h"
#include <radio.h>
#include "quad_remote.h" 

enum State {
  DISARMED,
  ARMED,
  CALIBRATION,
  QUAD_RESET
};

struct Packet {
    uint8_t magicNumber;
    uint8_t roll;
    uint8_t pitch;
    uint8_t yaw;
    uint8_t throttle;
    double Kp_pitch;
    double Ki_pitch; 
    double Kd_pitch;
    double Kp_roll;
    double Ki_roll;
    double Kd_roll;
    double Kp_yaw;
    double Ki_yaw;
    double Kd_yaw;
    uint8_t batteryVoltage;
    int trims[3];
    uint8_t state;
    uint8_t checksum;
};

class Wireless_Packet {
public:
    Wireless_Packet();
    void sendPacket(const Packet &packet);
    bool receivePacket(Packet &packet, bool isRemote);
    uint8_t calculateChecksum(const Packet &packet);
};

#endif
