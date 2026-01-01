#pragma once

#include "motion_sdk/usb_serial.hpp"

namespace motion_sdk {

class Servo {
public:
    Servo(USBSerial& serial, int id);

    bool ping();
    bool setPosition(float angle_deg);
    bool getPosition(float& angle_deg);

private:
    USBSerial& serial_;
    int id_;
};

} // namespace motion_sdk
