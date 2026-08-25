#include "HWT906.h"

HWT906::HWT906(){}

void HWT906::begin(HardwareSerial& serialPort, uint32_t baud) {
    serial = &serialPort;
    serial->begin(baud);
    parser.reset();
}

bool HWT906::update() {
    if (!serial) return false;

    bool newFrameReceived = false;
    while (serial->available() > 0) {
        uint8_t byteIn = serial->read();
        if (parser.parseByte(byteIn)) {
            newFrameReceived = true;
        }
    }
    return newFrameReceived;
}

const IMUData& HWT906::getData() const {
    return parser.data;
}

void HWT906::sendCommand(const uint8_t cmd[5]) {
    if (!serial) return;
    serial->write(cmd, 5);
    serial->flush();
}

void HWT906::unlock() {
    sendCommand(HWT906Cmd::UNLOCK);
    delay(20);
}

void HWT906::save() {
    sendCommand(HWT906Cmd::SAVE);
    delay(20);
}

// ==== 校正命令組合實作 ====

void HWT906::calibrateAcc() {
    unlock();
    delay(200);
    sendCommand(HWT906Cmd::ACC_CALI);
    delay(4000); 
    sendCommand(HWT906Cmd::EXIT_CALI);
    delay(100);
    save();
}

void HWT906::zeroXY() {
    unlock();
    delay(200);
    sendCommand(HWT906Cmd::ANGLE_CALI);
    delay(3000);
    save();
}

void HWT906::zeroYaw() {
    unlock();
    delay(200);
    sendCommand(HWT906Cmd::YAW_CALI);
    delay(3000);
    save();
}

// void HWT906::startMagCalibration() {
//     unlock();
//     sendCommand(HWT906Cmd::CALI_MAG);
//     delay(20);
// }

// void HWT906::stopMagCalibration() {
//     unlock();
//     sendCommand(HWT906Cmd::EXIT_CALI);
//     delay(20);
//     save();
// }

void HWT906::switchTo9Axis(){
    unlock();
    delay(200);
    sendCommand(HWT906Cmd::AXIS9);
    save();
}

void HWT906::switchTo6Axis(){
    unlock();
    delay(200);
    sendCommand(HWT906Cmd::AXIS6);
    save();
}

void HWT906::readAngle() {
    sendCommand(HWT906Cmd::READ_ANGLE);
}