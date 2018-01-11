#include "BucketESC.h"

BucketESC::BucketESC() {
    devAddr = ESC_DEFAULT_ADDR;
}

BucketESC::BucketESC(uint8_t address) {
    devAddr = address;
}

bool BucketESC::testConnection() {
    return getAlive() == ESC_ALIVE;
}

void BucketESC::setThrottle(int16_t throttle) {
	I2Cdev::writeWord(devAddr, ESC_ADDR_THROTTLE_HIGH, throttle);
}

uint16_t BucketESC::getPulseCount() {
	I2Cdev::readWord(devAddr, ESC_ADDR_PULSECOUNT_HIGH, &buffer16);
	return buffer16;
}

uint8_t BucketESC::getAlive() {
	I2Cdev::readByte(devAddr, ESC_ADDR_ALIVE, &buffer8);
	return (uint8_t)buffer8;
}
