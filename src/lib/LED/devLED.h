#pragma once

#include "device.h"

extern device_t RGB_device;
extern device_t LED_device;

#if defined(TARGET_RX) && defined(PLATFORM_ESP32)
bool rgbLedStripAllowed(int pin);
bool rgbLedStripInit(uint8_t pin);
bool rgbLedStripSetUs(uint16_t us);
void rgbLedStripFlush(bool force = false);
#endif
