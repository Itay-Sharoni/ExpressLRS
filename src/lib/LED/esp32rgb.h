#pragma once
#include <Arduino.h>

// I2S DMA cap: two buffers of 1024 16-bit stereo samples, 48 bytes per LED.
static constexpr int LED_I2S_DMA_BUF_LEN = 1024;
static constexpr int LED_I2S_DMA_BUF_COUNT = 2;
static constexpr int LED_BYTES_PER_LED = 48;
static constexpr int MAX_LEDS = (LED_I2S_DMA_BUF_LEN * LED_I2S_DMA_BUF_COUNT - 4) / LED_BYTES_PER_LED;

class RgbColor
{
public:
    RgbColor(const uint8_t r, const uint8_t g, const uint8_t b) : R(r), G(g), B(b) {}
    explicit RgbColor(const uint8_t brightness) : R(brightness), G(brightness), B(brightness) {}

    uint8_t R;
    uint8_t G;
    uint8_t B;
};

class ESP32LedDriver
{
public:
    ESP32LedDriver(int count, int pin, int port = 0);
    virtual ~ESP32LedDriver();

    void Begin() const;
    void Show() const;
    void ClearTo(RgbColor color, int first, int last);
    virtual void SetPixelColor(int indexPixel, RgbColor color) = 0;

private:
    uint16_t *out_buffer = nullptr;
    size_t out_buffer_size;
    int num_leds;
    int gpio_pin;
    int i2s_port;

    friend class ESP32LedDriverGRB;
    friend class ESP32LedDriverRGB;
};

class ESP32LedDriverGRB final : public ESP32LedDriver
{
public:
    ESP32LedDriverGRB(const int count, const int pin, const int port = 0) : ESP32LedDriver(count, pin, port) {}
    void SetPixelColor(int indexPixel, RgbColor color) override;
};

class ESP32LedDriverRGB final : public ESP32LedDriver
{
public:
    ESP32LedDriverRGB(const int count, const int pin, const int port = 0) : ESP32LedDriver(count, pin, port) {}
    void SetPixelColor(int indexPixel, RgbColor color) override;
};
