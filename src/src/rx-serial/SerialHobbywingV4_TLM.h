#pragma once

#include "SerialIO.h"
#include "device.h"

#include <cstddef>
#include <cstdint>

#if defined(TARGET_RX) || defined(UNIT_TEST)

class SerialHobbywingV4TlmTestAccess;

// Receive-only bridge for unsolicited Hobbywing V4 ESC telemetry.
// Valid frames are translated into standard CRSF telemetry sensors.
class SerialHobbywingV4_TLM final : public SerialIO
{
public:
    SerialHobbywingV4_TLM(Stream &outputPort, Stream &inputPort);
    ~SerialHobbywingV4_TLM() override = default;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

private:
    friend class SerialHobbywingV4TlmTestAccess;

    static constexpr uint8_t SYNC_BYTE = 0x9B;
    static constexpr uint8_t TRAILING_MARKER = 0xB9;
    static constexpr uint8_t INFO_FRAME_LENGTH = 13;
    static constexpr uint8_t DATA_FRAME_LENGTH = 19;
    static constexpr uint32_t PARTIAL_FRAME_TIMEOUT_MS = 50;
    static constexpr uint32_t MAX_CAPACITY_SAMPLE_GAP_MS = 1000;
    static constexpr uint32_t MAX_CRSF_CAPACITY_MAH = 0xFFFFFF;
    static constexpr uint8_t RPM_SOURCE_ID = 0;          // Motor 1
    static constexpr uint8_t TEMPERATURE_SOURCE_ID = 0;  // ESC FET

    struct CalibrationData
    {
        bool valid = false;                    // both scales decoded and finite
        uint16_t throttleRange = 1000;         // raw throttle span reported by the ESC
        float voltageScale = 0.0f;             // volt per voltage ADC count
        float currentScale = 0.0f;             // ampere per current ADC count
        float currentOffset = 0.0f;            // current ADC zero offset
    };

    struct DecodedData
    {
        uint32_t packetCounter = 0;            // ESC frame counter
        uint16_t throttlePermille = 0;         // 0.1 percent
        uint16_t pwmPermille = 0;              // 0.1 percent
        uint32_t rpm = 0;                      // electrical revolutions per minute
        uint16_t voltageMillivolts = 0;        // millivolt
        uint32_t currentMilliamps = 0;         // milliampere
        int16_t fetTemperatureDeciCelsius = 0; // 0.1 degree Celsius
        int16_t capTemperatureDeciCelsius = 0; // 0.1 degree Celsius
        bool hasBattery = false;               // voltage and current are valid
        bool hasTemperature = false;           // temperatures are valid
    };

    void processBytes(uint8_t *bytes, uint16_t size) override;
    void processByte(uint8_t value);
    void resetParser();
    void resynchronizeParser();

    bool validateInfoFrame() const;
    bool validateDataFrame() const;
    void applyCalibrationFrame();
    bool decodeDataFrame();
    void integrateConsumedCapacity(uint32_t nowMs);
    void scheduleTelemetry();

    void sendCRSFbattery();
    void sendCRSFrpm();
    void sendCRSFtemp();

    static uint16_t readU16BE(const uint8_t *data);
    static uint32_t readU24BE(const uint8_t *data);
    static uint16_t scaleToPermille(uint16_t value, uint16_t range);
    static bool partialFrameTimedOut(uint32_t now, uint32_t lastReceived);
    static bool decodeTemperatureDeciCelsius(uint16_t adc, int16_t &outDeciCelsius);
    static uint16_t saturateU16(uint32_t value);

    uint8_t frame[DATA_FRAME_LENGTH] = {};
    uint8_t framePosition = 0;

    uint32_t lastReceivedByteMs = 0;
    uint32_t validDataFrameCount = 0;
    bool hasCurrentSample = false;
    uint32_t lastCurrentSampleMs = 0;
    uint64_t consumedMilliampMilliseconds = 0;
    uint32_t consumedMah = 0;

    CalibrationData calibration;
    DecodedData decoded;
};

#endif