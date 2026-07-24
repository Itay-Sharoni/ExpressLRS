#include "SerialHobbywingV4_TLM.h"

#if defined(TARGET_RX) || defined(UNIT_TEST)

#include "CRSFRouter.h"
#include "common.h"
#include "crsf_protocol.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
constexpr uint8_t SIZE_8BIT = 1;
constexpr uint8_t SIZE_16BIT = 2;
constexpr uint8_t SIZE_24BIT = 3;

// Hobbywing V4 telemetry is an unsolicited, big-endian stream at 19200 8N1. The ESC
// sends a 13-byte INFO frame carrying calibration constants, then 19-byte DATA frames.
// Only fields consumed by this driver are listed below.
//
// DATA frame (19 bytes):
//   0       sync (0x9B)
//   1..3    packet counter
//   4..5    throttle (raw)
//   6..7    PWM output (raw)
//   8..10   eRPM
//   11..12  voltage ADC
//   13..14  current ADC
//   15..16  FET temperature ADC
//   17..18  capacitor temperature ADC
constexpr uint8_t COUNTER_OFFSET = 1;
constexpr uint8_t THROTTLE_OFFSET = 4;
constexpr uint8_t PWM_OFFSET = 6;
constexpr uint8_t RPM_OFFSET = 8;
constexpr uint8_t VOLTAGE_ADC_OFFSET = 11;
constexpr uint8_t CURRENT_ADC_OFFSET = 13;
constexpr uint8_t FET_TEMP_ADC_OFFSET = 15;
constexpr uint8_t CAP_TEMP_ADC_OFFSET = 17;

// INFO frame (13 bytes):
//   0..1    sync (0x9B 0x9B)
//   2..3    throttle range
//   4       RPM steps (always 1)
//   5..6    voltage scale numerator, denominator
//   7..9    current scale numerator, denominator, offset
//   12      trailing marker (0xB9)
constexpr uint8_t INFO_THROTTLE_RANGE_OFFSET = 2;
constexpr uint8_t INFO_RPM_STEPS_OFFSET = 4;
constexpr uint8_t INFO_VOLTAGE_OFFSET = 5;
constexpr uint8_t INFO_CURRENT_OFFSET = 7;

constexpr uint8_t SLOW_TELEMETRY_INTERVAL = 5;
constexpr float HW4_NTC_GAMMA = 0.00025316455696f;
constexpr float HW4_NTC_DELTA = 0.00296226896087f;
constexpr float MIN_TEMPERATURE_C = -50.0f;
constexpr float MAX_TEMPERATURE_C = 300.0f;
constexpr uint32_t MAX_RPM = 10000000UL;
constexpr uint32_t MAX_VOLTAGE_MV = 100000UL;
constexpr uint32_t MAX_CURRENT_MA = 500000UL;
}

SerialHobbywingV4_TLM::SerialHobbywingV4_TLM(Stream &outputPort, Stream &inputPort)
    : SerialIO(&outputPort, &inputPort)
{
}

uint32_t SerialHobbywingV4_TLM::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    (void)frameAvailable;
    (void)frameMissed;
    (void)channelData;
    return DURATION_IMMEDIATELY;
}

void SerialHobbywingV4_TLM::sendQueuedData(uint32_t maxBytesToSend)
{
    (void)maxBytesToSend;
    const uint32_t now = millis();
    if (framePosition != 0 && partialFrameTimedOut(now, lastReceivedByteMs))
    {
        resetParser();
    }
}

void SerialHobbywingV4_TLM::processBytes(uint8_t *bytes, uint16_t size)
{
    for (uint16_t i = 0; i < size; ++i)
    {
        processByte(bytes[i]);
    }
}

void SerialHobbywingV4_TLM::processByte(uint8_t value)
{
    const uint32_t now = millis();

    if (framePosition != 0 && partialFrameTimedOut(now, lastReceivedByteMs))
    {
        resetParser();
    }

    if (framePosition == 0)
    {
        if (value == TRAILING_MARKER)
        {
            return;
        }

        if (value != SYNC_BYTE)
        {
            return;
        }

        frame[framePosition++] = value;
        lastReceivedByteMs = now;
        return;
    }

    if (framePosition >= sizeof(frame))
    {
        resynchronizeParser();
        return;
    }

    frame[framePosition++] = value;
    lastReceivedByteMs = now;

    if (framePosition == INFO_FRAME_LENGTH && validateInfoFrame())
    {
        applyCalibrationFrame();
        resetParser();
        return;
    }

    if (framePosition != DATA_FRAME_LENGTH)
    {
        return;
    }

    if (validateDataFrame() && decodeDataFrame())
    {
        integrateConsumedCapacity(now);
        scheduleTelemetry();
        resetParser();
    }
    else
    {
        resynchronizeParser();
    }
}

void SerialHobbywingV4_TLM::resetParser()
{
    framePosition = 0;
}

void SerialHobbywingV4_TLM::resynchronizeParser()
{
    while (framePosition != 0)
    {
        uint8_t syncPosition = 1;
        while (syncPosition < framePosition && frame[syncPosition] != SYNC_BYTE)
        {
            ++syncPosition;
        }

        if (syncPosition == framePosition)
        {
            resetParser();
            return;
        }

        framePosition -= syncPosition;
        std::memmove(frame, frame + syncPosition, framePosition);
        return;
    }
}

bool SerialHobbywingV4_TLM::validateInfoFrame() const
{
    // Sync markers, RPM steps of 1 and the trailing marker identify an INFO frame.
    // Individual constants are range-checked when applied, so a single zero field
    // does not drop the frame and lose the voltage scale.
    return frame[0] == SYNC_BYTE && frame[1] == SYNC_BYTE &&
        frame[INFO_RPM_STEPS_OFFSET] == 1 && frame[12] == TRAILING_MARKER;
}

bool SerialHobbywingV4_TLM::validateDataFrame() const
{
    if (frame[0] != SYNC_BYTE)
    {
        return false;
    }

    if (frame[THROTTLE_OFFSET] >= 4 || frame[PWM_OFFSET] >= 4)
    {
        return false;
    }

    if (frame[VOLTAGE_ADC_OFFSET] >= 0x10 || frame[CURRENT_ADC_OFFSET] >= 0x10 ||
        frame[FET_TEMP_ADC_OFFSET] >= 0x10 || frame[CAP_TEMP_ADC_OFFSET] >= 0x10)
    {
        return false;
    }

    const uint16_t rawThrottle = readU16BE(frame + THROTTLE_OFFSET);
    const uint16_t rawPwm = readU16BE(frame + PWM_OFFSET);
    const uint16_t throttleLimit = calibration.throttleRange == 0 ? 1000 : calibration.throttleRange;
    if (rawThrottle > throttleLimit || rawPwm > throttleLimit)
    {
        return false;
    }

    const uint32_t rpm = readU24BE(frame + RPM_OFFSET);
    if (rpm > MAX_RPM)
    {
        return false;
    }

    return true;
}

void SerialHobbywingV4_TLM::applyCalibrationFrame()
{
    // Each constant is applied only when non-zero so a partial INFO frame keeps
    // the previously decoded scales.
    const uint16_t throttleRange = readU16BE(frame + INFO_THROTTLE_RANGE_OFFSET);
    if (throttleRange != 0)
    {
        calibration.throttleRange = throttleRange;
    }

    const uint8_t voltageNumerator = frame[INFO_VOLTAGE_OFFSET];
    const uint8_t voltageDenominator = frame[INFO_VOLTAGE_OFFSET + 1];
    if (voltageNumerator != 0 && voltageDenominator != 0)
    {
        calibration.voltageScale = static_cast<float>(voltageNumerator) /
            (static_cast<float>(voltageDenominator) * 10.0f);
    }

    const uint8_t currentNumerator = frame[INFO_CURRENT_OFFSET];
    const uint8_t currentDenominator = frame[INFO_CURRENT_OFFSET + 1];
    if (currentNumerator != 0 && currentDenominator != 0)
    {
        calibration.currentScale = static_cast<float>(currentNumerator) /
            static_cast<float>(currentDenominator);
        calibration.currentOffset = static_cast<float>(frame[INFO_CURRENT_OFFSET + 2]) / calibration.currentScale;
    }

    calibration.valid = std::isfinite(calibration.voltageScale) && calibration.voltageScale > 0.0f &&
        std::isfinite(calibration.currentScale) && calibration.currentScale > 0.0f &&
        std::isfinite(calibration.currentOffset) && calibration.currentOffset >= 0.0f;
}

bool SerialHobbywingV4_TLM::decodeDataFrame()
{
    decoded = {};
    decoded.packetCounter = readU24BE(frame + COUNTER_OFFSET);

    const uint16_t rawThrottle = readU16BE(frame + THROTTLE_OFFSET);
    const uint16_t rawPwm = readU16BE(frame + PWM_OFFSET);
    const uint16_t voltageAdc = readU16BE(frame + VOLTAGE_ADC_OFFSET);
    const uint16_t currentAdc = readU16BE(frame + CURRENT_ADC_OFFSET);
    const uint16_t fetTempAdc = readU16BE(frame + FET_TEMP_ADC_OFFSET);
    const uint16_t capTempAdc = readU16BE(frame + CAP_TEMP_ADC_OFFSET);

    decoded.throttlePermille = scaleToPermille(rawThrottle, calibration.throttleRange);
    decoded.pwmPermille = scaleToPermille(rawPwm, calibration.throttleRange);
    decoded.rpm = readU24BE(frame + RPM_OFFSET);

    int16_t fetTemperatureDeciCelsius = 0;
    int16_t capTemperatureDeciCelsius = 0;
    if (!decodeTemperatureDeciCelsius(fetTempAdc, fetTemperatureDeciCelsius) ||
        !decodeTemperatureDeciCelsius(capTempAdc, capTemperatureDeciCelsius))
    {
        return false;
    }

    decoded.fetTemperatureDeciCelsius = fetTemperatureDeciCelsius;
    decoded.capTemperatureDeciCelsius = capTemperatureDeciCelsius;
    decoded.hasTemperature = true;

    if (!calibration.valid)
    {
        return true;
    }

    const float voltageMv = static_cast<float>(voltageAdc) * calibration.voltageScale * 1000.0f;
    float currentMa = 0.0f;
    if (static_cast<float>(currentAdc) > calibration.currentOffset)
    {
        currentMa = (static_cast<float>(currentAdc) - calibration.currentOffset) * calibration.currentScale * 1000.0f;
    }

    if (rawThrottle == 0)
    {
        currentMa = 0.0f;
    }

    if (!std::isfinite(voltageMv) || !std::isfinite(currentMa) || voltageMv < 0.0f || currentMa < 0.0f)
    {
        return false;
    }

    const uint32_t voltageMillivolts = static_cast<uint32_t>(std::lround(voltageMv));
    const uint32_t currentMilliamps = static_cast<uint32_t>(std::lround(currentMa));
    if (voltageMillivolts > MAX_VOLTAGE_MV || currentMilliamps > MAX_CURRENT_MA)
    {
        return false;
    }

    decoded.voltageMillivolts = saturateU16(voltageMillivolts);
    decoded.currentMilliamps = currentMilliamps;
    decoded.hasBattery = true;
    return true;
}

void SerialHobbywingV4_TLM::integrateConsumedCapacity(uint32_t nowMs)
{
    if (!decoded.hasBattery)
    {
        return;
    }

    if (!hasCurrentSample)
    {
        hasCurrentSample = true;
        lastCurrentSampleMs = nowMs;
        return;
    }

    const uint32_t deltaMs = nowMs - lastCurrentSampleMs;
    lastCurrentSampleMs = nowMs;
    if (deltaMs == 0 || deltaMs > MAX_CAPACITY_SAMPLE_GAP_MS)
    {
        return;
    }

    if (consumedMah >= MAX_CRSF_CAPACITY_MAH)
    {
        consumedMah = MAX_CRSF_CAPACITY_MAH;
        consumedMilliampMilliseconds = static_cast<uint64_t>(MAX_CRSF_CAPACITY_MAH) * 3600000ULL;
        return;
    }

    const uint64_t increment = static_cast<uint64_t>(decoded.currentMilliamps) * deltaMs;
    const uint64_t maxAccumulator = static_cast<uint64_t>(MAX_CRSF_CAPACITY_MAH) * 3600000ULL;
    consumedMilliampMilliseconds = std::min(consumedMilliampMilliseconds + increment, maxAccumulator);
    consumedMah = static_cast<uint32_t>(std::min<uint64_t>(consumedMilliampMilliseconds / 3600000ULL, MAX_CRSF_CAPACITY_MAH));
}

void SerialHobbywingV4_TLM::scheduleTelemetry()
{
    ++validDataFrameCount;

    if (decoded.hasBattery && (validDataFrameCount & 1U) != 0)
    {
        sendCRSFbattery();
    }
    else
    {
        sendCRSFrpm();
    }

    if (decoded.hasTemperature && validDataFrameCount % SLOW_TELEMETRY_INTERVAL == 0)
    {
        sendCRSFtemp();
    }
}

void SerialHobbywingV4_TLM::sendCRSFbattery()
{
    crsfBatterySensorDetected = true;

    CRSF_MK_FRAME_T(crsf_sensor_battery_t) crsfBattery = {0};
    crsfBattery.p.voltage = htobe16((decoded.voltageMillivolts + 50U) / 100U);
    crsfBattery.p.current = htobe16(saturateU16((decoded.currentMilliamps + 50U) / 100U));
    crsfBattery.p.capacity = htobe24(consumedMah);
    crsfBattery.p.remaining = 0;

    crsfRouter.SetHeaderAndCrc(&crsfBattery.h, CRSF_FRAMETYPE_BATTERY_SENSOR,
        CRSF_FRAME_SIZE(sizeof(crsf_sensor_battery_t)));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfBattery.h);
}

void SerialHobbywingV4_TLM::sendCRSFrpm()
{
    CRSF_MK_FRAME_T(crsf_sensor_rpm_t) crsfRpm = {0};
    crsfRpm.p.source_id = RPM_SOURCE_ID;
    crsfRpm.p.rpm0 = htobe24(decoded.rpm);

    constexpr uint8_t payloadSize = SIZE_8BIT + SIZE_24BIT;
    crsfRouter.SetHeaderAndCrc(&crsfRpm.h, CRSF_FRAMETYPE_RPM, CRSF_FRAME_SIZE(payloadSize));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfRpm.h);
}

void SerialHobbywingV4_TLM::sendCRSFtemp()
{
    CRSF_MK_FRAME_T(crsf_sensor_temp_t) crsfTemperature = {0};
    crsfTemperature.p.source_id = TEMPERATURE_SOURCE_ID;
    crsfTemperature.p.temperature[0] = htobe16(static_cast<uint16_t>(decoded.fetTemperatureDeciCelsius));

    constexpr uint8_t payloadSize = SIZE_8BIT + SIZE_16BIT;
    crsfRouter.SetHeaderAndCrc(&crsfTemperature.h, CRSF_FRAMETYPE_TEMP, CRSF_FRAME_SIZE(payloadSize));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfTemperature.h);
}

uint16_t SerialHobbywingV4_TLM::readU16BE(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) << 8U |
        static_cast<uint16_t>(data[1]);
}

uint32_t SerialHobbywingV4_TLM::readU24BE(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) << 16U |
        static_cast<uint32_t>(data[1]) << 8U |
        static_cast<uint32_t>(data[2]);
}

uint16_t SerialHobbywingV4_TLM::scaleToPermille(uint16_t value, uint16_t range)
{
    const uint16_t safeRange = range == 0 ? 1000 : range;
    return static_cast<uint16_t>((static_cast<uint32_t>(value) * 1000U) / safeRange);
}

bool SerialHobbywingV4_TLM::partialFrameTimedOut(uint32_t now, uint32_t lastReceived)
{
    return static_cast<uint32_t>(now - lastReceived) >= PARTIAL_FRAME_TIMEOUT_MS;
}

bool SerialHobbywingV4_TLM::decodeTemperatureDeciCelsius(uint16_t adc, int16_t &outDeciCelsius)
{
    // NTC thermistor conversion using the HW4 divider constants. The Steinhart-Hart
    // coefficients were referenced from Rotorflight's GPLv3 Hobbywing implementation:
    // https://github.com/rotorflight/rotorflight-firmware/blob/master/src/main/sensors/esc_sensor.c
    const float clippedAdc = static_cast<float>(adc <= 1 ? 1 : (adc >= 4095 ? 4095 : adc));
    const float ratio = clippedAdc / (4096.0f - clippedAdc);
    const float denominator = std::log(ratio) * HW4_NTC_GAMMA + HW4_NTC_DELTA;
    if (!std::isfinite(denominator) || denominator <= 0.0f)
    {
        return false;
    }

    const float temperatureC = 1.0f / denominator - 273.15f;
    if (!std::isfinite(temperatureC) || temperatureC < MIN_TEMPERATURE_C || temperatureC > MAX_TEMPERATURE_C)
    {
        return false;
    }

    outDeciCelsius = static_cast<int16_t>(std::lround(temperatureC * 10.0f));
    return true;
}

uint16_t SerialHobbywingV4_TLM::saturateU16(uint32_t value)
{
    return value > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(value);
}

#endif