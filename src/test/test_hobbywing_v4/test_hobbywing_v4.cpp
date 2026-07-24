#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include <unity.h>

#include "CRSFRouter.h"
#include "rx-serial/SerialHobbywingV4_TLM.h"
#include "common.h"

CRSFRouter crsfRouter;
bool crsfBatterySensorDetected = false;

using std::min;
#include "../../src/rx-serial/SerialIO.cpp"
#include "../../src/rx-serial/SerialHobbywingV4_TLM.cpp"

static_assert(PROTOCOL_HOBBYWING_V4_TLM == 11, "Unexpected Hobbywing V4 protocol value");
static_assert(PROTOCOL_HOBBYWING_V4_TLM < 16, "Hobbywing V4 protocol does not fit persisted storage");

namespace
{
class MockStream : public Stream
{
public:
    int available() override { return static_cast<int>(input.size() - readPosition); }
    int read() override { return readPosition < input.size() ? input[readPosition++] : -1; }
    int peek() override { return readPosition < input.size() ? input[readPosition] : -1; }
    void flush() override {}

    size_t write(uint8_t value) override
    {
        output.push_back(value);
        return 1;
    }

    size_t write(const uint8_t *data, size_t length) override
    {
        output.insert(output.end(), data, data + length);
        return length;
    }

    std::vector<uint8_t> input;
    std::vector<uint8_t> output;

private:
    size_t readPosition = 0;
};

class MockConnector : public CRSFConnector
{
public:
    MockConnector()
    {
        addDevice(CRSF_ADDRESS_RADIO_TRANSMITTER);
    }

    void forwardMessage(const crsf_header_t *message) override
    {
        const uint8_t *bytes = reinterpret_cast<const uint8_t *>(message);
        frames.emplace_back(bytes, bytes + message->frame_size + CRSF_FRAME_NOT_COUNTED_BYTES);
    }

    std::vector<std::vector<uint8_t>> frames;
};

using HobbywingInfoFrame = std::array<uint8_t, 13>;
using HobbywingDataFrame = std::array<uint8_t, 19>;

HobbywingInfoFrame validInfoFrame()
{
    return {{
        0x9B, 0x9B,
        0x03, 0xE8,
        0x01,
        0x01, 0x02,
        0x01, 0x0A, 0x0A,
        0x00, 0x00,
        0xB9
    }};
}

HobbywingDataFrame validDataFrame()
{
    return {{
        0x9B,
        0x00, 0x00, 0x01,
        0x01, 0xF4,
        0x01, 0x2C,
        0x00, 0x30, 0x39,
        0x01, 0x90,
        0x00, 0x96,
        0x08, 0x00,
        0x04, 0x00
    }};
}

HobbywingDataFrame zeroThrottleFrame()
{
    HobbywingDataFrame frame = validDataFrame();
    frame[4] = 0x00;
    frame[5] = 0x00;
    return frame;
}

struct Fixture
{
    Fixture()
        : serial(output, input)
    {
        crsfRouter.addConnector(&connector);
    }

    ~Fixture() { crsfRouter.removeConnector(&connector); }

    MockStream output;
    MockStream input;
    MockConnector connector;
    SerialHobbywingV4_TLM serial;
};
}

class SerialHobbywingV4TlmTestAccess
{
public:
    static void process(SerialHobbywingV4_TLM &serial, const uint8_t *data, size_t length)
    {
        serial.processBytes(const_cast<uint8_t *>(data), static_cast<uint16_t>(length));
    }

    static void setLastReceivedByteMs(SerialHobbywingV4_TLM &serial, uint32_t value) { serial.lastReceivedByteMs = value; }
    static void setLastCurrentSampleMs(SerialHobbywingV4_TLM &serial, uint32_t value)
    {
        serial.hasCurrentSample = true;
        serial.lastCurrentSampleMs = value;
    }
    static uint8_t framePosition(const SerialHobbywingV4_TLM &serial) { return serial.framePosition; }
    static bool calibrationValid(const SerialHobbywingV4_TLM &serial) { return serial.calibration.valid; }
    static uint16_t throttleRange(const SerialHobbywingV4_TLM &serial) { return serial.calibration.throttleRange; }
    static float voltageScale(const SerialHobbywingV4_TLM &serial) { return serial.calibration.voltageScale; }
    static float currentScale(const SerialHobbywingV4_TLM &serial) { return serial.calibration.currentScale; }
    static float currentOffset(const SerialHobbywingV4_TLM &serial) { return serial.calibration.currentOffset; }
    static uint32_t validDataFrameCount(const SerialHobbywingV4_TLM &serial) { return serial.validDataFrameCount; }
    static uint32_t packetCounter(const SerialHobbywingV4_TLM &serial) { return serial.decoded.packetCounter; }
    static uint16_t throttlePermille(const SerialHobbywingV4_TLM &serial) { return serial.decoded.throttlePermille; }
    static uint16_t pwmPermille(const SerialHobbywingV4_TLM &serial) { return serial.decoded.pwmPermille; }
    static uint32_t rpm(const SerialHobbywingV4_TLM &serial) { return serial.decoded.rpm; }
    static uint16_t voltageMillivolts(const SerialHobbywingV4_TLM &serial) { return serial.decoded.voltageMillivolts; }
    static uint32_t currentMilliamps(const SerialHobbywingV4_TLM &serial) { return serial.decoded.currentMilliamps; }
    static int16_t fetTemperatureDeciCelsius(const SerialHobbywingV4_TLM &serial) { return serial.decoded.fetTemperatureDeciCelsius; }
    static int16_t capTemperatureDeciCelsius(const SerialHobbywingV4_TLM &serial) { return serial.decoded.capTemperatureDeciCelsius; }
    static bool hasBattery(const SerialHobbywingV4_TLM &serial) { return serial.decoded.hasBattery; }
    static uint32_t consumedMah(const SerialHobbywingV4_TLM &serial) { return serial.consumedMah; }
};

void test_valid_info_frame_updates_calibration()
{
    Fixture fixture;
    const HobbywingInfoFrame frame = validInfoFrame();

    SerialHobbywingV4TlmTestAccess::process(fixture.serial, frame.data(), frame.size());

    TEST_ASSERT_TRUE(SerialHobbywingV4TlmTestAccess::calibrationValid(fixture.serial));
    TEST_ASSERT_EQUAL_UINT16(1000, SerialHobbywingV4TlmTestAccess::throttleRange(fixture.serial));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.05f, SerialHobbywingV4TlmTestAccess::voltageScale(fixture.serial));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1f, SerialHobbywingV4TlmTestAccess::currentScale(fixture.serial));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, SerialHobbywingV4TlmTestAccess::currentOffset(fixture.serial));
    TEST_ASSERT_EQUAL_UINT8(0, SerialHobbywingV4TlmTestAccess::framePosition(fixture.serial));
}

void test_data_frame_without_calibration_only_publishes_rpm()
{
    Fixture fixture;
    const HobbywingDataFrame frame = validDataFrame();

    SerialHobbywingV4TlmTestAccess::process(fixture.serial, frame.data(), frame.size());

    TEST_ASSERT_FALSE(SerialHobbywingV4TlmTestAccess::hasBattery(fixture.serial));
    TEST_ASSERT_FALSE(crsfBatterySensorDetected);
    TEST_ASSERT_EQUAL_UINT32(1, fixture.connector.frames.size());
    TEST_ASSERT_EQUAL_HEX8(CRSF_FRAMETYPE_RPM, fixture.connector.frames[0][2]);
}

void test_valid_data_frame_decodes_big_endian_fields()
{
    Fixture fixture;
    const HobbywingInfoFrame info = validInfoFrame();
    const HobbywingDataFrame data = validDataFrame();

    SerialHobbywingV4TlmTestAccess::process(fixture.serial, info.data(), info.size());
    SerialHobbywingV4TlmTestAccess::process(fixture.serial, data.data(), data.size());

    TEST_ASSERT_EQUAL_UINT32(1, SerialHobbywingV4TlmTestAccess::packetCounter(fixture.serial));
    TEST_ASSERT_EQUAL_UINT16(500, SerialHobbywingV4TlmTestAccess::throttlePermille(fixture.serial));
    TEST_ASSERT_EQUAL_UINT16(300, SerialHobbywingV4TlmTestAccess::pwmPermille(fixture.serial));
    TEST_ASSERT_EQUAL_UINT32(12345, SerialHobbywingV4TlmTestAccess::rpm(fixture.serial));
    TEST_ASSERT_EQUAL_UINT16(20000, SerialHobbywingV4TlmTestAccess::voltageMillivolts(fixture.serial));
    TEST_ASSERT_EQUAL_UINT32(5000, SerialHobbywingV4TlmTestAccess::currentMilliamps(fixture.serial));
    TEST_ASSERT_EQUAL_INT16(644, SerialHobbywingV4TlmTestAccess::fetTemperatureDeciCelsius(fixture.serial));
    TEST_ASSERT_EQUAL_INT16(994, SerialHobbywingV4TlmTestAccess::capTemperatureDeciCelsius(fixture.serial));
    TEST_ASSERT_TRUE(SerialHobbywingV4TlmTestAccess::hasBattery(fixture.serial));
    TEST_ASSERT_TRUE(crsfBatterySensorDetected);
}

void test_zero_throttle_forces_current_to_zero()
{
    Fixture fixture;
    const HobbywingInfoFrame info = validInfoFrame();
    const HobbywingDataFrame data = zeroThrottleFrame();

    SerialHobbywingV4TlmTestAccess::process(fixture.serial, info.data(), info.size());
    SerialHobbywingV4TlmTestAccess::process(fixture.serial, data.data(), data.size());

    TEST_ASSERT_EQUAL_UINT32(0, SerialHobbywingV4TlmTestAccess::currentMilliamps(fixture.serial));
}

void test_consumed_capacity_integration_and_rollover()
{
    Fixture fixture;
    const HobbywingInfoFrame info = validInfoFrame();
    const HobbywingDataFrame data = validDataFrame();

    SerialHobbywingV4TlmTestAccess::process(fixture.serial, info.data(), info.size());
    SerialHobbywingV4TlmTestAccess::setLastCurrentSampleMs(
        fixture.serial, static_cast<uint32_t>(millis()) - 1000U);
    SerialHobbywingV4TlmTestAccess::process(fixture.serial, data.data(), data.size());
    TEST_ASSERT_EQUAL_UINT32(1, SerialHobbywingV4TlmTestAccess::consumedMah(fixture.serial));

    SerialHobbywingV4TlmTestAccess::setLastCurrentSampleMs(fixture.serial, UINT32_MAX - 499U);
    SerialHobbywingV4TlmTestAccess::process(fixture.serial, data.data(), data.size());
    TEST_ASSERT_TRUE(SerialHobbywingV4TlmTestAccess::consumedMah(fixture.serial) >= 1);
}

void test_optional_trailing_marker_is_ignored()
{
    Fixture fixture;
    const HobbywingInfoFrame info = validInfoFrame();
    const HobbywingDataFrame data = validDataFrame();
    std::array<uint8_t, 20> framed = {};
    std::copy(data.begin(), data.end(), framed.begin());
    framed[19] = 0xB9;

    SerialHobbywingV4TlmTestAccess::process(fixture.serial, info.data(), info.size());
    SerialHobbywingV4TlmTestAccess::process(fixture.serial, framed.data(), framed.size());
    SerialHobbywingV4TlmTestAccess::process(fixture.serial, data.data(), data.size());

    TEST_ASSERT_EQUAL_UINT32(2, SerialHobbywingV4TlmTestAccess::validDataFrameCount(fixture.serial));
}

void test_partial_frame_can_span_multiple_reads_and_timeout()
{
    Fixture fixture;
    const HobbywingDataFrame data = validDataFrame();

    SerialHobbywingV4TlmTestAccess::process(fixture.serial, data.data(), 7);
    TEST_ASSERT_EQUAL_UINT8(7, SerialHobbywingV4TlmTestAccess::framePosition(fixture.serial));

    SerialHobbywingV4TlmTestAccess::setLastReceivedByteMs(
        fixture.serial, static_cast<uint32_t>(millis()) - 50U);
    fixture.serial.sendQueuedData(0);
    TEST_ASSERT_EQUAL_UINT8(0, SerialHobbywingV4TlmTestAccess::framePosition(fixture.serial));

    SerialHobbywingV4TlmTestAccess::process(fixture.serial, data.data(), data.size());
    TEST_ASSERT_EQUAL_UINT32(1, SerialHobbywingV4TlmTestAccess::validDataFrameCount(fixture.serial));
}

void test_multiple_frames_in_one_read_and_serial_input_chunking()
{
    Fixture fixture;
    const HobbywingInfoFrame info = validInfoFrame();
    const HobbywingDataFrame data = validDataFrame();

    fixture.input.input.insert(fixture.input.input.end(), info.begin(), info.end());
    fixture.input.input.insert(fixture.input.input.end(), data.begin(), data.end());
    fixture.input.input.insert(fixture.input.input.end(), data.begin(), data.end());
    fixture.serial.processSerialInput();

    TEST_ASSERT_EQUAL_UINT32(2, SerialHobbywingV4TlmTestAccess::validDataFrameCount(fixture.serial));
    TEST_ASSERT_EQUAL_INT(0, fixture.input.available());
}

void test_noise_dropped_bytes_and_recovery_after_malformed_input()
{
    Fixture fixture;
    HobbywingDataFrame malformed = validDataFrame();
    const HobbywingInfoFrame info = validInfoFrame();
    const HobbywingDataFrame valid = validDataFrame();
    malformed[11] = 0x20;

    const uint8_t noise[] = {0x00, 0x7E, 0xB9, 0x12};
    SerialHobbywingV4TlmTestAccess::process(fixture.serial, noise, sizeof(noise));
    SerialHobbywingV4TlmTestAccess::process(fixture.serial, malformed.data(), malformed.size() - 1);
    SerialHobbywingV4TlmTestAccess::process(fixture.serial, info.data(), info.size());
    SerialHobbywingV4TlmTestAccess::process(fixture.serial, valid.data(), valid.size());

    TEST_ASSERT_EQUAL_UINT32(1, SerialHobbywingV4TlmTestAccess::validDataFrameCount(fixture.serial));
}

void test_invalid_info_and_invalid_data_are_rejected()
{
    Fixture fixture;
    HobbywingInfoFrame invalidInfo = validInfoFrame();
    HobbywingDataFrame invalidData = validDataFrame();
    invalidInfo[4] = 0x02;
    invalidData[13] = 0x20;

    SerialHobbywingV4TlmTestAccess::process(fixture.serial, invalidInfo.data(), invalidInfo.size());
    TEST_ASSERT_FALSE(SerialHobbywingV4TlmTestAccess::calibrationValid(fixture.serial));

    SerialHobbywingV4TlmTestAccess::process(fixture.serial, invalidData.data(), invalidData.size());
    TEST_ASSERT_TRUE(fixture.connector.frames.empty());
    TEST_ASSERT_FALSE(crsfBatterySensorDetected);
}

void test_crsf_encoding_and_battery_detection_behavior()
{
    Fixture fixture;
    const HobbywingInfoFrame info = validInfoFrame();
    const HobbywingDataFrame data = validDataFrame();

    SerialHobbywingV4TlmTestAccess::process(fixture.serial, info.data(), info.size());
    for (uint8_t i = 0; i < 5; ++i)
    {
        SerialHobbywingV4TlmTestAccess::process(fixture.serial, data.data(), data.size());
    }

    TEST_ASSERT_TRUE(crsfBatterySensorDetected);
    TEST_ASSERT_EQUAL_HEX8(CRSF_FRAMETYPE_BATTERY_SENSOR, fixture.connector.frames[0][2]);
    TEST_ASSERT_EQUAL_HEX8(CRSF_FRAMETYPE_RPM, fixture.connector.frames[1][2]);
    TEST_ASSERT_EQUAL_HEX8(CRSF_FRAMETYPE_TEMP, fixture.connector.frames[5][2]);

    const uint8_t expectedBattery[] = {0xC8, 0x0A, 0x08, 0x00, 0xC8, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x3E};
    const uint8_t expectedRpm[] = {0xC8, 0x06, 0x0C, 0x00, 0x00, 0x30, 0x39, 0x33};
    const uint8_t expectedTemp[] = {0xC8, 0x05, 0x0D, 0x00, 0x02, 0x84, 0xD4};

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedBattery, fixture.connector.frames[0].data(), sizeof(expectedBattery));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedRpm, fixture.connector.frames[1].data(), sizeof(expectedRpm));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expectedTemp, fixture.connector.frames[5].data(), sizeof(expectedTemp));
}

void test_driver_never_writes_to_uart()
{
    Fixture fixture;

    TEST_ASSERT_EQUAL_UINT32(DURATION_IMMEDIATELY, fixture.serial.sendRCFrame(false, false, nullptr));
    fixture.serial.sendQueuedData(128);

    TEST_ASSERT_TRUE(fixture.output.output.empty());
}

void setUp()
{
    crsfBatterySensorDetected = false;
}

void tearDown()
{
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_valid_info_frame_updates_calibration);
    RUN_TEST(test_data_frame_without_calibration_only_publishes_rpm);
    RUN_TEST(test_valid_data_frame_decodes_big_endian_fields);
    RUN_TEST(test_zero_throttle_forces_current_to_zero);
    RUN_TEST(test_consumed_capacity_integration_and_rollover);
    RUN_TEST(test_optional_trailing_marker_is_ignored);
    RUN_TEST(test_partial_frame_can_span_multiple_reads_and_timeout);
    RUN_TEST(test_multiple_frames_in_one_read_and_serial_input_chunking);
    RUN_TEST(test_noise_dropped_bytes_and_recovery_after_malformed_input);
    RUN_TEST(test_invalid_info_and_invalid_data_are_rejected);
    RUN_TEST(test_crsf_encoding_and_battery_detection_behavior);
    RUN_TEST(test_driver_never_writes_to_uart);
    return UNITY_END();
}