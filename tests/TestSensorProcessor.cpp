#define CATCH_CONFIG_MAIN // Define this in only one cpp file
#include "../catch.hpp"

#include "../components/ld2410s/SensorProcessor.cpp"

SCENARIO("SensorProcessor processes bytes correctly", "[SensorProcessor]")
{
    GIVEN("A SensorProcessor with a predefined headerFooterMap")
    {
        std::unordered_map<uint8_t, HeaderDataFooter> headerDataFooterMap = {
            {0x01, {{0x01, 0x02}, 0, 10, {0x03, 0x04}}},
            {0x02, {{0x05}, 0, 10, {0x06}}},
        };
        SensorProcessor sensorProcessor(headerDataFooterMap);

        REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeaderStart);

        WHEN("Processing bytes in the WaitingForHeaderStart state")
        {
            THEN("It transitions to WaitingForFooterStart state when the correct header is received")
            {
                REQUIRE(sensorProcessor.processByte(0x01) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeaderEnd);
                REQUIRE(sensorProcessor.processByte(0x02) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForFooterStart);
            }

            THEN("It remains in WaitingForHeaderStart state when an incorrect header is received")
            {
                REQUIRE(sensorProcessor.processByte(0x01) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeaderEnd);
                REQUIRE(sensorProcessor.processByte(0x03) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeaderStart);
                REQUIRE(sensorProcessor.processByte(0x02) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeaderStart);
            }
        }

        WHEN("Processing bytes in the WaitingForFooterStart state")
        {
            sensorProcessor.processByte(0x01);
            sensorProcessor.processByte(0x02);
            REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForFooterStart);

            THEN("It remains in WaitingForFooterStart state when data matches a different header")
            {
                REQUIRE(sensorProcessor.processByte(0x05) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForFooterStart);
                REQUIRE(sensorProcessor.processByte(0x06) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForFooterStart);
            }

            THEN("It transitions to WaitingForHeaderStart state when the correct footer is received")
            {
                REQUIRE(sensorProcessor.processByte(0x03) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForFooterEnd);
                auto result = sensorProcessor.processByte(0x04);
                REQUIRE(result->data == std::vector<uint8_t>{0x01, 0x02, 0x03, 0x04});
                REQUIRE(result->type == 0x01);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeaderStart);
            }

            THEN("It remains in WaitingForFooterStart state when an incorrect footer is received")
            {
                REQUIRE(sensorProcessor.processByte(0x03) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForFooterEnd);
                REQUIRE(sensorProcessor.processByte(0x05) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForFooterStart);
                REQUIRE(sensorProcessor.processByte(0x04) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForFooterStart);
            }
        }
    }
}