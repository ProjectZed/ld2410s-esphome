#define CATCH_CONFIG_MAIN // Define this in only one cpp file
#include "../catch.hpp"

#include "../components/ld2410s/SensorProcessor.cpp"

SCENARIO("SensorProcessor processes bytes correctly", "[SensorProcessor]") {
    GIVEN("A SensorProcessor with a predefined headerFooterMap") {
        std::unordered_map<uint8_t, HeaderFooter> headerFooterMap = {
            {0x01, {{0x01, 0x02}, {0x03, 0x04}}},
            {0x02, {{0x05}, {0x06}}},
        };
        SensorProcessor sensorProcessor(headerFooterMap);

        REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeader);

        WHEN("Processing bytes in the WaitingForHeader state") {
            THEN("It transitions to ReadingData state when the correct header is received") {
                REQUIRE(sensorProcessor.processByte(0x01) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeader);
                REQUIRE(sensorProcessor.processByte(0x02) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::ReadingData);
            }

            THEN("It remains in WaitingForHeader state when an incorrect header is received") {
                REQUIRE(sensorProcessor.processByte(0x01) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeader);
                REQUIRE(sensorProcessor.processByte(0x03) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeader);
                REQUIRE(sensorProcessor.processByte(0x02) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeader);
            }
        }

        WHEN("Processing bytes in the ReadingData state") {
            sensorProcessor.processByte(0x01);
            sensorProcessor.processByte(0x02);
            REQUIRE(sensorProcessor.getState() == ProcessorState::ReadingData);

            THEN("It remains in ReadingData state when data matches a different header") {
                REQUIRE(sensorProcessor.processByte(0x05) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::ReadingData);
                REQUIRE(sensorProcessor.processByte(0x06) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::ReadingData);
            }

            THEN("It transitions to WaitingForHeader state when the correct footer is received") {
                REQUIRE(sensorProcessor.processByte(0x03) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForFooter);
                auto result = sensorProcessor.processByte(0x04);
                REQUIRE(result->data == std::vector<uint8_t>{0x01, 0x02, 0x03, 0x04});
                REQUIRE(result->type == 0x01);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForHeader);
            }

            THEN("It remains in ReadingData state when an incorrect footer is received") {
                REQUIRE(sensorProcessor.processByte(0x03) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::WaitingForFooter);
                REQUIRE(sensorProcessor.processByte(0x05) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::ReadingData);
                REQUIRE(sensorProcessor.processByte(0x04) == nullptr);
                REQUIRE(sensorProcessor.getState() == ProcessorState::ReadingData);
            }
        }
    }
}