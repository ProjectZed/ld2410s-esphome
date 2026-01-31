#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif

namespace esphome
{
    namespace ld2410s
    {
        // ========================================================================
        // Protocol Constants - HLK-LD2410S User Manual V1.2
        // ========================================================================

        // Data Frame (Manual Page 30)
        static const uint8_t DATA_FRAME_HEADER = 0xF4;
        static const uint8_t DATA_FRAME_FOOTER = 0xF8;

        // Command/Ack Frame (Manual Page 11)
        static const uint8_t CMD_FRAME_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
        static const uint8_t CMD_FRAME_FOOTER[4] = {0x04, 0x03, 0x02, 0x01};

        // Threshold Update Frame (Manual Page 30)
        static const uint8_t THRESHOLD_HEADER[4] = {0xF1, 0xF2, 0xF3, 0xF4};
        static const uint8_t THRESHOLD_FOOTER[4] = {0xF5, 0xF6, 0xF7, 0xF8};

        // ========================================================================
        // Command Codes (Manual Page 11)
        // ========================================================================
        static const uint16_t CMD_READ_PARAMS = 0x0060;      // Read parameters
        static const uint16_t CMD_WRITE_PARAMS = 0x0061;     // Write parameters
        static const uint16_t CMD_CONFIG_MODE = 0x00FF;      // Enable/disable config mode
        static const uint16_t CMD_GET_FW_VERSION = 0x0000;   // Get firmware version
        static const uint16_t CMD_SET_BAUD_RATE = 0x00A1;    // Set baud rate
        static const uint16_t CMD_FACTORY_RESET = 0x00A2;    // Factory reset
        static const uint16_t CMD_RESTART = 0x0003;          // Restart module
        static const uint16_t CMD_AUTO_THRESHOLD = 0x00A3;   // Auto threshold calibration

        // ========================================================================
        // Response Command Codes (MSB set)
        // ========================================================================
        static const uint16_t RESP_READ_PARAMS = 0x0160;
        static const uint16_t RESP_WRITE_PARAMS = 0x0161;
        static const uint16_t RESP_CONFIG_MODE = 0x01FF;
        static const uint16_t RESP_GET_FW_VERSION = 0x0100;

        // ========================================================================
        // End Command Values for Config Mode (Manual Page 18)
        // ========================================================================
        static const uint16_t END_CMD_ENABLE_CONFIG = 0x0001;
        static const uint16_t END_CMD_DISABLE_CONFIG = 0x0002;

        // ========================================================================
        // Parameter Type Codes (Manual Page 19)
        // ========================================================================
        static const uint16_t PARAM_MAX_DISTANCE = 0x0001;
        static const uint16_t PARAM_MIN_DISTANCE = 0x0002;
        static const uint16_t PARAM_NO_DELAY = 0x0003;
        static const uint16_t PARAM_STATUS_FREQ = 0x0004;
        static const uint16_t PARAM_DISTANCE_FREQ = 0x0005;
        static const uint16_t PARAM_RESPONSE_SPEED = 0x0006;

        // ========================================================================
        // Response Speed Values (Manual Page 20)
        // ========================================================================
        static const uint16_t RESPONSE_SPEED_NORMAL = 0x0000;
        static const uint16_t RESPONSE_SPEED_FAST = 0x0001;

        // ========================================================================
        // Legacy Constants (for backward compatibility during transition)
        // ========================================================================
        static const uint16_t READ_FW_CMD = 0x0000;
        static const uint16_t START_CONFIG_MODE_CMD = 0x00FF;
        static const uint16_t END_CONFIG_MODE_CMD = 0x00FF;  // Corrected - was 0x00FE

        // Threshold calibration parameters (Manual Page 29)
        static const uint16_t THRESHOLD_TRIGGER_VALUE = 0x0002;
        static const uint16_t THRESHOLD_RETENTION_VALUE = 0x0001;
        static const uint16_t THRESHOLD_TIME_VALUE = 0x0078;

        // Response speed display strings
        static const std::string RESPONSE_SPEED_NORMAL_STR = "Normal";
        static const std::string RESPONSE_SPEED_FAST_STR = "Fast";

        // ========================================================================
        // Data Structures
        // ========================================================================

        struct Config
        {
            uint16_t max_distance{0};
            uint16_t min_distance{0};
            uint16_t no_delay{0};
            uint16_t status_freq{0};    // Stored in 0.1Hz units
            uint16_t distance_freq{0};  // Stored in 0.1Hz units
            uint16_t response_speed{0};
        };

        struct CmdFrameT
        {
            uint16_t command{0};
            uint8_t data[36];
            uint16_t data_length{0};
        };

        struct CmdAckT
        {
            uint16_t command{0};
            uint8_t data[36];
            uint16_t length{0};
            bool result{false};
        };

        // ========================================================================
        // Frame Type Enumeration
        // ========================================================================
        enum class PackageType
        {
            ACK,
            SHORT_DATA,
            THRESHOLD,
            UNKNOWN
        };

        // ========================================================================
        // State Machine Parse States (uint8_t for memory efficiency)
        // ========================================================================
        enum ParseState : uint8_t
        {
            STATE_IDLE = 0,

            // Data frame states (fixed 8 bytes)
            STATE_DATA_HEADER,
            STATE_DATA_STATE,
            STATE_DATA_MOVING_L,
            STATE_DATA_MOVING_H,
            STATE_DATA_STATIC_L,
            STATE_DATA_STATIC_H,
            STATE_DATA_UNUSED,
            STATE_DATA_FOOTER,

            // Command frame states (variable length)
            STATE_CMD_HEADER,
            STATE_CMD_LENGTH_L,
            STATE_CMD_LENGTH_H,
            STATE_CMD_CMD_L,
            STATE_CMD_CMD_H,
            STATE_CMD_DATA,
            STATE_CMD_FOOTER,

            // Threshold frame states (16 bytes)
            STATE_THRESH_HEADER,
            STATE_THRESH_DATA,
            STATE_THRESH_FOOTER
        };

        // ========================================================================
        // Listener Interface
        // ========================================================================
        class LD2410SListener
        {
        public:
            virtual void on_presence(bool presence) {};
            virtual void on_distance(int distance) {};
            virtual void on_threshold_update(bool running) {};
            virtual void on_threshold_progress(int progress) {};
            virtual void on_fw_version(std::string& fw) {};
        };

        // ========================================================================
        // Main LD2410S Component
        // ========================================================================
        class LD2410S : public uart::UARTDevice, public Component
        {
        public:
            Config pending_config;  // Renamed from new_config

            void setup() override;
            void loop() override;
            float get_setup_priority() const override;

            void register_listener(LD2410SListener* listener) { this->listeners.push_back(listener); };
            void set_config_mode(bool enabled);
            void apply_config();
            void start_auto_threshold_update();

#ifdef USE_NUMBER
            void set_max_distance_number(number::Number* max_distance_number) { this->max_distance_number = max_distance_number; };
            void set_min_distance_number(number::Number* min_distance_number) { this->min_distance_number = min_distance_number; };
            void set_no_delay_number(number::Number* delay_number) { this->no_delay_number = delay_number; };
            void set_status_reporting_freq_number(number::Number* status_reporting_freq_number) { this->status_reporting_freq_number = status_reporting_freq_number; };
            void set_distance_reporting_freq_number(number::Number* distance_reporting_freq_number) { this->distance_reporting_freq_number = distance_reporting_freq_number; };
#endif
#ifdef USE_BUTTON
            void set_enable_config_button(button::Button* button) { this->enable_config_button = button; };
            void set_disable_config_button(button::Button* button) { this->disable_config_button = button; };
            void set_apply_config_button(button::Button* button) { this->apply_config_button = button; };
            void set_auto_threshold_button(button::Button* button) { this->auto_threshold_button = button; };
#endif
#ifdef USE_SELECT
            void set_response_speed_select(select::Select* selector) { this->response_speed_select = selector; };
#endif

        private:
            // State management
            Config device_config;       // Actual config from device
            bool config_mode_active{false};

            // State machine variables
            ParseState state_{STATE_IDLE};
            uint8_t header_idx_{0};      // Header match position
            uint8_t frame_buf_[16];      // Max frame size (threshold frame)
            uint8_t frame_pos_{0};
            uint8_t cmd_data_len_{0};    // Expected data length
            uint8_t cmd_data_idx_{0};    // Current data byte index

            // Parsed values (avoid re-parsing)
            uint16_t moving_dist_{0};
            uint16_t static_dist_{0};
            uint8_t target_state_{0};

            // Listeners and optional components
            std::vector<LD2410SListener*> listeners{};
            bool cmd_active{false};

#ifdef USE_NUMBER
            number::Number* max_distance_number{nullptr};
            number::Number* min_distance_number{nullptr};
            number::Number* no_delay_number{nullptr};
            number::Number* status_reporting_freq_number{nullptr};
            number::Number* distance_reporting_freq_number{nullptr};
#endif
#ifdef USE_BUTTON
            button::Button* enable_config_button{nullptr};
            button::Button* disable_config_button{nullptr};
            button::Button* apply_config_button{nullptr};
            button::Button* auto_threshold_button{nullptr};
#endif
#ifdef USE_SELECT
            select::Select* response_speed_select{nullptr};
#endif

            // Protocol methods
            CmdFrameT prepare_read_config_cmd();
            CmdFrameT prepare_apply_config_cmd();
            CmdFrameT prepare_threshold_cmd();
            CmdFrameT prepare_read_fw_cmd();
            void send_command(CmdFrameT cmd_frame);

            // State machine methods
            void process_byte(uint8_t byte);
            void dispatch_data_frame();
            void process_cmd_ack();
            void process_threshold_frame();

            // State management methods
            void read_config_from_device();
            void publish_config_to_ha();

            // Processing methods
            PackageType read_line(uint8_t data, uint8_t* buffer, size_t pos);
            bool process_cmd_ack_package(uint8_t* buffer, int len);
            void process_data_package(PackageType type, uint8_t* buffer, size_t pos);

            // Helper methods
            int read_int(uint8_t* buffer, size_t pos, size_t len)
            {
                unsigned int ret = 0;
                int shift = 0;
                for (size_t i = 0; i < len; i++)
                {
                    ret |= static_cast<unsigned int>(buffer[pos + i]) << shift;
                    shift += 8;
                }
                return ret;
            }

            int two_byte_to_int(uint8_t firstbyte, uint8_t secondbyte) { return (secondbyte << 8) + firstbyte; }

            CmdAckT parse_ack(uint8_t* buffer, size_t length);
            void process_config_read_ack(uint8_t* data);
            void process_read_fw_ack(uint8_t* data);
            void process_short_data_package(uint8_t* data);
            void process_threshold_package(uint8_t* data);
        };
    }
}
