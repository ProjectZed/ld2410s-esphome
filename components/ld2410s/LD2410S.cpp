#include "esphome/core/log.h"
#include "LD2410S.h"

namespace esphome
{
    namespace ld2410s
    {

        static const char* TAG = "ld2410s";

        // ========================================================================
        // Setup - Initialize and read current config from device
        // ========================================================================
        void LD2410S::setup()
        {
            // Read firmware version
            this->send_command(this->prepare_read_fw_cmd());

            // Read current configuration from device
            this->read_config_from_device();
        }

        // ========================================================================
        // Main Loop - State machine for byte processing
        // ========================================================================
        void LD2410S::loop()
        {
            if (!this->cmd_active)
            {
                while (available())
                {
                    uint8_t byte = read();
                    this->process_byte(byte);
                }
            }
        }

        // ========================================================================
        // State Machine - Efficient byte-by-byte parsing
        // ========================================================================
        void LD2410S::process_byte(uint8_t byte)
        {
            switch (this->state_)
            {
                // ---------------------------------------------------------------------
                // IDLE - Looking for frame start
                // ---------------------------------------------------------------------
                case STATE_IDLE:
                    if (byte == 0xF4)
                    {
                        // Data frame start
                        this->frame_buf_[0] = byte;
                        this->frame_pos_ = 1;
                        this->state_ = STATE_DATA_STATE;
                    }
                    else if (byte == 0xFD)
                    {
                        // Command frame start
                        this->frame_buf_[0] = byte;
                        this->frame_pos_ = 1;
                        this->header_idx_ = 1;
                        this->state_ = STATE_CMD_HEADER;
                    }
                    else if (byte == 0xF1)
                    {
                        // Threshold frame start
                        this->frame_buf_[0] = byte;
                        this->frame_pos_ = 1;
                        this->header_idx_ = 1;
                        this->state_ = STATE_THRESH_HEADER;
                    }
                    break;

                // ---------------------------------------------------------------------
                // Data Frame (8 bytes): F4 [state] [moving_L] [moving_H] [static_L] [static_H] [unused] F8
                // ---------------------------------------------------------------------
                case STATE_DATA_STATE:
                    this->target_state_ = byte;
                    this->frame_buf_[this->frame_pos_++] = byte;
                    this->state_ = STATE_DATA_MOVING_L;
                    break;

                case STATE_DATA_MOVING_L:
                    this->moving_dist_ = byte;
                    this->frame_buf_[this->frame_pos_++] = byte;
                    this->state_ = STATE_DATA_MOVING_H;
                    break;

                case STATE_DATA_MOVING_H:
                    this->moving_dist_ |= (byte << 8);
                    this->frame_buf_[this->frame_pos_++] = byte;
                    this->state_ = STATE_DATA_STATIC_L;
                    break;

                case STATE_DATA_STATIC_L:
                    this->static_dist_ = byte;
                    this->frame_buf_[this->frame_pos_++] = byte;
                    this->state_ = STATE_DATA_STATIC_H;
                    break;

                case STATE_DATA_STATIC_H:
                    this->static_dist_ |= (byte << 8);
                    this->frame_buf_[this->frame_pos_++] = byte;
                    this->state_ = STATE_DATA_UNUSED;
                    break;

                case STATE_DATA_UNUSED:
                    this->frame_buf_[this->frame_pos_++] = byte;
                    this->state_ = STATE_DATA_FOOTER;
                    break;

                case STATE_DATA_FOOTER:
                    if (byte == 0xF8)
                    {
                        // Valid frame - dispatch to listeners
                        this->dispatch_data_frame();
                    }
                    // Frame complete, return to IDLE
                    this->state_ = STATE_IDLE;
                    break;

                // ---------------------------------------------------------------------
                // Command Frame: FD FC FB FA [len_L] [len_H] [cmd_L] [cmd_H] [data...] [04 03 02 01]
                // ---------------------------------------------------------------------
                case STATE_CMD_HEADER:
                    this->frame_buf_[this->frame_pos_++] = byte;
                    // Expect: 0xFD 0xFC 0xFB 0xFA
                    if (byte == CMD_FRAME_HEADER[this->header_idx_])
                    {
                        if (++this->header_idx_ == 4)
                        {
                            this->header_idx_ = 0;
                            this->state_ = STATE_CMD_LENGTH_L;
                        }
                    }
                    else
                    {
                        this->state_ = STATE_IDLE;  // Invalid header
                    }
                    break;

                case STATE_CMD_LENGTH_L:
                    this->cmd_data_len_ = byte;
                    this->frame_buf_[this->frame_pos_++] = byte;
                    this->state_ = STATE_CMD_LENGTH_H;
                    break;

                case STATE_CMD_LENGTH_H:
                    this->cmd_data_len_ |= (byte << 8);
                    this->frame_buf_[this->frame_pos_++] = byte;
                    this->state_ = STATE_CMD_CMD_L;
                    break;

                case STATE_CMD_CMD_L:
                    this->frame_buf_[this->frame_pos_++] = byte;
                    this->state_ = STATE_CMD_CMD_H;
                    break;

                case STATE_CMD_CMD_H:
                    this->frame_buf_[this->frame_pos_++] = byte;
                    this->cmd_data_idx_ = 0;
                    this->state_ = (this->cmd_data_len_ > 0) ? STATE_CMD_DATA : STATE_CMD_FOOTER;
                    break;

                case STATE_CMD_DATA:
                    this->frame_buf_[this->frame_pos_++] = byte;
                    if (++this->cmd_data_idx_ >= this->cmd_data_len_)
                    {
                        this->state_ = STATE_CMD_FOOTER;
                    }
                    break;

                case STATE_CMD_FOOTER:
                    this->frame_buf_[this->frame_pos_++] = byte;
                    // Check for footer: 0x04 0x03 0x02 0x01
                    // Calculate footer position in frame
                    // Frame structure: Header(4) + Length(2) + Command(2) + Data(len) + Footer(4)
                    // We're at position frame_pos_, and we started footer after consuming cmd_data_len_ bytes
                    uint8_t footer_idx = (this->frame_pos_ - 4 - 2 - 2 - this->cmd_data_len_) % 4;
                    if (byte == CMD_FRAME_FOOTER[footer_idx])
                    {
                        if (footer_idx == 3)
                        {
                            // Complete frame - process ACK
                            this->process_cmd_ack();
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Invalid CMD footer byte at idx %d: got 0x%02X, expected 0x%02X",
                                 footer_idx, byte, CMD_FRAME_FOOTER[footer_idx]);
                    }
                    this->state_ = STATE_IDLE;
                    break;

                // ---------------------------------------------------------------------
                // Threshold Frame: F1 F2 F3 F4 [8 data bytes] F5 F6 F7 F8
                // ---------------------------------------------------------------------
                case STATE_THRESH_HEADER:
                    this->frame_buf_[this->frame_pos_++] = byte;
                    // Expect: 0xF1 0xF2 0xF3 0xF4
                    if (byte == THRESHOLD_HEADER[this->header_idx_])
                    {
                        if (++this->header_idx_ == 4)
                        {
                            this->header_idx_ = 0;
                            this->cmd_data_idx_ = 0;
                            this->state_ = STATE_THRESH_DATA;
                        }
                    }
                    else
                    {
                        this->state_ = STATE_IDLE;
                    }
                    break;

                case STATE_THRESH_DATA:
                    this->frame_buf_[this->frame_pos_++] = byte;
                    if (++this->cmd_data_idx_ >= 8)
                    {
                        this->state_ = STATE_THRESH_FOOTER;
                    }
                    break;

                case STATE_THRESH_FOOTER:
                    this->frame_buf_[this->frame_pos_++] = byte;
                    // Check for footer: 0xF5 0xF6 0xF7 0xF8
                    uint8_t ftr_idx = (this->frame_pos_ - 12) % 4;
                    if (byte == THRESHOLD_FOOTER[ftr_idx])
                    {
                        if (ftr_idx == 3)
                        {
                            this->process_threshold_frame();
                        }
                    }
                    this->state_ = STATE_IDLE;
                    break;
            }
        }

        // ========================================================================
        // Dispatch Data Frame to Listeners
        // ========================================================================
        void LD2410S::dispatch_data_frame()
        {
            // State: 0x00=none, 0x01=moving, 0x02=static, 0x03=both
            bool has_target = (this->target_state_ != 0);

            // Select appropriate distance based on target state
            int distance = 0;
            if (this->target_state_ == 1 || this->target_state_ == 3)
            {
                // Moving target detected
                distance = this->moving_dist_;
            }
            else if (this->target_state_ == 2)
            {
                // Static target only
                distance = this->static_dist_;
            }

            // Dispatch to all listeners
            for (auto* listener : this->listeners)
            {
                listener->on_presence(has_target);
                listener->on_distance(distance);
            }
        }

        // ========================================================================
        // Process Command ACK Frame
        // ========================================================================
        void LD2410S::process_cmd_ack()
        {
            // Extract command (little-endian at offset 6)
            uint16_t command = this->frame_buf_[6] | (this->frame_buf_[7] << 8);

            // Check status (0x00 0x01 = success, 0x00 0x00 = failure)
            bool success = (this->frame_buf_[8] == 0x00 && this->frame_buf_[9] == 0x01);

            if (!success)
            {
                ESP_LOGW(TAG, "Command 0x%04X failed with status 0x%02X 0x%02X",
                         command, this->frame_buf_[8], this->frame_buf_[9]);
                return;
            }

            ESP_LOGI(TAG, "Command 0x%04X success", command);

            // Process based on command type
            switch (command)
            {
                case RESP_CONFIG_MODE:  // 0x01FF
                    ESP_LOGD(TAG, "Config mode ACK");
                    break;

                case RESP_READ_PARAMS:  // 0x0160
                    this->process_config_read_ack(&this->frame_buf_[10]);
                    break;

                case RESP_WRITE_PARAMS: // 0x0161
                    ESP_LOGD(TAG, "Write params ACK");
                    // Re-read config to confirm
                    this->read_config_from_device();
                    break;

                case RESP_GET_FW_VERSION: // 0x0100
                    this->process_read_fw_ack(&this->frame_buf_[10]);
                    break;

                default:
                    ESP_LOGD(TAG, "Unknown ACK command: 0x%04X", command);
                    break;
            }
        }

        // ========================================================================
        // Process Threshold Frame
        // ========================================================================
        void LD2410S::process_threshold_frame()
        {
            uint8_t* data = &this->frame_buf_[4];
            int progress = this->two_byte_to_int(data[3], data[4]);

            for (auto* listener : this->listeners)
            {
                if (progress == 100)
                {
                    listener->on_threshold_progress(0);
                    listener->on_threshold_update(false);
                }
                else
                {
                    listener->on_threshold_progress(progress);
                    listener->on_threshold_update(true);
                }
            }
        }

        // ========================================================================
        // Config Mode Control
        // ========================================================================
        void LD2410S::set_config_mode(bool enabled)
        {
            CmdFrameT cmd;
            cmd.command = CMD_CONFIG_MODE;  // 0x00FF
            cmd.data_length = 2;

            // End command value
            uint16_t end_cmd = enabled ? END_CMD_ENABLE_CONFIG : END_CMD_DISABLE_CONFIG;
            cmd.data[0] = end_cmd & 0xFF;
            cmd.data[1] = (end_cmd >> 8) & 0xFF;

            this->send_command(cmd);
        }

        // ========================================================================
        // Apply Configuration - Write and re-read to confirm
        // ========================================================================
        void LD2410S::apply_config()
        {
            this->status_set_warning("Applying configuration...");

            // Enable config mode
            this->set_config_mode(true);

            // Send write command with pending_config values
            CmdFrameT write_cmd = this->prepare_apply_config_cmd();
            this->send_command(write_cmd);

            // Disable config mode (saves to flash)
            this->set_config_mode(false);

            this->status_clear_warning();
        }

        // ========================================================================
        // Start Auto Threshold Calibration
        // ========================================================================
        void LD2410S::start_auto_threshold_update()
        {
            this->status_set_warning("Starting threshold calibration...");
            this->set_config_mode(true);
            CmdFrameT threshold_update_cmd = this->prepare_threshold_cmd();
            this->send_command(threshold_update_cmd);
            this->set_config_mode(false);
            this->status_clear_warning();
        }

        // ========================================================================
        // Read Config from Device
        // ========================================================================
        void LD2410S::read_config_from_device()
        {
            this->set_config_mode(true);
            CmdFrameT read_cmd = this->prepare_read_config_cmd();
            this->send_command(read_cmd);
            this->set_config_mode(false);
            // publish_config_to_ha() will be called from process_config_read_ack()
        }

        // ========================================================================
        // Publish Config to Home Assistant
        // ========================================================================
        void LD2410S::publish_config_to_ha()
        {
#ifdef USE_NUMBER
            if (this->max_distance_number != nullptr)
            {
                this->max_distance_number->publish_state(this->device_config.max_distance);
            }
            if (this->min_distance_number != nullptr)
            {
                this->min_distance_number->publish_state(this->device_config.min_distance);
            }
            if (this->no_delay_number != nullptr)
            {
                this->no_delay_number->publish_state(this->device_config.no_delay);
            }
            if (this->status_reporting_freq_number != nullptr)
            {
                // Convert from 0.1Hz units to Hz for display
                this->status_reporting_freq_number->publish_state(this->device_config.status_freq / 10.0f);
            }
            if (this->distance_reporting_freq_number != nullptr)
            {
                this->distance_reporting_freq_number->publish_state(this->device_config.distance_freq / 10.0f);
            }
#endif
#ifdef USE_SELECT
            if (this->response_speed_select != nullptr)
            {
                const char* speed = (this->device_config.response_speed == 0) ?
                    RESPONSE_SPEED_NORMAL_STR.c_str() : RESPONSE_SPEED_FAST_STR.c_str();
                this->response_speed_select->publish_state(speed);
            }
#endif
        }

        // ========================================================================
        // Prepare Read Config Command
        // ========================================================================
        CmdFrameT LD2410S::prepare_read_config_cmd()
        {
            CmdFrameT cmd_frame;
            cmd_frame.command = CMD_READ_PARAMS;  // 0x0060
            cmd_frame.data_length = 12;  // 6 parameters, 2 bytes each

            // Add parameter types (little-endian)
            uint16_t params[] = {
                PARAM_MAX_DISTANCE,
                PARAM_MIN_DISTANCE,
                PARAM_NO_DELAY,
                PARAM_STATUS_FREQ,
                PARAM_DISTANCE_FREQ,
                PARAM_RESPONSE_SPEED
            };

            for (int i = 0; i < 6; i++)
            {
                cmd_frame.data[i * 2] = params[i] & 0xFF;
                cmd_frame.data[i * 2 + 1] = (params[i] >> 8) & 0xFF;
            }

            return cmd_frame;
        }

        // ========================================================================
        // Prepare Apply Config Command
        // ========================================================================
        CmdFrameT LD2410S::prepare_apply_config_cmd()
        {
            CmdFrameT cmd_frame;
            cmd_frame.command = CMD_WRITE_PARAMS;  // 0x0061
            cmd_frame.data_length = 24;  // 6 params (type + value, 2 bytes each)

            Config to_save = this->pending_config;
            uint16_t param_types[] = {
                PARAM_MAX_DISTANCE,
                PARAM_MIN_DISTANCE,
                PARAM_NO_DELAY,
                PARAM_STATUS_FREQ,
                PARAM_DISTANCE_FREQ,
                PARAM_RESPONSE_SPEED
            };

            uint16_t param_values[] = {
                to_save.max_distance,
                to_save.min_distance,
                to_save.no_delay,
                to_save.status_freq,
                to_save.distance_freq,
                to_save.response_speed
            };

            for (int i = 0; i < 6; i++)
            {
                // Parameter type (little-endian)
                cmd_frame.data[i * 4] = param_types[i] & 0xFF;
                cmd_frame.data[i * 4 + 1] = (param_types[i] >> 8) & 0xFF;
                // Parameter value (little-endian)
                cmd_frame.data[i * 4 + 2] = param_values[i] & 0xFF;
                cmd_frame.data[i * 4 + 3] = (param_values[i] >> 8) & 0xFF;
            }

            return cmd_frame;
        }

        // ========================================================================
        // Prepare Threshold Calibration Command
        // ========================================================================
        CmdFrameT LD2410S::prepare_threshold_cmd()
        {
            CmdFrameT cmd_frame;
            cmd_frame.command = CMD_AUTO_THRESHOLD;  // 0x00A3
            cmd_frame.data_length = 6;

            // Trigger value (little-endian)
            cmd_frame.data[0] = THRESHOLD_TRIGGER_VALUE & 0xFF;
            cmd_frame.data[1] = (THRESHOLD_TRIGGER_VALUE >> 8) & 0xFF;

            // Retention value (little-endian)
            cmd_frame.data[2] = THRESHOLD_RETENTION_VALUE & 0xFF;
            cmd_frame.data[3] = (THRESHOLD_RETENTION_VALUE >> 8) & 0xFF;

            // Time value (little-endian)
            cmd_frame.data[4] = THRESHOLD_TIME_VALUE & 0xFF;
            cmd_frame.data[5] = (THRESHOLD_TIME_VALUE >> 8) & 0xFF;

            return cmd_frame;
        }

        // ========================================================================
        // Prepare Read Firmware Command
        // ========================================================================
        CmdFrameT LD2410S::prepare_read_fw_cmd()
        {
            CmdFrameT cmd_frame;
            cmd_frame.command = CMD_GET_FW_VERSION;  // 0x0000
            cmd_frame.data_length = 0;
            return cmd_frame;
        }

        // ========================================================================
        // Send Command to UART
        // ========================================================================
        void LD2410S::send_command(CmdFrameT frame)
        {
            this->cmd_active = true;

            uint16_t frame_length = 2 + frame.data_length;  // 2 bytes for command

            uint8_t buffer[128];
            size_t pos = 0;

            // Header: 0xFD 0xFC 0xFB 0xFA
            for (int i = 0; i < 4; i++)
            {
                buffer[pos++] = CMD_FRAME_HEADER[i];
            }

            // Length (little-endian)
            buffer[pos++] = frame_length & 0xFF;
            buffer[pos++] = (frame_length >> 8) & 0xFF;

            // Command (little-endian)
            buffer[pos++] = frame.command & 0xFF;
            buffer[pos++] = (frame.command >> 8) & 0xFF;

            // Data
            for (size_t i = 0; i < frame.data_length; i++)
            {
                buffer[pos++] = frame.data[i];
            }

            // Footer: 0x04 0x03 0x02 0x01
            for (int i = 0; i < 4; i++)
            {
                buffer[pos++] = CMD_FRAME_FOOTER[i];
            }

            // Write to UART
            for (size_t i = 0; i < pos; i++)
            {
                this->write_byte(buffer[i]);
            }
            this->flush();

            // Wait for response with timeout
            uint32_t start_millis = millis();
            uint8_t retry = 3;

            while (retry > 0)
            {
                // Check for response
                bool reply_received = false;
                uint32_t timeout_start = millis();

                while ((millis() - timeout_start) < 1000)
                {
                    if (available())
                    {
                        uint8_t byte = read();
                        this->process_byte(byte);
                        // Check if we got an ACK (state machine handles processing)
                        // We need some way to know if we got the right ACK
                        // For now, just wait a bit and assume success
                    }
                    delay_microseconds_safe(1450);
                }

                // Simple timeout/retry logic
                retry--;
            }

            this->cmd_active = false;
        }

        // ========================================================================
        // Legacy Methods (kept for backward compatibility during transition)
        // ========================================================================

        PackageType LD2410S::read_line(uint8_t data, uint8_t* buffer, size_t pos)
        {
            buffer[pos] = data;

            if (pos > 4)
            {
                // Check for command footer (reversed 0x01 0x02 0x03 0x04)
                if (memcmp(&buffer[pos - 3], "\x01\x02\x03\x04", 4) == 0)
                {
                    return PackageType::ACK;
                }
                // Check for data frame (0xF4 ... 0xF8)
                else if (buffer[pos] == 0xF8 && buffer[pos - 6] == 0xF4)
                {
                    return PackageType::SHORT_DATA;
                }
                // Check for threshold footer (reversed 0xF8 0xF7 0xF6 0xF5)
                else if (memcmp(&buffer[pos - 3], "\xF8\xF7\xF6\xF5", 4) == 0)
                {
                    return PackageType::THRESHOLD;
                }
            }
            return PackageType::UNKNOWN;
        }

        bool LD2410S::process_cmd_ack_package(uint8_t* buffer, int len)
        {
            CmdAckT ack = this->parse_ack(buffer, len);
            int command_word = ack.command;
            bool result = ack.result;

            if (!result)
            {
                ESP_LOGW(TAG, "Command 0x%04X failed", command_word);
                return false;
            }

            ESP_LOGI(TAG, "Command 0x%04X success", command_word);

            uint8_t* data = ack.data;

            switch (command_word)
            {
                case RESP_CONFIG_MODE:
                    ESP_LOGD(TAG, "Config mode ACK");
                    break;
                case RESP_READ_PARAMS:
                    this->process_config_read_ack(data);
                    break;
                case RESP_WRITE_PARAMS:
                    ESP_LOGD(TAG, "Write config ACK");
                    break;
                case RESP_GET_FW_VERSION:
                    this->process_read_fw_ack(data);
                    break;
                default:
                    ESP_LOGD(TAG, "Unknown ACK: 0x%04X", command_word);
                    break;
            }

            return true;
        }

        void LD2410S::process_config_read_ack(uint8_t* data)
        {
            // Parse values from device (little-endian)
            uint16_t max_dist = data[0] | (data[1] << 8);    // Param 0x0001
            uint16_t min_dist = data[4] | (data[5] << 8);    // Param 0x0002
            uint16_t delay = data[8] | (data[9] << 8);       // Param 0x0003
            uint16_t status_freq = data[12] | (data[13] << 8);  // Param 0x0004
            uint16_t dist_freq = data[16] | (data[17] << 8);    // Param 0x0005
            uint16_t resp_speed = data[20] | (data[21] << 8);   // Param 0x0006

            // Update device state
            this->device_config.max_distance = max_dist;
            this->device_config.min_distance = min_dist;
            this->device_config.no_delay = delay;
            this->device_config.status_freq = status_freq;
            this->device_config.distance_freq = dist_freq;
            this->device_config.response_speed = resp_speed;

            // Sync pending config to match device
            this->pending_config = this->device_config;

            // Publish to Home Assistant
            this->publish_config_to_ha();

            ESP_LOGD(TAG, "Read config: max=%d, min=%d, delay=%d, status_freq=%d, dist_freq=%d, speed=%d",
                     max_dist, min_dist, delay, status_freq, dist_freq, resp_speed);
        }

        void LD2410S::process_read_fw_ack(uint8_t* data)
        {
            int major_v = static_cast<int>(data[0]);
            int minor_v = static_cast<int>(data[1]);
            int patch_v = static_cast<int>(data[2]);
            std::string version = "v" + std::to_string(major_v) + "." +
                                  std::to_string(minor_v) + "." +
                                  std::to_string(patch_v);

            for (auto* listener : this->listeners)
            {
                listener->on_fw_version(version);
            }

            ESP_LOGD(TAG, "Firmware version: %s", version.c_str());
        }

        void LD2410S::process_short_data_package(uint8_t* data)
        {
            // Data frame format per Manual Page 30:
            // Byte 0: State (0x00=none, 0x01=moving, 0x02=static, 0x03=both)
            // Byte 1-2: Moving distance (little-endian)
            // Byte 3-4: Static distance (little-endian)
            // Byte 5: Unused
            // Byte 6: Footer (0xF8) - already validated by state machine

            uint8_t state = data[0];
            bool has_target = (state != 0);

            int distance = 0;
            if (state == 1 || state == 3)
            {
                // Moving target
                distance = data[1] | (data[2] << 8);
            }
            else if (state == 2)
            {
                // Static target only
                distance = data[3] | (data[4] << 8);
            }

            for (auto* listener : this->listeners)
            {
                listener->on_presence(has_target);
                listener->on_distance(distance);
            }
        }

        void LD2410S::process_threshold_package(uint8_t* data)
        {
            int progress = this->two_byte_to_int(data[3], data[4]);

            for (auto* listener : this->listeners)
            {
                if (progress == 100)
                {
                    listener->on_threshold_progress(0);
                    listener->on_threshold_update(false);
                }
                else
                {
                    listener->on_threshold_progress(progress);
                    listener->on_threshold_update(true);
                }
            }
        }

        void LD2410S::process_data_package(PackageType type, uint8_t* buffer, size_t pos)
        {
            switch (type)
            {
                case PackageType::SHORT_DATA:
                    this->process_short_data_package(&buffer[1]);
                    break;
                case PackageType::THRESHOLD:
                    this->process_threshold_package(&buffer[4]);
                    break;
                default:
                    ESP_LOGD(TAG, "Unexpected package type");
                    break;
            }
        }

        float LD2410S::get_setup_priority() const
        {
            return setup_priority::HARDWARE;
        }

        CmdAckT LD2410S::parse_ack(uint8_t* buffer, size_t length)
        {
            CmdAckT result;
            size_t start = -1;

            // Find header
            for (size_t i = 0; i < length; i++)
            {
                if (memcmp(&buffer[i], "\xFD\xFC\xFB\xFA", 4) == 0)
                {
                    start = i;
                    break;
                }
            }

            if (start == static_cast<size_t>(-1))
            {
                ESP_LOGE(TAG, "Can't find CMD header");
                result.result = false;
                return result;
            }

            // Parse length (little-endian)
            int data_length = buffer[start + 4] | (buffer[start + 5] << 8);
            result.length = data_length;

            // Parse command (little-endian)
            int command_word = buffer[start + 6] | (buffer[start + 7] << 8);
            result.command = command_word;

            // Check status (0x00 0x01 = success)
            bool ack = (buffer[start + 8] == 0x00 && buffer[start + 9] == 0x01);
            result.result = ack;

            // Copy data
            for (size_t idx = 0; idx < result.length && idx < sizeof(result.data); idx++)
            {
                result.data[idx] = buffer[start + 10 + idx];
            }

            return result;
        }

    }
}
