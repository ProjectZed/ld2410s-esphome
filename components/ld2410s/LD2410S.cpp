#include "esphome/core/log.h"
#include "LD2410S.h"
#include "SensorProcessor.cpp"
#include <cstdint>  // for uint8_t
#include <iomanip>  // for std::setw, std::setfill, std::hex
#include <sstream>  // for std::stringstream

namespace esphome
{
    namespace ld2410s
    {
        template <typename T>
        bool areVectorsDifferent(const std::vector<T>& vec1, const std::vector<T>& vec2) {
            // Quick check: if sizes are different, vectors are different
            if (vec1.size() != vec2.size()) {
                return true;
            }
            
            // Compare each element
            for (size_t i = 0; i < vec1.size(); ++i) {
                if (vec1[i] != vec2[i]) {
                    return true;  // Found a difference
                }
            }
            
            // No differences found
            return false;
        }

        static const char *TAG = "ld2410s";
        ReadState currentState = IDLE;
        unsigned long commandSentTime = 0;
        const unsigned long COMMAND_TIMEOUT = 1000; // 1 second timeout
        const std::unordered_map<uint8_t, HeaderDataFooter> headerDataFooterMap = {
            {0x00, {{0xFD, 0xFC, 0xFB, 0xFA}, 6, 72, {0x04, 0x03, 0x02, 0x01}}},  // Command Frames
            {0x01, {{0x6E}, 3, 3, {0x62}}},  // Short Data Frames
            {0x02, {{0xF4, 0xF3, 0xF2, 0xF1}, 5, 72, {0xF8, 0xF7, 0xF6, 0xF5}}},  // Long Data Frames
        };
        SensorProcessor sensor_processor = SensorProcessor(headerDataFooterMap);

        void LD2410S::setup()
        {
            this->enable_configuration_command();
            this->read_fw_version();
            this->read_serial_number();
            this->read_common_parameters();
            this->read_threshold_parameters();
            this->disable_configuration_command();
        }

        void log_frame(const Frame &frame)
        {
            std::stringstream ss;

            ss << "Frame [" << frame.data.size() << " bytes][" << std::to_string(frame.type) << " type]:" << std::endl;

            for (size_t i = 0; i < frame.data.size(); ++i)
            {
                // Format each byte as its hex value with leading zeros
                ss << "0x" << std::hex << std::uppercase << std::setw(2)
                   << std::setfill('0') << static_cast<int>(frame.data[i]);

                // Reset to decimal for other output
                ss << std::dec;

                if (i < frame.data.size() - 1)
                {
                    ss << " ";
                }
            }
            ESP_LOGI(TAG, "%s", ss.str().c_str());
        }

        void log_buffer(const char *prefix, const std::vector<uint8_t> buffer, uint16_t length)
        {
            char log_buffer[256]; // Buffer for formatted log
            char *log_ptr = log_buffer;
            int remaining = sizeof(log_buffer);
            int n;

            // Format header with prefix
            n = snprintf(log_ptr, remaining, "%s [", prefix);
            log_ptr += n;
            remaining -= n;

            // Format each byte in hex
            for (uint16_t i = 0; i < length && remaining > 0; i++)
            {
                n = snprintf(log_ptr, remaining, "%02X", buffer[i]);
                log_ptr += n;
                remaining -= n;

                // Add separator except for last byte
                if (i < length - 1 && remaining > 0)
                {
                    n = snprintf(log_ptr, remaining, " ");
                    log_ptr += n;
                    remaining -= n;
                }
            }

            // Close the log message
            if (remaining > 0)
            {
                snprintf(log_ptr, remaining, "]");
            }

            // Output the log
            ESP_LOGI(TAG, "%s", log_buffer);
            delay(10);
        }

        Frame lastFrame = {std::vector<uint8_t>(), 0};

        void LD2410S::loop()
        {
            while (available())
            {
                uint8_t byte = this->read();
                auto frame = sensor_processor.processByte(byte);
                if (frame && areVectorsDifferent(lastFrame.data, frame->data))
                {
                    log_frame(*frame);
                    lastFrame = *frame;
                }
            }
        }

        void LD2410S::enable_configuration_command()
        {
            CmdFrameT en_conf_cmd = this->build_cmd_frame(START_CONFIG_MODE_CMD, START_CONFIG_MODE_VALUE, sizeof(START_CONFIG_MODE_VALUE) / sizeof(START_CONFIG_MODE_VALUE[0]));
            this->send_command(en_conf_cmd);
        }

        void LD2410S::disable_configuration_command()
        {
            CmdFrameT dis_conf_cmd = this->build_cmd_frame(END_CONFIG_MODE_CMD, nullptr, 0);
            this->send_command(dis_conf_cmd);
        }

        void LD2410S::read_fw_version()
        {
            CmdFrameT read_fw_cmd = this->build_cmd_frame(READ_FW_CMD, nullptr, 0);
            this->send_command(read_fw_cmd);
        }

        void LD2410S::read_serial_number()
        {
            CmdFrameT read_sn_cmd = this->build_cmd_frame(READ_SN_CMD, nullptr, 0);
            this->send_command(read_sn_cmd);
        }

        void LD2410S::read_common_parameters()
        {
            CmdFrameT read_config_cmd = this->build_cmd_frame(READ_PARAMS_CMD, READ_PARAMS_VALUE, sizeof(READ_PARAMS_VALUE) / sizeof(READ_PARAMS_VALUE[0]));
            this->send_command(read_config_cmd);
        }

        void LD2410S::read_threshold_parameters()
        {
            CmdFrameT read_threshold_cmd = this->build_cmd_frame(READ_THRESHOLD_CMD, READ_THRESHOLD_VALUE, sizeof(READ_THRESHOLD_VALUE) / sizeof(READ_THRESHOLD_VALUE[0]));
            this->send_command(read_threshold_cmd);
        }

        CmdFrameT LD2410S::build_cmd_frame(uint16_t command, const uint8_t *data, size_t data_length)
        {
            CmdFrameT cmd_frame = {
                .header = CMD_FRAME_HEADER,
                .data_length = static_cast<uint16_t>(data_length + sizeof(command)),
                .command = command,
                .footer = CMD_FRAME_FOOTER,
            };
            for (uint16_t i = 0; i < data_length; i++)
            {
                cmd_frame.data[i] = data[i];
            }
            return cmd_frame;
        }

        uint16_t frame_to_buffer(const CmdFrameT &frame, uint8_t *cmd_buffer, uint16_t buffer_size)
        {
            uint16_t pos = 0;
            uint16_t total_required_size = sizeof(frame.header) + sizeof(frame.data_length) +
                                           frame.data_length + sizeof(frame.footer);

            // Check if buffer is large enough
            if (buffer_size < total_required_size)
            {
                return 0; // Buffer too small
            }

            // HEADER - direct assignment
            uint32_t *header_ptr = reinterpret_cast<uint32_t *>(&cmd_buffer[pos]);
            *header_ptr = frame.header;
            pos += sizeof(frame.header);

            // SIZE - direct assignment
            uint16_t *size_ptr = reinterpret_cast<uint16_t *>(&cmd_buffer[pos]);
            *size_ptr = frame.data_length;
            pos += sizeof(frame.data_length);

            // COMMAND - direct assignment
            uint16_t *cmd_ptr = reinterpret_cast<uint16_t *>(&cmd_buffer[pos]);
            *cmd_ptr = frame.command;
            pos += sizeof(frame.command);

            // DATA - direct assignment in loop
            for (uint16_t i = 0; i < frame.data_length - sizeof(frame.command); i++)
            {
                cmd_buffer[pos++] = frame.data[i];
            }

            // FOOTER - direct assignment
            uint32_t *footer_ptr = reinterpret_cast<uint32_t *>(&cmd_buffer[pos]);
            *footer_ptr = frame.footer;
            pos += sizeof(frame.footer);

            return pos; // Return the actual buffer length
        }

        void log_command_frame(const CmdFrameT &frame)
        {
            char buffer[256];
            char line_2[128];
            char line_3[128];
            char line_4[128];
            char line_5[256];
            char line_6[128];
            char line_7[128];

            sprintf(line_2, "  Header: 0x%08X", frame.header);
            sprintf(line_3, "  Data Length: %u bytes", frame.data_length);
            sprintf(line_4, "  Command: 0x%04X", frame.command);

            if (frame.data_length > 0)
            {
                char data_log[256] = "  Data: ";
                char *data_ptr = data_log + strlen(data_log);
                int remaining = sizeof(data_log) - strlen(data_log);

                for (uint16_t i = 0; i < frame.data_length - sizeof(frame.command) && remaining > 0; i++)
                {
                    int n = snprintf(data_ptr, remaining, "%02X ", frame.data[i]);
                    data_ptr += n;
                    remaining -= n;
                }
                strcpy(line_5, data_log);
            }
            else
            {
                strcpy(line_5, "");
            }

            sprintf(line_6, "  Footer: 0x%08X", frame.footer);
            sprintf(line_7, "  Total Length: %u bytes", frame.length);

            snprintf(buffer, sizeof(buffer), "Command Frame\n%s\n%s\n%s\n%s\n%s\n%s",
                     line_2, line_3, line_4, line_5, line_6, line_7);

            ESP_LOGI(TAG, "%s", buffer);
            delay(10);
        }

        void log_command_ack(const CmdAckT &ack)
        {
            char buffer[512];
            char line_data[128] = "";

            if (ack.data_length > 0)
            {
                strcpy(line_data, "  Data: ");
                char *data_ptr = line_data + strlen(line_data);
                int remaining = sizeof(line_data) - strlen(line_data);

                for (uint16_t i = 0; i < ack.data_length - sizeof(ack.command) && remaining > 0; i++)
                {
                    int n = snprintf(data_ptr, remaining, "%02X ", ack.data[i]);
                    data_ptr += n;
                    remaining -= n;
                }
            }

            snprintf(buffer, sizeof(buffer),
                     "Command Ack\n"
                     "  Header: 0x%08X\n"
                     "  Data Length: %u bytes\n"
                     "  Command: 0x%04X\n"
                     "%s%s"
                     "  Footer: 0x%08X\n"
                     "  Total Length: %u bytes",
                     ack.header, ack.data_length, ack.command,
                     ack.data_length > 0 ? line_data : "", ack.data_length > 0 ? "\n" : "",
                     ack.footer, ack.length);

            ESP_LOGI(TAG, "%s", buffer);
            delay(10);
        }

        void LD2410S::apply_config()
        {
            this->status_set_warning("Sending command to sensor");
            this->enable_configuration_command();
            CmdFrameT apply_config_cmd = this->prepare_apply_config_cmd();
            this->send_command(apply_config_cmd);
            this->disable_configuration_command();
            this->status_clear_warning();
        }

        void LD2410S::start_auto_threshold_update()
        {
            this->status_set_warning("Sending command to sensor");
            this->enable_configuration_command();
            CmdFrameT threshold_update_cmd = this->prepare_threshold_cmd();
            this->send_command(threshold_update_cmd);
            this->disable_configuration_command();
            this->status_clear_warning();
        }

        CmdFrameT LD2410S::prepare_read_config_cmd()
        {
            CmdFrameT cmd_frame;
            cmd_frame.header = CMD_FRAME_HEADER;
            cmd_frame.command = READ_PARAMS_CMD;
            cmd_frame.data_length = 0;

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_MAX_DETECTION_VALUE, sizeof(CFG_MAX_DETECTION_VALUE));
            cmd_frame.data_length += sizeof(CFG_MAX_DETECTION_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_MIN_DETECTION_VALUE, sizeof(CFG_MIN_DETECTION_VALUE));
            cmd_frame.data_length += sizeof(CFG_MIN_DETECTION_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_NO_DELAY_VALUE, sizeof(CFG_NO_DELAY_VALUE));
            cmd_frame.data_length += sizeof(CFG_NO_DELAY_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_STATUS_FREQ_VALUE, sizeof(CFG_STATUS_FREQ_VALUE));
            cmd_frame.data_length += sizeof(CFG_STATUS_FREQ_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_DISTANCE_FREQ_VALUE, sizeof(CFG_DISTANCE_FREQ_VALUE));
            cmd_frame.data_length += sizeof(CFG_DISTANCE_FREQ_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_RESPONSE_SPEED_VALUE, sizeof(CFG_RESPONSE_SPEED_VALUE));
            cmd_frame.data_length += sizeof(CFG_RESPONSE_SPEED_VALUE);

            cmd_frame.footer = CMD_FRAME_FOOTER;
            return cmd_frame;
        }

        CmdFrameT LD2410S::prepare_apply_config_cmd()
        {
            CmdFrameT cmd_frame;
            cmd_frame.header = CMD_FRAME_HEADER;
            cmd_frame.command = WRITE_PARAMS_CMD;
            cmd_frame.data_length = 0;

            Config to_save = this->new_config;

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_MAX_DETECTION_VALUE, sizeof(CFG_MAX_DETECTION_VALUE));
            cmd_frame.data_length += sizeof(CFG_MAX_DETECTION_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.max_dist, sizeof(to_save.max_dist));
            cmd_frame.data_length += sizeof(to_save.max_dist);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_MIN_DETECTION_VALUE, sizeof(CFG_MIN_DETECTION_VALUE));
            cmd_frame.data_length += sizeof(CFG_MIN_DETECTION_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.min_dist, sizeof(to_save.min_dist));
            cmd_frame.data_length += sizeof(to_save.min_dist);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_NO_DELAY_VALUE, sizeof(CFG_NO_DELAY_VALUE));
            cmd_frame.data_length += sizeof(CFG_NO_DELAY_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.delay, sizeof(to_save.delay));
            cmd_frame.data_length += sizeof(to_save.delay);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_STATUS_FREQ_VALUE, sizeof(CFG_STATUS_FREQ_VALUE));
            cmd_frame.data_length += sizeof(CFG_STATUS_FREQ_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.status_freq, sizeof(to_save.status_freq));
            cmd_frame.data_length += sizeof(to_save.status_freq);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_DISTANCE_FREQ_VALUE, sizeof(CFG_DISTANCE_FREQ_VALUE));
            cmd_frame.data_length += sizeof(CFG_DISTANCE_FREQ_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.dist_freq, sizeof(to_save.dist_freq));
            cmd_frame.data_length += sizeof(to_save.dist_freq);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_RESPONSE_SPEED_VALUE, sizeof(CFG_RESPONSE_SPEED_VALUE));
            cmd_frame.data_length += sizeof(CFG_RESPONSE_SPEED_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.resp_speed, sizeof(to_save.resp_speed));
            cmd_frame.data_length += sizeof(to_save.resp_speed);

            cmd_frame.footer = CMD_FRAME_FOOTER;
            return cmd_frame;
        }

        CmdFrameT LD2410S::prepare_threshold_cmd()
        {
            CmdFrameT cmd_frame;
            cmd_frame.header = CMD_FRAME_HEADER;
            cmd_frame.command = AUTO_UPDATE_THRESHOLD_CMD;
            cmd_frame.data_length = 0;

            memcpy(&cmd_frame.data[cmd_frame.data_length], &THRESHOLD_TRIGGER_VALUE, sizeof(THRESHOLD_TRIGGER_VALUE));
            cmd_frame.data_length += sizeof(THRESHOLD_TRIGGER_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &THRESHOLD_RETENTION_VALUE, sizeof(THRESHOLD_RETENTION_VALUE));
            cmd_frame.data_length += sizeof(THRESHOLD_RETENTION_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &THRESHOLD_TIME_VALUE, sizeof(THRESHOLD_TIME_VALUE));
            cmd_frame.data_length += sizeof(THRESHOLD_TIME_VALUE);

            cmd_frame.footer = CMD_FRAME_FOOTER;
            return cmd_frame;
        }

        bool buffer_to_cmd_ack(const uint8_t *buffer, uint16_t buffer_length, CmdAckT &cmd_ack)
        {
            uint16_t min_size = sizeof(uint32_t) + sizeof(uint16_t) + sizeof(uint32_t);
            uint16_t pos = 0;

            // Extract header using direct pointer casting
            cmd_ack.header = *reinterpret_cast<const uint32_t *>(&buffer[pos]);
            pos += sizeof(cmd_ack.header);

            // Extract data length
            cmd_ack.data_length = *reinterpret_cast<const uint16_t *>(&buffer[pos]);
            pos += sizeof(cmd_ack.data_length);

            // Validate buffer size
            uint16_t expected_total = min_size + cmd_ack.data_length;
            if (buffer_length < expected_total)
            {
                ESP_LOGE(TAG, "Buffer too small: %d", buffer_length);
                return false;
            }

            // Extract command
            cmd_ack.command = *reinterpret_cast<const uint16_t *>(&buffer[pos]);
            pos += sizeof(cmd_ack.command);

            // Extract data
            uint16_t data_to_copy = cmd_ack.data_length - sizeof(cmd_ack.command);
            if (data_to_copy > 0)
            {
                memcpy(cmd_ack.data, &buffer[pos], data_to_copy);
            }
            pos += data_to_copy;

            // Extract footer
            cmd_ack.footer = *reinterpret_cast<const uint32_t *>(&buffer[pos]);

            // Set length
            cmd_ack.length = buffer_length;

            return true;
        }

        void LD2410S::send_command(CmdFrameT frame, bool wait_for_response = true)
        {
            uint32_t start_millis = millis();
            uint8_t cmd_buffer[128];
            frame.length = frame_to_buffer(frame, cmd_buffer, sizeof(cmd_buffer));
            if (frame.length == 0)
            {
                ESP_LOGE(TAG, "Command buffer too small");
                return;
            }

            log_command_frame(frame);
            this->write_array(cmd_buffer, frame.length);
            this->flush();
            if (wait_for_response)
            {
                sensor_processor.reset();
                commandSentTime = millis();
                auto frame;
                while (!frame)
                {
                    if (available())
                    {
                        uint8_t byte = this->read();
                        frame = sensor_processor.processByte(byte);
                        if (frame)
                        {
                            log_frame(*frame);
                            break;
                        }
                    } else {
                        delay(10);
                    }
                    if (millis() - commandSentTime > COMMAND_TIMEOUT)
                    {
                        ESP_LOGE(TAG, "Command timeout");
                        break;
                    }
                }
            }
        }

        PackageType LD2410S::read_line(uint8_t data, uint8_t *buffer, size_t pos)
        {
            buffer[pos] = data;

            if (pos > 4)
            {
                if (memcmp(&buffer[pos - 3], &CMD_FRAME_FOOTER, sizeof(CMD_FRAME_FOOTER)) == 0)
                {
                    return PackageType::ACK;
                }
                else if (buffer[pos] == DATA_FRAME_FOOTER && buffer[pos - 4] == DATA_FRAME_HEADER)
                {
                    return PackageType::SHORT_DATA;
                }
                else if (memcmp(&buffer[pos - 3], &THRESHOLD_FOOTER, sizeof(THRESHOLD_FOOTER)) == 0)
                {
                    return PackageType::TRESHOLD;
                }
            }
            return PackageType::UNKNOWN;
        }

        void LD2410S::process_config_read_ack(uint8_t *data)
        {
            int max_dist = this->read_int(data, 0, 4);
            int min_dist = this->read_int(data, 4, 4);
            int delay = this->read_int(data, 8, 4);
            int status_resp_freq = this->read_int(data, 12, 4);
            int dist_resp_freq = this->read_int(data, 16, 4);
            int resp_speed = this->read_int(data, 20, 4);
#ifdef USE_NUMBER
            this->max_distance_number->publish_state(max_dist);
            this->min_distance_number->publish_state(min_dist);
            this->no_delay_number->publish_state(delay);
            this->status_reporting_freq_number->publish_state(status_resp_freq / 10);
            this->distance_reporting_freq_number->publish_state(dist_resp_freq / 10);
#endif
#ifdef USE_SELECT
            this->response_speed_select->publish_state(resp_speed == 5 ? RESPONSE_SPEED_NORMAL : RESPONSE_SPEED_FAST);
#endif
            memcpy(&this->new_config, &this->current_config, sizeof(this->current_config));
            ESP_LOGD(TAG, "Read config reply: max_dist=%d, min_dist=%d, delay=%d, status_resp_freq=%d, dist_resp_freq=%d, resp_speed=%d", max_dist, min_dist, delay, status_resp_freq, dist_resp_freq, resp_speed);
        }

        void LD2410S::process_read_fw_ack(uint8_t *data)
        {
            ESP_LOGD(TAG, "Read firmware DATA: %x", data);
            int major_v = static_cast<int>(data[0]);
            int minor_v = static_cast<int>(data[1]);
            int patch_v = static_cast<int>(data[2]);
            std::string version = "v" + std::to_string(major_v) + "." + std::to_string(minor_v) + "." + std::to_string(patch_v);
            for (auto &listener : this->listeners)
            {
                listener->on_fw_version(version);
            }
            ESP_LOGD(TAG, "Read firmware reply: %s", version.c_str());
        }

        void LD2410S::process_read_sn_ack(uint8_t *data)
        {
            ESP_LOGD(TAG, "Read serial number DATA: %x", data);
            // std::string sn = std::string(data);
            // for (auto& listener : this->listeners) {
            //     listener->on_sn(sn);
            // }
            // ESP_LOGD(TAG, "Read serial number reply: %s", sn.c_str());
        }

        // bool LD2410S::process_cmd_ack_package(uint8_t *buffer, int len)
        // {
        //     CmdAckT ack = this->parse_ack(buffer, len);
        //     int command_word = ack.command;
        //     bool result = ack.result;
        //     if (!result)
        //     {
        //         ESP_LOGE(TAG, "Command Failed: 0x%04X", command_word);
        //         return false;
        //     }
        //     else
        //     {
        //         ESP_LOGI(TAG, "Command Success: 0x%04X", command_word);
        //     }
        //     uint8_t *data = ack.data;
        //     log_buffer("ACK:", data, sizeof(data));
        //     switch (command_word)
        //     {
        //     case START_CONFIG_MODE_REPLY:
        //         ESP_LOGD(TAG, "Config mode enabled");
        //         break;
        //     case END_CONFIG_MODE_REPLY:
        //         ESP_LOGD(TAG, "Config mode disabled");
        //         break;
        //     case READ_PARAMS_REPLAY:
        //         this->process_config_read_ack(data);
        //         break;
        //     case WRITE_PARAMS_REPLAY:
        //         ESP_LOGD(TAG, "Write config reply processed");
        //         break;
        //     case READ_FW_REPLY:
        //         this->process_read_fw_ack(data);
        //         break;
        //     case READ_SN_REPLY:
        //         this->process_read_sn_ack(data);
        //         break;
        //     default:
        //         ESP_LOGD(TAG, "Unknown reply: %x", command_word);
        //         break;
        //     }
        //     return true;
        // }

        void LD2410S::process_short_data_package(uint8_t *data)
        {
            const bool presenceState = data[0] > 1;
            int distance = this->two_byte_to_int(data[1], data[2]);
            for (auto &listener : this->listeners)
            {
                listener->on_presence(presenceState);
                listener->on_distance(distance);
            }
        }

        void LD2410S::process_threshold_package(uint8_t *data)
        {
            int progress = this->two_byte_to_int(data[3], data[4]);
            for (auto &listener : this->listeners)
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

        void LD2410S::process_data_package(PackageType type, uint8_t *buffer, size_t pos)
        {
            switch (type)
            {
            case PackageType::SHORT_DATA:
                this->process_short_data_package(&buffer[1]);
                break;
            case PackageType::TRESHOLD:
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

        // CmdAckT LD2410S::parse_ack(uint8_t *buffer, size_t length)
        // {
        //     CmdAckT result;
        //     size_t start = -1;
        //     for (size_t i = 0; i < length; i++)
        //     {
        //         if (memcmp(&buffer[i], &CMD_FRAME_HEADER, sizeof(CMD_FRAME_HEADER)) == 0)
        //         {
        //             start = i;
        //             break;
        //         }
        //     }
        //     if (start == -1)
        //     {
        //         ESP_LOGE(TAG, "Can't find cmd header");
        //         result.result = false;
        //         return result;
        //     }
        //     int data_length = this->two_byte_to_int(buffer[start + 4], buffer[start + 5]);
        //     result.length = data_length;
        //     int command_word = this->two_byte_to_int(buffer[start + 6], buffer[start + 7]);
        //     result.command = command_word;
        //     bool ack = buffer[start + 8] == 0x00 && buffer[start + 9] == 0x00;
        //     result.result = ack;
        //     // memcpy(&result.data, &buffer[start + 10], sizeof(uint8_t) * result.length);
        //     for (size_t idx = 0; idx < result.length; idx++)
        //     {
        //         memcpy(&result.data[idx], &buffer[idx + 10], sizeof(buffer[idx + 10]));
        //     }
        //     return result;
        // }
    }
}