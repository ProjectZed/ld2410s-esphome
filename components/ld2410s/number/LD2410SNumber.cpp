#include "LD2410SNumber.h"

namespace esphome
{
    namespace ld2410s
    {
        void LD2410SMaxDistanceNumber::control(float max_distance)
        {
            this->publish_state(max_distance);
            this->parent_->pending_config.max_distance = static_cast<uint16_t>(max_distance);
        }

        void LD2410SMinDistanceNumber::control(float min_distance)
        {
            this->publish_state(min_distance);
            this->parent_->pending_config.min_distance = static_cast<uint16_t>(min_distance);
        }

        void LD2410SDelayNumber::control(float no_delay)
        {
            this->publish_state(no_delay);
            this->parent_->pending_config.no_delay = static_cast<uint16_t>(no_delay);
        }

        void LD2410SStatusReportingFreqNumber::control(float status_reporting_freq)
        {
            this->publish_state(status_reporting_freq);
            // Convert Hz to 0.1Hz units for device
            this->parent_->pending_config.status_freq = static_cast<uint16_t>(status_reporting_freq * 10);
        }

        void LD2410SDistReportingFreqNumber::control(float distance_reporting_freq)
        {
            this->publish_state(distance_reporting_freq);
            // Convert Hz to 0.1Hz units for device
            this->parent_->pending_config.distance_freq = static_cast<uint16_t>(distance_reporting_freq * 10);
        }
    }
}
