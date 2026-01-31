#include "LD2410SResponseSpeedSelect.h"

namespace esphome
{
    namespace ld2410s
    {
        void LD2410SResponseSpeedSelect::control(const std::string& value)
        {
            this->publish_state(value);
            // Response speed: 0x0000 = Normal, 0x0001 = Fast
            this->parent_->pending_config.response_speed =
                (value == RESPONSE_SPEED_NORMAL_STR) ? RESPONSE_SPEED_NORMAL : RESPONSE_SPEED_FAST;
        }
    }
}
