#include "claw.hpp"
#include "lemlib/util.hpp"

namespace {
// Reject a disconnected/glitched sensor reading before it can cause a sudden
// jerk. Normal claw rotation angles should be far inside this range.
constexpr float MAX_REASONABLE_ANGLE_DEG = 1000.0f;

// Low-pass filter strength: how much a single new reading can move the
// filtered value. Lower = smoother but slower to react to real motion.
constexpr float FILTER_ALPHA = 0.25f;

} // namespace

void Claw::update() {
    const float position_deg = position_callback_();

    if (std::isfinite(position_deg) && std::abs(position_deg) <= MAX_REASONABLE_ANGLE_DEG) {
        if (!filter_initialized_) {
            filtered_position_ = position_deg;
            filter_initialized_ = true;
        } else {
            filtered_position_ += FILTER_ALPHA * (position_deg - filtered_position_);
        }

        rotate_callback_(std::clamp<float>(
            angular_pid_.update(target_angle_ - filtered_position_)
                + feed_forward_ * std::sin(lemlib::degToRad(filtered_position_)),
                -127, 127
            )
        );
    } else {
        rotate_callback_(0.0f);
    }

    flex_wheel_callback_(static_cast<int32_t>(state_));
}