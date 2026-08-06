#include "dr4b.hpp"
#include "lemlib/util.hpp"
#include <cmath>

void Dr4b::update() {
    const float cur_pos = position_callback_();
    printf("lift pos: %f\n", cur_pos);
    printf("error: %f %f\n", target_ - cur_pos, target_);
    move_callback_(pid_.update(target_ - cur_pos) + feed_forward_ * std::sin(lemlib::degToRad(cur_pos)));
}