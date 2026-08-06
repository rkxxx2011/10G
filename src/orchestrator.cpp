#include "orchestrator.hpp"
#include <cstddef>

Orchestrator::Orchestrator(
    const std::shared_ptr<Dr4b>& dr4b,
    const std::shared_ptr<Claw>& claw,
    const std::shared_ptr<pros::Controller>& controller
) : dr4b_ {dr4b}, claw_ {claw}, controller_ {controller} {}

bool Orchestrator::is_lift_at_lowest_level() const {
    // Checks the lift's actual target position, so this stays correct
    // regardless of whether R1's stage stepping or the R2/UP manual jogs
    // (opcontrol.cpp) most recently moved the lift.
    return dr4b_ && dr4b_->get_target() <= 0.5f;
}

namespace {
// The very first press up from rest is a tiny bit smaller than a normal step
// so it feels a little more deliberate, while later presses stay at the
// standard step size.
constexpr float FIRST_STEP_FRACTION = 0.95f;
constexpr float FIRST_STAGE_TARGET_DEG = LIFT_STEP_HEIGHT * FIRST_STEP_FRACTION;
// Highest target R1 can reach, matching the old "LIFT_MAX_LEVEL levels" cap
// but as an absolute position instead of a press-counter.
constexpr float LIFT_MAX_TARGET_DEG =
    FIRST_STAGE_TARGET_DEG + static_cast<float>(LIFT_MAX_LEVEL - 1) * LIFT_STEP_HEIGHT;
} // namespace

void Orchestrator::set_claw_state(const ClawState state) {
    if (claw_) claw_->set_state(state);
}

float Orchestrator::get_default_claw_angle() const {
    return is_lift_at_lowest_level() ? claw_angle::LOADING : CLAW_ANGLE_CARRY;
}

void Orchestrator::move_claw_to_angle(const float angle) {
    if (claw_) claw_->set_target_angle(angle);
}

void Orchestrator::buzz_controller() {
    if (controller_) controller_->rumble(".");
}

namespace {
// Minimum time between accepted R1 presses. Real driver presses are never
// this close together, so this only ever blocks a spurious double-registered
// press, not legitimate rapid button mashing.
constexpr std::uint32_t LIFT_BUTTON_DEBOUNCE_MS = 60;
} // namespace

void Orchestrator::handle_lift_up() {
    const std::uint32_t now = pros::millis();
    if (now - last_lift_button_ms_ < LIFT_BUTTON_DEBOUNCE_MS) return;
    last_lift_button_ms_ = now;

    if (!dr4b_) return;

    // Step up relative to wherever the target actually is right now (not a
    // separately-tracked counter), so it can never desync from R2/UP jogging.
    const float current_target = dr4b_->get_target();
    const float new_target = (current_target <= 0.5f)
        ? FIRST_STAGE_TARGET_DEG
        : std::min(current_target + LIFT_STEP_HEIGHT, LIFT_MAX_TARGET_DEG);

    dr4b_->set_target(new_target);
    buzz_controller();
}

void Orchestrator::handle_intake() {
    set_claw_state(ClawState::INTAKING);
    move_claw_to_angle(get_default_claw_angle());
}

void Orchestrator::handle_outtake() {
    // Only spin the wheels out — leave the rotation wherever it already is
    // instead of also driving it to the scoring angle.
    set_claw_state(ClawState::OUTAKING);
}

void Orchestrator::start_score_sequence() {
    if (score_phase_ != ScoreSequencePhase::IDLE) return; // already running, ignore

    if (dr4b_) {
        const float dropped_target = dr4b_->get_target() - SCORE_LIFT_DROP_DEG;
        dr4b_->set_target(dropped_target < 0.0f ? 0.0f : dropped_target);
    }

    score_phase_ = ScoreSequencePhase::LOWERING;
    score_phase_start_ms_ = pros::millis();
}

void Orchestrator::update_score_sequence() {
    const std::uint32_t elapsed_ms = pros::millis() - score_phase_start_ms_;

    switch (score_phase_) {
        case ScoreSequencePhase::IDLE:
            break;

        case ScoreSequencePhase::LOWERING: {
            const bool settled = dr4b_ && dr4b_->is_within_target_threshold();
            if (settled || elapsed_ms >= SCORE_PHASE_TIMEOUT_MS) {
                move_claw_to_angle(claw_angle::SCORING);
                score_phase_ = ScoreSequencePhase::ROTATING_TO_SCORE;
                score_phase_start_ms_ = pros::millis();
            }
            break;
        }

        case ScoreSequencePhase::ROTATING_TO_SCORE: {
            const bool settled = claw_ && claw_->is_within_stopping_threshold();
            if (settled || elapsed_ms >= SCORE_PHASE_TIMEOUT_MS) {
                set_claw_state(ClawState::OUTAKING);
                score_phase_ = ScoreSequencePhase::OUTTAKING;
                score_phase_start_ms_ = pros::millis();
            }
            break;
        }

        case ScoreSequencePhase::OUTTAKING: {
            if (elapsed_ms >= SCORE_OUTTAKE_MS) {
                set_claw_state(ClawState::STOPPED);
                move_claw_to_angle(claw_angle::GOONER);
                score_phase_ = ScoreSequencePhase::ROTATING_TO_MAX;
                score_phase_start_ms_ = pros::millis();
            }
            break;
        }

        case ScoreSequencePhase::ROTATING_TO_MAX: {
            const bool settled = claw_ && claw_->is_within_stopping_threshold();
            if (settled || elapsed_ms >= SCORE_PHASE_TIMEOUT_MS) {
                score_phase_ = ScoreSequencePhase::IDLE;
            }
            break;
        }
    }
}

void Orchestrator::update() {
    if (controller_) {
        if (controller_->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) start_score_sequence();

        if (score_phase_ != ScoreSequencePhase::IDLE) {
            // The scoring sequence owns the lift/claw for its duration —
            // driving still works, but R1/R2/L1/L2 are ignored until it's done.
            update_score_sequence();
        } else {
            if (controller_->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) handle_lift_up();
            // R2 is a raw hold-to-jog-down (see opcontrol.cpp), not a level
            // step, so it isn't handled here.

            const bool intake_pressed = controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L1);
            const bool outtake_pressed = controller_->get_digital(pros::E_CONTROLLER_DIGITAL_L2);

            if (intake_pressed) {
                handle_intake();
                intake_release_pending_ = false;
            } else if (outtake_pressed) {
                handle_outtake();
                intake_release_pending_ = false;
            } else if (intake_release_pending_) {
                if (pros::millis() - intake_release_started_ms_ >= 100) {
                    set_claw_state(ClawState::STOPPED);
                    intake_release_pending_ = false;
                }
            } else {
                intake_release_pending_ = true;
                intake_release_started_ms_ = pros::millis();

                // Idle: intake keeps spinning through the first stage, and
                // shuts off above it — separate from the rotation angle,
                // which still switches right at target 0. Checks the actual
                // target (not a level counter) so it can't desync from
                // R2/UP jogging either.
                const bool at_or_below_first_stage =
                    dr4b_ && dr4b_->get_target() <= (FIRST_STAGE_TARGET_DEG + 0.5f);
                set_claw_state(at_or_below_first_stage ? ClawState::INTAKING : ClawState::STOPPED);
                move_claw_to_angle(get_default_claw_angle());
            }
        }
    }

    if (dr4b_ != nullptr) dr4b_->update();
    if (claw_ != nullptr) claw_->update();
}
