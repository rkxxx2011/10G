#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "claw.hpp"
#include "dr4b.hpp"

// Degrees the lift travels for one press of the up/down buttons — matches the
// physical height of one pin-and-cup stack. Calibrate to hardware.
inline constexpr float LIFT_STEP_HEIGHT = 17.5;
inline constexpr size_t LIFT_MAX_LEVEL = 10;

// Claw rotation angle for the lifted (carry) pose. Positive, matching the
// sign convent.ion of claw_angle's LOADING/SCORING/GOONER in claw.hpp — the
// claw's real ~120 degrees of travel is on the positive side, so this stays
// inside that range (and below claw_angle::GOONER = 120) instead of jamming.
// The scoring pose reuses claw_angle::SCORING, and the loading pose reuses
// claw_angle::LOADING.
inline constexpr float CLAW_ANGLE_CARRY = 110;

// ==================== SCORING SEQUENCE (Y button) ====================
// 1. Lift is raised to at least SCORE_MIN_LIFT_TARGET_DEG (never lowered
// below it, but left alone if it's already higher). 2. Claw rotates to at
// least SCORE_MIN_CLAW_ANGLE_DEG (0 = face down, so this is a floor on how
// far down it's allowed to rotate). 3. Outtake for SCORE_OUTTAKE_MS.
// 4. Claw rotates all the way up to its max travel.
// SCORE_LIFT_DROP_DEG is currently unused (lift no longer drops before
// scoring) — left here in case that behavior is wanted again later.
inline constexpr float SCORE_LIFT_DROP_DEG = 5.0f;
// Minimum lift target enforced the instant scoring starts. Fixes the arm
// ending up too low to actually score — whether from gravity sag or the
// lift simply being at a low level when Y is pressed. This is in the same
// raw sensor-degree units the lift's set_target()/get_target() already use
// (not a calibrated physical measurement) — tune on the real robot.
inline constexpr float SCORE_MIN_LIFT_TARGET_DEG = 80.0f;
// Minimum claw rotation angle used when scoring (0 = face down, matching
// claw_angle::LOADING). Fixes the claw rotating too far down to actually
// score — whatever angle the scoring flow would otherwise use gets raised
// to at least this floor.
inline constexpr float SCORE_MIN_CLAW_ANGLE_DEG = 80.0f;
inline constexpr std::uint32_t SCORE_OUTTAKE_MS = 350;
// Safety cap per phase in case a motion never settles within its stopping
// threshold (e.g. a stall) — keeps the sequence from hanging forever.
inline constexpr std::uint32_t SCORE_PHASE_TIMEOUT_MS = 800;

enum class ScoreSequencePhase {
    IDLE,
    LOWERING,
    ROTATING_TO_SCORE,
    OUTTAKING,
    ROTATING_TO_MAX
};

class Orchestrator {
private:
    std::shared_ptr<Dr4b> dr4b_ {nullptr};
    std::shared_ptr<Claw> claw_ {nullptr};
    std::shared_ptr<pros::Controller> controller_ {nullptr};

    ScoreSequencePhase score_phase_ {ScoreSequencePhase::IDLE};
    std::uint32_t score_phase_start_ms_ = 0;

    // Guards against a single physical R1 tap somehow registering as
    // multiple rapid presses (e.g. a controller comms hiccup or loop stall
    // while driving) and stacking up several stage changes almost instantly.
    std::uint32_t last_lift_button_ms_ = 0;

    // Keeps intake running briefly after the button is released so the intake
    // doesn't stop instantly on the last frame of input.
    bool intake_release_pending_ = false;
    std::uint32_t intake_release_started_ms_ = 0;

private:
    bool is_lift_at_lowest_level() const;
    void set_claw_state(ClawState state);
    float get_default_claw_angle() const;
    void move_claw_to_angle(float angle);
    void buzz_controller();
    void start_score_sequence();
    void update_score_sequence();

public:
    Orchestrator() = default;
    Orchestrator(
        const std::shared_ptr<Dr4b>& dr4b,
        const std::shared_ptr<Claw>& claw,
        const std::shared_ptr<pros::Controller>& controller
    );

public:
    // Controller binds
    void handle_lift_up();   // R1 — steps up one stage relative to wherever
                              // the lift's target actually is right now (not
                              // a separately-tracked counter), so it can't
                              // desync from R2/UP manual jogging.
    void handle_intake();    // L1
    void handle_outtake();   // L2
    // R2/UP are raw hold-to-jog, handled directly in opcontrol.cpp.
    // Y starts the scoring sequence (see start_score_sequence/update_score_sequence)

    ScoreSequencePhase get_score_phase() const {return score_phase_;}

    // Polls the controller and drives the lift/claw accordingly. Call once per opcontrol tick.
    void update();
};
