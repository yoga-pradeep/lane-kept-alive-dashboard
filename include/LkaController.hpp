#pragma once

// ============================================================================
// LkaController.hpp
// ----------------------------------------------------------------------------
// A minimal Proportional (P) Controller for Lane Keep Assist.
//
// Control law:
//      SteeringAngle = -(lateral_offset_m * KP)
//
//   - lateral_offset_m : how far the car is from lane center (meters)
//   - KP                : tuning gain, fixed at 15.0 for this project
//   - The minus sign    : if the car has drifted RIGHT (positive offset),
//                          we steer LEFT (negative angle) to correct, and
//                          vice-versa. This negative feedback is what keeps
//                          the system stable.
//
// Kept deliberately simple (no integral/derivative terms) so every line is
// easy to explain in an interview - the core idea (measure error, apply a
// gain, saturate the output) is the same idea used in real PID controllers.
// ============================================================================

// Small POD struct returned by the controller. Bundling the angle and the
// saturation flag together makes it trivial for the dashboard UI to show
// "Normal" vs "SATURATED" without recomputing anything.
struct SteeringResult
{
    double angle_deg = 0.0;     // final, safety-clamped steering angle
    bool is_saturated = false;  // true if the raw calculation exceeded the limit
};

class LkaController
{
public:
    // kp: Proportional gain. Fixed at 15.0 per the project spec.
    explicit LkaController(double kp = 15.0);

    // Computes the steering correction for a given lateral offset (meters).
    SteeringResult computeSteeringAngle(double lateral_offset_m) const;

private:
    double kp_;

    // Hard saturation limit, in degrees. The output is never allowed to
    // exceed this range, no matter what the raw math produces.
    static constexpr double kMaxSteeringDeg = 30.0;
    static constexpr double kMinSteeringDeg = -30.0;
};
