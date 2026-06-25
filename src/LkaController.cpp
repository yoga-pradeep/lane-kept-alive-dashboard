#include "LkaController.hpp"
#include <algorithm>  // std clamp (manual fallback below)

// ============================================================================
// LkaController.cpp
// ============================================================================

LkaController::LkaController(double kp)
    : kp_(kp)
{
}

SteeringResult LkaController::computeSteeringAngle(double lateral_offset_m) const
{
    SteeringResult result;

    // ------------------------------------------------------------------
    // CORE CONTROL LAW: Proportional controller.
    //
    //   raw_angle_deg = -(lateral_offset_m * kp_)
    //
    // Example: offset = +0.5 m (drifted right), kp_ = 15
    //          raw_angle = -(0.5 * 15) = -7.5 degrees (steer left)
    // ------------------------------------------------------------------
    double raw_angle_deg = -(lateral_offset_m * kp_);

    // ------------------------------------------------------------------
    // SAFETY CHECK: Saturation limit.
    //
    // Even a "correct" calculation might ask for an unrealistic steering
    // angle. We hard-clamp the output to +/- 30 degrees, and remember
    // whether we actually had to clamp it - the dashboard uses this to
    // show "Normal" vs "SATURATED".
    // ------------------------------------------------------------------
    // Manual clamp: max(min_val, min(val, max_val))
    double clamped_angle_deg = (raw_angle_deg < kMinSteeringDeg) ? kMinSteeringDeg :
                               (raw_angle_deg > kMaxSteeringDeg) ? kMaxSteeringDeg :
                               raw_angle_deg;

    result.angle_deg = clamped_angle_deg;
    result.is_saturated = (clamped_angle_deg != raw_angle_deg);

    return result;
}
