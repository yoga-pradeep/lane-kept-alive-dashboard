#pragma once

#include <opencv2/opencv.hpp>
#include "../include/LkaController.hpp"

// ============================================================================
// DashboardUI.hpp
// ----------------------------------------------------------------------------
// The visual "ECU diagnostic tool" simulation layer.
//
// This class doesn't calculate anything - it only DRAWS. It takes the
// numbers already computed by LaneDetector and LkaController and renders
// them as a live dashboard panel attached to the right side of the video,
// including a steering wheel graphic that physically rotates using OpenCV
// affine transforms (cv::getRotationMatrix2D + cv::warpAffine).
//
// Keeping rendering completely separate from calculation is good practice:
// it means LaneDetector and LkaController stay simple, pure-logic, and easy
// to unit test, while all the "make it look nice" code lives in one place.
//
// LAYOUT MODEL
// ------------
// The final displayed frame is split exactly:
//   - 65% of the total width  -> the original video
//   - 35% of the total width  -> this dashboard panel
//
// Every element drawn inside the panel (header, text readouts, steering
// wheel, caption) is positioned as a PROPORTION of the panel's own width
// and height, not as a hardcoded pixel offset. That's what guarantees the
// whole dashboard fits cleanly on screen and never overlaps or clips,
// regardless of the input video's resolution.
// ============================================================================

class DashboardUI
{
public:
    DashboardUI();

    // Takes the original camera frame plus the latest measurements, and
    // returns a NEW, wider frame: [ original video (65%) | dashboard panel (35%) ].
    cv::Mat render(const cv::Mat& original_frame,
                    double lateral_offset_m,
                    const SteeringResult& steering);

private:
    // ---- Screen-split ratio: video gets 65%, the dashboard panel gets 35%. ----
    // If video_width is the width of the incoming frame (unchanged), then:
    //   panel_width / (video_width + panel_width) = kPanelShare
    // Solving for panel_width gives:
    //   panel_width = video_width * kPanelShare / (1 - kPanelShare)
    static constexpr double kPanelShare = 0.35;  // panel = 35% of total width

    // Steering gain used only for displaying the equation text
    // (Theta = -(offset * kGainForDisplay)) - must match LkaController's kp.
    static constexpr double kGainForDisplay = 15.0;

    // Draws a simple steering wheel (circle + 3 spokes) onto a small,
    // square canvas, then rotates that canvas by angle_deg using an affine
    // transform, and returns the rotated wheel image.
    cv::Mat drawRotatedSteeringWheel(double angle_deg, int wheel_size) const;

    // Draws the full text block (header, offset, equation, angle, status)
    // onto the panel, with every line positioned as a fraction of the
    // panel's own dimensions so it always fits.
    void drawTextReadouts(cv::Mat& panel, double lateral_offset_m,
                           const SteeringResult& steering) const;
};