#include "DashboardUI.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>  // std::clamp

// ============================================================================
// DashboardUI.cpp
// ============================================================================

DashboardUI::DashboardUI() = default;

// ----------------------------------------------------------------------------
// drawRotatedSteeringWheel()
//   Draws a simple steering wheel (a circle with 3 spoke lines) onto its own
//   small canvas, then rotates that whole canvas by `angle_deg` around its
//   center using an OpenCV affine transform.
//
//   Why draw it on its own canvas first, instead of directly on the panel?
//   Rotating a shape in-place is much simpler if the shape's own center is
//   also the canvas's center - cv::getRotationMatrix2D rotates around a
//   given point, so by drawing the wheel centered on a square canvas, we
//   can rotate the ENTIRE canvas and the wheel rotates cleanly around its
//   own middle, with no manual coordinate math for each spoke.
// ----------------------------------------------------------------------------
cv::Mat DashboardUI::drawRotatedSteeringWheel(double angle_deg, int wheel_size) const
{
    cv::Mat wheel_canvas = cv::Mat::zeros(wheel_size, wheel_size, CV_8UC3);

    cv::Point center(wheel_size / 2, wheel_size / 2);
    int radius = static_cast<int>(wheel_size * 0.42);
    int line_thickness = std::max(2, static_cast<int>(wheel_size * 0.027));

    // ---- Outer rim. ----
    cv::circle(wheel_canvas, center, radius, cv::Scalar(255, 255, 255), line_thickness);

    // ---- 3 spokes, 120 degrees apart, all meeting at the center. ----
    // A point on a circle at angle "a" is:
    //   x = center.x + radius * cos(a)
    //   y = center.y + radius * sin(a)
    for (int i = 0; i < 3; ++i)
    {
        double spoke_angle_rad = (i * 120.0) * CV_PI / 180.0;
        cv::Point spoke_end(
            center.x + static_cast<int>(radius * std::cos(spoke_angle_rad)),
            center.y + static_cast<int>(radius * std::sin(spoke_angle_rad)));
        cv::line(wheel_canvas, center, spoke_end, cv::Scalar(255, 255, 255), line_thickness);
    }

    // ---- Small hub circle in the middle, for visual polish. ----
    cv::circle(wheel_canvas, center, std::max(4, wheel_size / 22), cv::Scalar(255, 255, 255), -1);

    // ------------------------------------------------------------------
    // THE ROTATION: this is the heart of the "steering wheel animation".
    //
    // cv::getRotationMatrix2D(center, angle, scale) builds a 2x3 affine
    // transformation matrix that rotates an image by `angle` degrees
    // around `center`. OpenCV's rotation convention is COUNTER-clockwise
    // for positive angles, but in our convention a POSITIVE steering
    // angle means "steer right" (clockwise wheel turn) - so we negate
    // the angle to make the wheel visually turn the intuitive direction.
    //
    // cv::warpAffine then actually applies that matrix to every pixel of
    // the canvas, producing the rotated image.
    // ------------------------------------------------------------------
    double rotation_for_display = -angle_deg;
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, rotation_for_display, 1.0);

    cv::Mat rotated_wheel;
    cv::warpAffine(wheel_canvas, rotated_wheel, rotation_matrix, wheel_canvas.size());

    return rotated_wheel;
}

// ----------------------------------------------------------------------------
// drawTextReadouts()
//   Draws the "ECU diagnostic" text block onto the panel: header, offset,
//   live equation, calculated angle, and saturation status.
//
//   Every Y position is computed as a FRACTION of the panel's own height,
//   and every font scale is computed as a fraction of the panel's own
//   width - with NO upper ceiling, so on a large window the text keeps
//   growing (and stays bold, via scaled thickness) instead of looking
//   tiny and lost in a sea of black.
// ----------------------------------------------------------------------------
void DashboardUI::drawTextReadouts(cv::Mat& panel, double lateral_offset_m,
                                    const SteeringResult& steering) const
{
    const int panel_w = panel.cols;
    const int panel_h = panel.rows;

    // Left margin and font scale both derive from panel WIDTH, so text
    // never runs wider than the panel regardless of resolution. There is
    // no upper ceiling here on purpose - on a large window (e.g. a Mac
    // running the video at full resolution), the text should keep growing
    // so it stays readable instead of looking lost in a sea of black.
    const int left_margin = std::max(12, static_cast<int>(panel_w * 0.07));
    const double base_font_scale = std::max(0.5, panel_w / 420.0);

    // Text thickness must scale up alongside font size, or large text
    // ends up looking thin/faint instead of bold and legible.
    const int thin_thickness = std::max(1, static_cast<int>(std::round(base_font_scale * 1.3)));
    const int bold_thickness = std::max(2, static_cast<int>(std::round(base_font_scale * 2.2)));

    // Small helper: converts a fraction of panel height into a pixel Y,
    // always staying inside the panel.
    auto y_at = [panel_h](double fraction) {
        return std::clamp(static_cast<int>(panel_h * fraction), 0, panel_h - 1);
    };

    // ---- Header ----
    cv::putText(panel, "LKA DIAGNOSTIC DASHBOARD", cv::Point(left_margin, y_at(0.06)),
                cv::FONT_HERSHEY_SIMPLEX, base_font_scale * 0.75,
                cv::Scalar(0, 220, 255), bold_thickness);

    cv::line(panel, cv::Point(left_margin, y_at(0.08)),
              cv::Point(panel_w - left_margin, y_at(0.08)),
              cv::Scalar(80, 80, 80), std::max(1, bold_thickness / 2));

    // ---- Lateral offset readout ----
    std::ostringstream offset_text;
    offset_text << std::fixed << std::setprecision(2)
                 << "Lateral Offset: " << lateral_offset_m << " m";
    cv::putText(panel, offset_text.str(), cv::Point(left_margin, y_at(0.15)),
                cv::FONT_HERSHEY_SIMPLEX, base_font_scale * 0.62,
                cv::Scalar(255, 255, 255), thin_thickness);

    // ---- Live equation ----
    std::ostringstream equation_text;
    equation_text << std::fixed << std::setprecision(2)
                  << "Theta = -(" << lateral_offset_m << "m * "
                  << std::setprecision(0) << kGainForDisplay << ")";
    cv::putText(panel, equation_text.str(), cv::Point(left_margin, y_at(0.21)),
                cv::FONT_HERSHEY_SIMPLEX, base_font_scale * 0.62,
                cv::Scalar(200, 200, 200), thin_thickness);

    // ---- Calculated angle (the headline number) ----
    std::ostringstream angle_text;
    angle_text << std::fixed << std::setprecision(1)
               << "Calculated Angle: " << steering.angle_deg << " deg";
    cv::putText(panel, angle_text.str(), cv::Point(left_margin, y_at(0.29)),
                cv::FONT_HERSHEY_SIMPLEX, base_font_scale * 0.8,
                cv::Scalar(255, 255, 255), bold_thickness);

    // ---- Saturation status - the safety story of the whole project ----
    std::string status_text = steering.is_saturated ? "Status: SATURATED" : "Status: Normal";
    cv::Scalar status_color = steering.is_saturated
                                   ? cv::Scalar(0, 0, 255)    // red  = hit the safety limit
                                   : cv::Scalar(0, 255, 0);   // green = normal operation
    cv::putText(panel, status_text, cv::Point(left_margin, y_at(0.35)),
                cv::FONT_HERSHEY_SIMPLEX, base_font_scale * 0.7,
                status_color, bold_thickness);
}

// ----------------------------------------------------------------------------
// render()
//   Builds the final combined frame: [ original video (65%) | dashboard
//   panel (35%) ]. The panel's own internal layout (text block, steering
//   wheel, caption) is entirely proportional, so nothing ever overlaps or
//   gets clipped, at any input resolution.
// ----------------------------------------------------------------------------
cv::Mat DashboardUI::render(const cv::Mat& original_frame,
                             double lateral_offset_m,
                             const SteeringResult& steering)
{
    const int video_w = original_frame.cols;
    const int panel_h = original_frame.rows;

    // ------------------------------------------------------------------
    // Solve for panel_width so that, in the FINAL combined frame:
    //   video_w  occupies exactly (1 - kPanelShare) of the total width
    //   panel_w  occupies exactly kPanelShare of the total width
    //
    //   panel_w / (video_w + panel_w) = kPanelShare
    //   panel_w = video_w * kPanelShare / (1 - kPanelShare)
    //
    // Example with kPanelShare = 0.35 (35%):
    //   panel_w = video_w * 0.35 / 0.65 = video_w * (7/13)
    // ------------------------------------------------------------------
    const int panel_w = static_cast<int>(
        std::round(video_w * kPanelShare / (1.0 - kPanelShare)));

    cv::Mat panel = cv::Mat::zeros(panel_h, panel_w, original_frame.type());

    // ---- Text block (header through saturation status). ----
    drawTextReadouts(panel, lateral_offset_m, steering);

    // ------------------------------------------------------------------
    // Steering wheel: reserve the remaining vertical space below the
    // text block (which ends around 35% of panel height) and above the
    // caption, then size + center the wheel inside that space so it
    // never collides with the text above it or runs off the bottom.
    // ------------------------------------------------------------------
    const double wheel_zone_top_frac = 0.42;     // just below the status line
    const double wheel_zone_bottom_frac = 0.90;  // leaves room for the caption

    const int wheel_zone_top = static_cast<int>(panel_h * wheel_zone_top_frac);
    const int wheel_zone_bottom = static_cast<int>(panel_h * wheel_zone_bottom_frac);
    const int wheel_zone_height = std::max(0, wheel_zone_bottom - wheel_zone_top);

    // The wheel must fit both the vertical zone AND the panel's width
    // (minus side margins), whichever is smaller.
    const int side_margin = static_cast<int>(panel_w * 0.08);
    const int max_wheel_width = std::max(0, panel_w - 2 * side_margin);

    int wheel_size = std::min(wheel_zone_height, max_wheel_width);
    wheel_size = std::max(wheel_size, 60);  // only a floor, no ceiling - let it fill the zone

    cv::Mat rotated_wheel = drawRotatedSteeringWheel(steering.angle_deg, wheel_size);

    // Center the wheel horizontally in the panel, and vertically within
    // its reserved zone.
    const int wheel_x = std::clamp((panel_w - wheel_size) / 2, 0, std::max(0, panel_w - wheel_size));
    const int wheel_y = std::clamp(
        wheel_zone_top + (wheel_zone_height - wheel_size) / 2,
        0, std::max(0, panel_h - wheel_size));

    if (wheel_size > 0 && wheel_x + wheel_size <= panel_w && wheel_y + wheel_size <= panel_h)
    {
        cv::Rect wheel_roi(wheel_x, wheel_y, wheel_size, wheel_size);
        rotated_wheel.copyTo(panel(wheel_roi));
    }

    // ---- Caption, pinned just below the wheel's reserved zone. ----
    const double caption_font_scale = std::max(0.4, panel_w / 600.0);
    const int caption_thickness = std::max(1, static_cast<int>(std::round(caption_font_scale * 1.3)));
    cv::Size caption_size = cv::getTextSize("Steering Wheel View", cv::FONT_HERSHEY_SIMPLEX,
                                             caption_font_scale, caption_thickness, nullptr);
    const int caption_x = std::clamp((panel_w - caption_size.width) / 2, 0, panel_w - 1);
    const int caption_y = std::min(panel_h - 8, static_cast<int>(panel_h * 0.96));

    cv::putText(panel, "Steering Wheel View", cv::Point(caption_x, caption_y),
                cv::FONT_HERSHEY_SIMPLEX, caption_font_scale,
                cv::Scalar(180, 180, 180), caption_thickness);

    // ---- Combine [ video (65%) | panel (35%) ] side-by-side. ----
    cv::Mat combined;
    cv::hconcat(original_frame, panel, combined);

    return combined;
}