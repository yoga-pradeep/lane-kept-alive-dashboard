#include "LaneDetector.hpp"
#include <cmath>

// ============================================================================
// LaneDetector.cpp
// ============================================================================

LaneDetector::LaneDetector() = default;

// ----------------------------------------------------------------------------
// preprocess()
//   Converts the raw color frame into a binary "edge map" highlighting
//   likely lane-marking boundaries.
// ----------------------------------------------------------------------------
cv::Mat LaneDetector::preprocess(const cv::Mat& frame) const
{
    // STEP 1: Grayscale.
    // Lane markings stand out by brightness contrast (white/yellow paint vs
    // dark asphalt), not by color hue. Grayscale is 3x less data and makes
    // every later step simpler.
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // STEP 2: Gaussian Blur.
    // Real camera frames have pixel-level sensor noise. Canny is gradient
    // based, so noise can create many tiny, useless "edges". Blurring first
    // smooths the image so only meaningful edges (like painted lane lines)
    // remain prominent.
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(kBlurKernelSize, kBlurKernelSize), 0);

    // STEP 3: Canny Edge Detection.
    // Canny computes the image gradient (rate of brightness change) at every
    // pixel and marks an edge wherever that gradient is large enough.
    cv::Mat edges;
    cv::Canny(blurred, edges, kCannyLow, kCannyHigh);

    return edges;
}

// ----------------------------------------------------------------------------
// applyRegionOfInterest()
//   Masks out everything except the bottom half of the frame (the road),
//   removing sky, trees, oncoming traffic, etc. that could otherwise create
//   false-positive "edges".
// ----------------------------------------------------------------------------
cv::Mat LaneDetector::applyRegionOfInterest(const cv::Mat& edges) const
{
    cv::Mat mask = cv::Mat::zeros(edges.size(), edges.type());

    int height = edges.rows;
    int width = edges.cols;

    // A trapezoid roughly follows the perspective shape of a road as it
    // recedes toward the horizon, while staying very simple to draw/explain.
    std::vector<cv::Point> roi_points = {
        cv::Point(0, height),
        cv::Point(width, height),
        cv::Point(static_cast<int>(width * 0.55), static_cast<int>(height * 0.5)),
        cv::Point(static_cast<int>(width * 0.45), static_cast<int>(height * 0.5))
    };

    std::vector<std::vector<cv::Point>> roi_poly = {roi_points};
    cv::fillPoly(mask, roi_poly, cv::Scalar(255));

    cv::Mat masked_edges;
    cv::bitwise_and(edges, mask, masked_edges);

    return masked_edges;
}

// ----------------------------------------------------------------------------
// computeLateralOffset()
//   Runs the full pipeline and returns how far (in meters) the vehicle is
//   from the center of the lane.
// ----------------------------------------------------------------------------
double LaneDetector::computeLateralOffset(const cv::Mat& frame, cv::Mat* debug_frame)
{
    // STEP 1-3: grayscale -> blur -> Canny edges.
    cv::Mat edges = preprocess(frame);

    // STEP 4: Crop to the road area only.
    cv::Mat roi_edges = applyRegionOfInterest(edges);

    // STEP 5: Hough Transform - turns edge pixels into straight line
    // segments, each defined by two endpoints (x1, y1) -> (x2, y2).
    std::vector<cv::Vec4i> detected_lines;
    cv::HoughLinesP(roi_edges, detected_lines,
                    kHoughRho, CV_PI / 180.0, kHoughThreshold,
                    kHoughMinLineLength, kHoughMaxLineGap);

    // STEP 6: Classify each line segment as LEFT or RIGHT lane marking,
    // based on its slope. In image coordinates, y increases DOWNWARD, so:
    //   - Negative slope -> line rises left-to-right on screen -> RIGHT lane.
    //   - Positive slope  -> line rises right-to-left on screen -> LEFT lane.
    std::vector<double> left_x_at_bottom;
    std::vector<double> right_x_at_bottom;

    int frame_height = frame.rows;
    int frame_width = frame.cols;

    for (const cv::Vec4i& line : detected_lines)
    {
        double x1 = line[0], y1 = line[1];
        double x2 = line[2], y2 = line[3];

        if (x2 - x1 == 0)
        {
            continue;  // avoid division by zero on vertical segments
        }

        double slope = (y2 - y1) / (x2 - x1);

        // Ignore near-horizontal lines - almost always noise (shadows,
        // road seams), not real lane markings.
        if (std::fabs(slope) < 0.3)
        {
            continue;
        }

        // Extrapolate the line down to the bottom of the frame:
        //   x = x1 + (y_target - y1) / slope
        double x_at_bottom = x1 + (frame_height - y1) / slope;

        if (slope < 0)
        {
            right_x_at_bottom.push_back(x_at_bottom);
        }
        else
        {
            left_x_at_bottom.push_back(x_at_bottom);
        }

        if (debug_frame != nullptr)
        {
            cv::line(*debug_frame, cv::Point(static_cast<int>(x1), static_cast<int>(y1)),
                     cv::Point(static_cast<int>(x2), static_cast<int>(y2)),
                     cv::Scalar(0, 255, 255), 2);
        }
    }

    bool found_left = !left_x_at_bottom.empty();
    bool found_right = !right_x_at_bottom.empty();

    if (!found_left && !found_right)
    {
        // Nothing detected - report zero offset rather than guessing.
        return 0.0;
    }

    auto average = [](const std::vector<double>& values) {
        double sum = 0.0;
        for (double v : values) sum += v;
        return sum / static_cast<double>(values.size());
    };

    double lane_center_px;

    if (found_left && found_right)
    {
        double left_x = average(left_x_at_bottom);
        double right_x = average(right_x_at_bottom);
        lane_center_px = (left_x + right_x) / 2.0;
    }
    else if (found_left)
    {
        // Only left line found - estimate the right line using the known
        // lane width in pixels.
        double left_x = average(left_x_at_bottom);
        lane_center_px = left_x + (kLaneWidthPixels / 2.0);
    }
    else
    {
        double right_x = average(right_x_at_bottom);
        lane_center_px = right_x - (kLaneWidthPixels / 2.0);
    }

    // ------------------------------------------------------------------
    // LATERAL OFFSET MATH
    //
    // The camera (and vehicle center) is assumed mounted at the
    // horizontal center of the frame: camera_center_px = frame_width / 2.
    //
    //   vehicle_offset_px = camera_center_px - lane_center_px
    //     -> positive => vehicle is to the RIGHT of lane center
    //     -> negative => vehicle is to the LEFT of lane center
    //
    // Convert pixels -> meters using our calibration ratio:
    //   offset_m = vehicle_offset_px * kMetersPerPixel
    // ------------------------------------------------------------------
    double camera_center_px = frame_width / 2.0;
    double vehicle_offset_px = camera_center_px - lane_center_px;
    double lateral_offset_m = vehicle_offset_px * kMetersPerPixel;

    if (debug_frame != nullptr)
    {
        cv::circle(*debug_frame, cv::Point(static_cast<int>(lane_center_px), frame_height - 10),
                   6, cv::Scalar(0, 0, 255), -1);
        cv::circle(*debug_frame, cv::Point(static_cast<int>(camera_center_px), frame_height - 10),
                   6, cv::Scalar(255, 0, 0), -1);
    }

    return lateral_offset_m;
}
