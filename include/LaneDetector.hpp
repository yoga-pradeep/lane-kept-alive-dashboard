#pragma once

#include <opencv2/opencv.hpp>

// ============================================================================
// LaneDetector.hpp
// ----------------------------------------------------------------------------
// A classical (non-ML) computer-vision lane detector.
//
// Pipeline (fully explained, step-by-step, in LaneDetector.cpp):
//   1. Grayscale       - reduce 3 color channels to 1 brightness channel.
//   2. Gaussian Blur   - smooth out sensor noise before edge detection.
//   3. Region of Interest (ROI) - only look at the road (bottom half).
//   4. Canny Edge Detection     - find pixels with sharp brightness changes.
//   5. HoughLinesP              - group edge pixels into straight lines.
//   6. Slope-based classification - split lines into LEFT vs RIGHT lane.
//   7. Lane center vs camera center -> lateral_offset_m (in meters).
//
// This mirrors a simplified version of real automotive lane detection
// pipelines and is intentionally kept simple so every line is easy to
// explain in an interview.
// ============================================================================

class LaneDetector
{
public:
    LaneDetector();

    // Processes a single BGR video frame and returns the lateral offset of
    // the vehicle from the lane center, in meters.
    //   - Positive value -> vehicle is to the RIGHT of lane center.
    //   - Negative value -> vehicle is to the LEFT of lane center.
    //
    // debug_frame (optional): if non-null, detected lane lines and the
    // lane/camera center markers are drawn onto it for visualization.
    double computeLateralOffset(const cv::Mat& frame, cv::Mat* debug_frame = nullptr);

private:
    // ---- Tunable pipeline parameters (named constants, easy to find/explain) ----

    static constexpr int kBlurKernelSize = 5;        // must be odd

    static constexpr double kCannyLow = 50.0;
    static constexpr double kCannyHigh = 150.0;

    static constexpr double kHoughRho = 2.0;
    static constexpr int kHoughThreshold = 80;
    static constexpr double kHoughMinLineLength = 100.0;
    static constexpr double kHoughMaxLineGap = 30.0;

    // -------------------------------------------------------------------
    // Pixel-to-meter conversion.
    //
    // A standard highway lane is about 3.7 meters wide. At the bottom of
    // our cropped frame, that distance corresponds to roughly 600 pixels
    // (measured empirically on sample frames). This gives a simple,
    // explainable linear ratio:
    //
    //      meters_per_pixel = 3.7 / 600  ~= 0.00617 m/pixel
    //
    // A production system would calibrate this from real camera
    // intrinsics + a perspective transform - this fixed ratio is a
    // deliberate simplification to keep the math 100% explainable.
    // -------------------------------------------------------------------
    static constexpr double kLaneWidthMeters = 3.7;
    static constexpr double kLaneWidthPixels = 600.0;
    static constexpr double kMetersPerPixel = kLaneWidthMeters / kLaneWidthPixels;

    // Helper: grayscale -> blur -> Canny. Returns the edge map.
    cv::Mat preprocess(const cv::Mat& frame) const;

    // Helper: masks out everything except the bottom-half road area.
    cv::Mat applyRegionOfInterest(const cv::Mat& edges) const;
};
