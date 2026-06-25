#include <opencv2/opencv.hpp>
#include <iostream>

#include "include/LaneDetector.hpp"
#include "include/LkaController.hpp"
#include "src/DashboardUI.hpp"

// ============================================================================
// main.cpp
// ----------------------------------------------------------------------------
// Real-Time Vision-Based Lane Keep Assist (LKA) - Live Simulation Dashboard
// ----------------------------------------------------------------------------
// This is the system integration layer: it ties the perception module
// (LaneDetector), the control module (LkaController), and the visualization
// module (DashboardUI) together into a single real-time processing loop -
// read frame -> perceive -> decide -> render -> repeat - exactly like an
// embedded ECU's main software cycle.
// ============================================================================

int main(int argc, char** argv)
{
    // Allow overriding the video path from the command line; default to
    // "data/dashcam.mp4".
    std::string video_path = (argc > 1) ? argv[1] : "data/dashcam1.mp4";

    cv::VideoCapture cap(video_path);
    if (!cap.isOpened())
    {
        std::cerr << "ERROR: Could not open video file: " << video_path << std::endl;
        std::cerr << "Usage: ./lka_demo [path_to_video.mp4]" << std::endl;
        return -1;
    }

    LaneDetector detector;
    LkaController controller(/* kp = */ 15.0);
    DashboardUI dashboard;

    cv::Mat frame;
    int frame_count = 0;

    std::cout << "Starting LKA dashboard demo on: " << video_path << std::endl;
    std::cout << "Press 'q' or ESC in the video window to quit." << std::endl;

    while (true)
    {
        // ----- 1. SENSE: grab the next frame -----
        bool got_frame = cap.read(frame);
        if (!got_frame || frame.empty())
        {
            std::cout << "End of video reached." << std::endl;
            break;
        }
        frame_count++;

        // Work on a copy so we can draw debug lane lines without affecting
        // the LaneDetector's own internal processing.
        cv::Mat debug_frame = frame.clone();

        // ----- 2. PERCEIVE: run the vision pipeline -----
        double lateral_offset_m = detector.computeLateralOffset(frame, &debug_frame);

        // ----- 3. DECIDE: compute the steering correction -----
        SteeringResult steering = controller.computeSteeringAngle(lateral_offset_m);

        // ----- 4. RENDER: build the combined [video | dashboard] frame -----
        cv::Mat output_frame = dashboard.render(debug_frame, lateral_offset_m, steering);

        // ----- 5. DISPLAY -----
        cv::imshow("LKA Live Simulation Dashboard", output_frame);

        // Process at roughly the video's native pace; exit on 'q' or ESC.
        int key = cv::waitKey(30);
        if (key == 'q' || key == 27)  // 27 == ESC
        {
            std::cout << "User requested exit." << std::endl;
            break;
        }
    }

    std::cout << "Processed " << frame_count << " frames. Shutting down." << std::endl;

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
