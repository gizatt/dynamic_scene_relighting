#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <time.h>
#include <memory>

class RealsenseHandler {
    private:
        rs2::config cfg;
        rs2::pipeline pipeline;
        rs2::pipeline_profile profile;
        std::unique_ptr<rs2::video_stream_profile> depth_profile;
        std::unique_ptr<rs2::align> align;
        rs2::pointcloud pointcloud;

        bool have_last_depth_frame = false;
        rs2::frame last_depth_frame;

        int n_frames;
        time_t start_time;

    public:
        RealsenseHandler(bool align_color_to_depth = false);
        void get_and_process_frame();
        const int get_width() {
            return depth_profile->width();
        };
        const int get_height() {
            return depth_profile->height();
        }
        float get_current_framerate();
        void report_rs2_error_and_exit(const rs2::error& e);
        const rs2::points get_current_pointcloud();
};