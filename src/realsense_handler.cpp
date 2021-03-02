#include "realsense_handler.h"
#include <iostream>

RealsenseHandler::RealsenseHandler(bool align_color_to_depth){
    try {
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
        // Configure and start the pipeline
        profile = pipeline.start(cfg);

        // Create an align object  align_to = rs.stream.color
        if (align_color_to_depth){
            const auto align_to = RS2_STREAM_DEPTH;
            align = std::unique_ptr<rs2::align>(new rs2::align(align_to));
        }

        depth_profile = std::unique_ptr<rs2::video_stream_profile>(
            new rs2::video_stream_profile(profile.get_stream(RS2_STREAM_DEPTH)));

        n_frames = 0;
        start_time = time(NULL);
    } catch (const rs2::error & e){
        report_rs2_error_and_exit(e);
    }
}

void RealsenseHandler::report_rs2_error_and_exit(const rs2::error& e){
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    exit(-1);
}

void RealsenseHandler::get_and_process_frame(){
    try {
        // Block program until frames arrive
        rs2::frameset frames = pipeline.wait_for_frames();

        if (align)
            frames = align->process(frames);

        // Try to get a frame of a depth image
        auto depth_frame = frames.get_depth_frame();
        auto color_frame = frames.get_color_frame();

        last_depth_frame = depth_frame;
        have_last_depth_frame = true;

        // Get the depth frame's dimensions
        float width = depth_frame.get_width();
        float height = depth_frame.get_height();

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth_frame.get_distance(width / 2, height / 2);

        n_frames += 1;
    } catch (rs2::error& e){
        report_rs2_error_and_exit(e);
    }
}

float RealsenseHandler::get_current_framerate(){
    double avg_framerate = n_frames / difftime(time(NULL), start_time);
    return avg_framerate;
}
const rs2::points RealsenseHandler::get_current_pointcloud(){
    if (have_last_depth_frame)
        return pointcloud.calculate(last_depth_frame);
    return rs2::points();
}
