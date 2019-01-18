#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <iostream>

#include "libterraclear/src/stopwatch.hpp"
namespace tc = terraclear;

//OPENCV 
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    // Create a simple OpenCV window
    //Open CV Window stuff
    std::string window_rgb = "rgb";
    cv::namedWindow(window_rgb, cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO);// | WINDOW_AUTOSIZE);

    std::string window_depth = "depth";
    cv::namedWindow(window_depth, cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO);// | WINDOW_AUTOSIZE);

     //setup and apply decimation filter with default options
    rs2::decimation_filter decimation_filter;

    //setup and apply spacial filter with default options
    rs2::spatial_filter spacial_filter;

    //setup and apply temporal filter with default options..
    rs2::temporal_filter temporal_filter;
   
    //setup and apply hole filling filter
    rs2::hole_filling_filter hole_filter;
    hole_filter.set_option(RS2_OPTION_HOLES_FILL, 2.0f); //0 0 = Fill From Left, 1 = Nearest, 2 = Furthest.
  
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    color_map.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 1.0f);
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.0f); // 0 = Jet, 1=  classic, 2 = White to Black        
    
    //align filter
    rs2::align align(RS2_STREAM_COLOR);
            
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline   _pipe;
    rs2::config     _pipe_config;
            
    //enable RGB and Depth streams at 30FPS
    _pipe_config.enable_stream(RS2_STREAM_COLOR, 1920,1080,RS2_FORMAT_BGR8, 30);
    _pipe_config.enable_stream(RS2_STREAM_DEPTH, 1280,720,RS2_FORMAT_Z16, 30);
    _pipe.start(_pipe_config);

    cv::Mat cv_depth(1080, 1920, CV_8UC3);
    cv::Mat cv_color(720, 1280, CV_8UC3);

    tc::stopwatch sw;
    sw.start();
    
    while (true) // Application still alive?
    {
        int fps_color = 0;
        int fps_depth = 0;
        
        rs2::frameset data = _pipe.wait_for_frames().
                             apply_filter(decimation_filter).   // Find and colorize the depth data
                             apply_filter(spacial_filter).   // Find and colorize the depth data
                             apply_filter(temporal_filter).   // Find and colorize the depth data
                             apply_filter(hole_filter).   // Find and colorize the depth data
                             apply_filter(color_map).   // Find and colorize the depth data
                             apply_filter(align);    // Wait for next set of frames from the camera

        for (auto frame : data)
        {
            uchar* dataptr =  (uchar*)frame.get_data();
            int w = frame.as<rs2::video_frame>().get_width();
            int h = frame.as<rs2::video_frame>().get_height();

            cv::Mat tmp_mat = cv::Mat(h, w, CV_8UC3, dataptr, cv::Mat::AUTO_STEP);            

            if (frame.get_profile().stream_type() == rs2_stream::RS2_STREAM_COLOR)
            {
               // cv::Mat tmp_mat = cv::Mat(h,w, CV_8UC3, dataptr);            
                tmp_mat.copyTo(cv_color);
            }
            else if (frame.get_profile().stream_type() == rs2_stream::RS2_STREAM_DEPTH)
            {
                tmp_mat.copyTo(cv_depth);
            }
        }
        
        if (data.size() > 0)
        {
            double refresh_fps = ( sw.get_elapsed_ms() > 0) ? 1000 /  sw.get_elapsed_ms() : 0;
            sw.reset();
            
            std::stringstream fpsstr;
            fpsstr << "fps: " << std::fixed << std::setprecision(0) << refresh_fps;
            cv::putText(cv_color, fpsstr.str(), cv::Point(10,50), cv::FONT_HERSHEY_PLAIN, 4,  cv::Scalar(0x00, 0x00, 0xff), 4);   
            
            cv::imshow(window_rgb, cv_color);
            cv::imshow(window_depth, cv_depth);
        }
        
        int x = cv::waitKey(1);
        if(x == 27) // ESC Key = exit
        {
            break;
            std::cout << "Done..." << std::endl;
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}