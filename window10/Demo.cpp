// This example is derived from ZJUNlicts technical blackout mission, and should be used as a demo for the robot class
// which adapted to be used with Intel RealSense Cameras
// under the Intel® Core™ i7-77000 CPU @ 3.60GHz × 4 

#include <opencv2/dnn.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include "example.hpp"
#include "cv-helpers.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <cmath>
#include <omp.h>
#include <fstream>
#include "iostream"
#include "time.h"
#include <chrono>
#include <thread>



using namespace cv;
using namespace cv::dnn;
using namespace rs2;
using namespace std;
using namespace rs400;

int main(int argc, char** argv) try
{


    // build one device
    context ctx;
    auto devices = ctx.query_devices();
    size_t device_count = devices.size();
    if (!device_count)
    {
        cout <<"No device detected. Is it plugged in?\n";
        return EXIT_SUCCESS;
    }

    // Get the first connected device
    auto dev = devices[0];
    // See if the device support advance mode(which is to enable using preset parameters)
    if (dev.is<rs400::advanced_mode>())
    {
        auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
        // Check if advanced-mode is enabled
        if (!advanced_mode_dev.is_enabled())
        {
            // Enable advanced-mode
            advanced_mode_dev.toggle_advanced_mode(true);
        }
    }
    else
    {
        cout << "Current device doesn't support advanced-mode!\n";
        return EXIT_FAILURE;
    }
   
    // Start streaming from Intel RealSense Camera
    pipeline pipe;

    //add filter

    // rs2::decimation_filter decimation_filter; You can add more filters!!!
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;
    rs2::hole_filling_filter hole_filling_filter;

    // decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, float(100));   
    temporal_filter.set_option(RS2_OPTION_HOLES_FILL,float(7)); 
    hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, float(2));   
    // spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
    auto config = pipe.start();

    // load one of the preset I used in lab (You can try your own preset saved in RealSense-viewer)
    std::ifstream t("../include/config/415highDensity.json");
    std::string str((std::istreambuf_iterator<char>(t)),
                std::istreambuf_iterator<char>());
    rs400::advanced_mode dev4json = config.get_device();
    dev4json.load_json(str);
    

    auto profile = config.get_stream(RS2_STREAM_COLOR)
                         .as<video_stream_profile>();
    rs2::align align_to(RS2_STREAM_COLOR);

 

	while (cvGetWindowHandle(window_name))
		// for(int i = 0; i<60 && cvGetWindowHandle(window_name) ; i++)
	{
		auto start_time = clock();

		// Wait for the next set of frames
		auto data = pipe.wait_for_frames();
		// Make sure the frames are spatially aligned
		data = align_to.process(data);

		auto color_frame = data.get_color_frame();
		auto depth_frame = data.get_depth_frame();
        depth_frame= temporal_filter.process(depth_frame);
        // depth_frame= hole_filling_filter.process(depth_frame);
        // depth_frame= spatial_filter.process(depth_frame);


		// If we only received new depth frame, 
		// but the color did not update, continue
		static int last_frame_number = 0;
		if (color_frame.get_frame_number() == last_frame_number) continue;
		last_frame_number = color_frame.get_frame_number();

		
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

