#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
//#include "example.hpp"
#include <cmath>
#include <omp.h>
#include "iostream"
#include "time.h"
#include <chrono>
#include <thread>
#include <mutex>
#include <list>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/opencv.hpp>

#include "kalmanfilter.h"

class GetImage{
public:
	GetImage();
	~GetImage();
	int check_camera();
	void load_JSON();
	void temporalFilter(double a = 0.55, double d = 100, int h = 7);
	void hole_filling_filter(int h = 2);
	void displayControl();
	void get_Frame();
	void get_FrameThread();
	bool get_RGBD_data();
	bool get_RGBD_dataThread();
	void convert_2_GMAT();
	bool convert_2_GMATThread();
	void rgb_2_HSV();
	void rgb_2_HSVThread();
	double depth_length_coefficient(double depth);
	void find_Contour(bool KF = false);
	void find_Contour_Tracking(bool KF = false);
	bool find_ContourThread(bool KF = false);
	void show_window();
	bool tracking(bool KF = false);


	cv::Mat imgThresholded;
	cv::Mat imgThresholdedThread;
	rs2::temporal_filter temporal_filter;
	cv::Rect crop;
	const char const* window_name;
	int iLowH = 0;
	int iHighH = 38;
	int iLowS = 71;
	int iHighS = 255;

	int iLowV = 203;
	int iHighV = 255;
	int magic_distance;
	int velocity;
	int y_vel = 0;
	int x_vel = 0;
	int length_to_mid;
	double last_x_meter = 0;
	double this_x_meter = 0;
	double last_y_meter = 0;
	double this_y_meter = 0;
	int count_for_while2 = 0;
	double lenght_to_midline_OFFSET = 0;
	double first_magic_distance = 500;
	//string move_direction;
	double move_distance = 0;
	cv::Moments moment;
	char key;
	int pixal_to_bottom = 480;
	cv::Mat Gcolor_mat;
	cv::Mat Gdepth_mat;
	cv::Rect2d object;
	cv::Ptr<cv::Tracker> tracker;
	cv::TrackerKCF::Params parameters;


private:
	rs2::pipeline pipe;
	rs2::pipeline_profile config;
	rs2::video_stream_profile *profile;
	rs2::align *align_to;
	rs2::frameset data;
	rs2::frameset dataThread;
	rs2::video_frame *color_frame;
	rs2::depth_frame *depth_frame;

	cv::Mat Gcolor_matThread;
	cv::Mat Gdepth_matThread;
	cv::Mat imgHSV;
	std::list<rs2::frameset> frameset_queue;
	std::list<rs2::video_frame> color_frame_queue;
	std::list<rs2::depth_frame> depth_frame_queue;
	std::list<cv::Mat> color_mat_queue;
	std::list<cv::Mat> depth_mat_queue;
	std::list<cv::Mat> imgThreshold_queue;
	std::mutex framesetlock;
	std::mutex color_depth_lock;
	std::mutex toHSV_lock;
	std::mutex hsv_2_contour_lock;
};