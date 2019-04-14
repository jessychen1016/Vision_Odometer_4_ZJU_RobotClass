#include <stdio.h>
#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>
#include <assert.h>
#include <string>
#include <opencv2/opencv.hpp>
using namespace std;
 
class cvThread
{
public:
	cvThread();
	~cvThread();
	/*paramThread用于传递线程需要的参数值*/
	struct paramThread
	{
		int w;
		int h;
		uchar * data;
	};
 
	/********************************************************
	*	@brief       : 多线程要处理的图像操作
	*	@param  args : 多线程传入的参数
	*	@return      : void
	********************************************************/
	static void * cvThreadBlur(void* args);
 
	/********************************************************
	*	@brief        : 多线程处理函数
	*	@param  image : 输入/输出Mat图像
	*	@param  type  : 分割类型0：垂直分割(推荐)，1：水平分割（不推荐）
	*	@param  thread_num : 多线程个数
	*	@return      : void
	********************************************************/
	void  cvThreadProcess(cv::Mat &image, const int type, const int thread_num);
	void  cvThreadProcess(cv::Mat &image);
	/********************************************************
	*	@brief       : 实现图像分割，
	*	@param  num  :  分割个数
	*	@param  type : 0：垂直分割(推荐)，1：水平分割（不推荐）
	*	@return      : vector<cv::Mat>
	*   PS：使用水平分割时（type=1），处理完后必须调用catImage进行拼接，
	*   使用垂直分割时（type=0），可以不进行catImage，因为是对原图进行操作的
	********************************************************/
	vector<cv::Mat> splitImage(cv::Mat image, int num, int type);
 
 
	/********************************************************
	*	@brief       : 实现图像拼接，
	*	@param  v    :
	*	@param  type : 0：垂直拼接，1：水平拼接
	*	@return      : Mat
	********************************************************/
	cv::Mat catImage(vector<cv::Mat> v, int type);
};
