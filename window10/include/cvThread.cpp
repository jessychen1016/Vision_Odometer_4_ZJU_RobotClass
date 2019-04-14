#include "cvThread.h"
 
 
 
cvThread::cvThread()
{
}
 
 
cvThread::~cvThread()
{
}
/********************************************************
*	@brief       : 多线程处理函数
*	@param  args : 多线程传入的参数
*	@return      : void
********************************************************/
void * cvThread::cvThreadBlur(void* args) {
	pthread_t myid = pthread_self();
	paramThread *para = (paramThread *)args;
	int w = para->w;
	int h = para->h;
	cv::Mat image(h, w, CV_8UC3, (uchar *)para->data);
	/***************************************************/
	/*这里实现多线程要处理的图像操作*/
	cv::blur(image, image, cv::Size(7, 7), cv::Point(-1, -1), cv::BORDER_REPLICATE);
 
	/***************************************************/
	pthread_exit(NULL);
	return NULL;
}
 
 
 
#define THREAD_NUMS 4
void  cvThread::cvThreadProcess(cv::Mat &image) {
 
	/*使用多线程图像处理*/
	int type = 0;
	vector<cv::Mat> v = splitImage(image, THREAD_NUMS, type);
	paramThread args[THREAD_NUMS];
	pthread_t pt[THREAD_NUMS];	//创建thread_num个子线程
	for (size_t i = 0; i < THREAD_NUMS; i++)
	{
		args[i].h = v.at(i).rows;
		args[i].w = v.at(i).cols;
		args[i].data = v.at(i).data;
		pthread_create(&pt[i], NULL, &cvThreadBlur, (void *)(&args[i]));
	}
	/*等待全部子线程处理完毕*/
	for (size_t i = 0; i < THREAD_NUMS; i++)
	{
		pthread_join(pt[i], NULL);
	}
	cv::Mat dest = catImage(v, type);
}
 
void  cvThread::cvThreadProcess(cv::Mat &image,const int type,const int thread_num) {
	/*使用多线程图像处理*/
	vector<cv::Mat> v = splitImage(image, thread_num, type);
	paramThread *args = new paramThread[thread_num];
	pthread_t *pt = new pthread_t[thread_num];	//创建thread_num个子线程
	for (size_t i = 0; i < thread_num; i++)
	{
		args[i].h = v.at(i).rows;
		args[i].w = v.at(i).cols;
		args[i].data = v.at(i).data;
		pthread_create(&pt[i], NULL, &cvThreadBlur, (void *)(&args[i]));
	}
	/*等待全部子线程处理完毕*/
	for (size_t i = 0; i < thread_num; i++)
	{
		pthread_join(pt[i], NULL);
	}
	/*PS：使用水平分割时（type = 1），处理完后必须调用catImage进行拼接，
	使用垂直分割时（type = 0），可以不进行catImage，因为是对原图进行操作的*/
	if (type==1)
	{
		image = catImage(v, type);
	}
	delete []args;
	delete []pt;
}
/********************************************************
*	@brief       : 实现图像分割，
*	@param  num  :  分割个数
*	@param  type : 0：垂直分割(推荐)，1：水平分割（不推荐）
*	@return      : vector<cv::Mat>
*   PS：使用水平分割时（type=1），处理完后必须调用catImage进行拼接，
*   使用垂直分割时（type=0），可以不进行catImage，因为是对原图进行操作的
********************************************************/
vector<cv::Mat> cvThread::splitImage(cv::Mat image, int num, int type) {
	int rows = image.rows;
	int cols = image.cols;
	vector<cv::Mat> v;
	if (type == 0) {//垂直分割
		for (size_t i = 0; i < num; i++) {
			int star = rows / num*i;
			int end = rows / num*(i + 1);
			if (i == num - 1) {
				end = rows;
			}
			//cv::Mat b = image.rowRange(star, end);
			v.push_back(image.rowRange(star, end));
		}
	}
	else if (type == 1) {//水平分割
		for (size_t i = 0; i < num; i++) {
			int star = cols / num*i;
			int end = cols / num*(i + 1);
			if (i == num - 1) {
				end = cols;
			}
			//cv::Mat b = image.colRange(star, end);
			/*解决水平分割的Bug:必须clone()*/
			v.push_back(image.colRange(star, end).clone());
		}
	}
	return  v;
}
 
/********************************************************
*	@brief       : 实现图像拼接，
*	@param  v    :
*	@param  type : 0：垂直拼接，1：水平拼接
*	@return      : Mat
********************************************************/
cv::Mat cvThread::catImage(vector<cv::Mat> v, int type) {
	cv::Mat dest = v.at(0);
	for (size_t i = 1; i < v.size(); i++)
	{
		if (type == 0)//垂直拼接
		{
			cv::vconcat(dest, v.at(i), dest);
		}
		else if (type == 1)//水平拼接
		{
			cv::hconcat(dest, v.at(i), dest);
		}
	}
	return dest;
}
