#include "libfreenect.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
//#include "ros/ros.h"

using namespace cv;
using namespace std;


class myMutex {
	public:
		myMutex() {
			pthread_mutex_init( &m_mutex, NULL );
		}
		void lock() {
			pthread_mutex_lock( &m_mutex );
		}
		void unlock() {
			pthread_mutex_unlock( &m_mutex );
		}
	private:
		pthread_mutex_t m_mutex;
};


class MyFreenectDevice : public Freenect::FreenectDevice {
	public:
		MyFreenectDevice(freenect_context *_ctx, int _index)
	 		: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
			m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
			m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1),
			rgbMat(Size(640,480), CV_8UC3, Scalar(0)),
			ownMat(Size(640,480),CV_8UC3,Scalar(0)) {
			
			for( unsigned int i = 0 ; i < 2048 ; i++) {
				float v = i/2048.0;
				v = std::pow(v, 3)* 6;
				m_gamma[i] = v*6*256;
			}
		}
		
		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp) {
			//std::cout << "RGB callback" << std::endl;
			m_rgb_mutex.lock();
			uint8_t* rgb = static_cast<uint8_t*>(_rgb);
			rgbMat.data = rgb;
			m_new_rgb_frame = true;
			m_rgb_mutex.unlock();
		};
		
		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp) {
			//std::cout << "Depth callback" << std::endl;
			m_depth_mutex.lock();
			uint16_t* depth = static_cast<uint16_t*>(_depth);
			depthMat.data = (uchar*) depth;
			m_new_depth_frame = true;
			m_depth_mutex.unlock();
		}
		
		bool getVideo(Mat& output) {
			m_rgb_mutex.lock();
			if(m_new_rgb_frame) {
				cv::cvtColor(rgbMat, output, CV_RGB2BGR);
				m_new_rgb_frame = false;
				m_rgb_mutex.unlock();
				return true;
			} else {
				m_rgb_mutex.unlock();
				return false;
			}
		}
		
		bool getDepth(Mat& output) {
				m_depth_mutex.lock();
				if(m_new_depth_frame) {
					depthMat.copyTo(output);
					m_new_depth_frame = false;
					m_depth_mutex.unlock();
					return true;
				} else {
					m_depth_mutex.unlock();
					return false;
				}
			}
	private:
		std::vector<uint8_t> m_buffer_depth;
		std::vector<uint8_t> m_buffer_rgb;
		std::vector<uint16_t> m_gamma;
		Mat depthMat;
		Mat rgbMat;
		Mat ownMat;
		myMutex m_rgb_mutex;
		myMutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
};


int main(int argc, char **argv) {
	
	Mat frame;
	Mat frame_mask;
	Mat frame_mask_thresh;
	Mat frame_with_keypoints;
	
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	Ptr<BackgroundSubtractorMOG2> mog = createBackgroundSubtractorMOG2();
	mog->setHistory(150);

	Freenect::Freenect freenect;
	MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
	
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();
	vector<KeyPoint> keypoints;
	
	//namedWindow("Frame Thresh", CV_WINDOW_AUTOSIZE);
	namedWindow("Frame", CV_WINDOW_AUTOSIZE);
	device.startDepth();
	device.startVideo();
	while(true) {
		device.getDepth(frame);
		//cout<<  frame.type() << endl;

		/*frame.convertTo(frame, CV_8UC1, 255.0/2048.0);
		imshow("Frame", frame);
		waitKey(30);*/
		//cout << frame_mask;
		mog->apply(frame, frame_mask);
		threshold(frame_mask, frame_mask_thresh, 50, 255, THRESH_BINARY);
		erode(frame_mask_thresh, frame_mask_thresh, Mat(), Point(-1,-1), 2);
		dilate(frame_mask_thresh, frame_mask_thresh, Mat(), Point(-1,-1), 1);
		
		
		//detector->detect(frame_mask, keypoints);
		//drawKeypoints(frame_mask, keypoints, frame_with_keypoints, 
		//	Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
		findContours(frame_mask_thresh, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		double largest_area = 0;
		int largest_contour_index = 0;
		if (contours.size() > 0) {
			for (int i = 0; i < contours.size(); i++) {
				double a = contourArea(contours[i], false);
				if (a > largest_area) {
					largest_area = a;
					largest_contour_index = i;
				}
			}
			Point2f p;
			float r;
			minEnclosingCircle(contours[largest_contour_index], p, r);
			circle(frame_mask_thresh, p, r/4, 0, 3);
			//cout << frame << endl;
			cout << p.x << " " << p.y  << " " << frame.at<unsigned short>((int)p.x, (int)p.y) << endl;
		}


		imshow("Frame Thresh", frame_mask_thresh);
		waitKey(30);
	}
	
	device.stopVideo();
	device.stopDepth();
	
	
	

/*
	bool die(false);
	string filename("snapshot");
	string suffix(".png");
	int i_snap(0),iter(0);
	
	Mat depthMat(Size(640,480),CV_16UC1);
	Mat depthf (Size(640,480),CV_8UC1);
	Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
	Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));
	
	// The next two lines must be changed as Freenect::Freenect
	// isn't a template but the method createDevice:
	// Freenect::Freenect<MyFreenectDevice> freenect;
	// MyFreenectDevice& device = freenect.createDevice(0);
	// by these two lines:
	
	Freenect::Freenect freenect;
	MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
	
	cv::SimpleBlobDetector detector;
	std::vector<KeyPoint> keypoints;
	
	namedWindow("rgb",CV_WINDOW_AUTOSIZE);
	//namedWindow("depth",CV_WINDOW_AUTOSIZE);
	device.startVideo();
	//device.startDepth();
	while (!die) {
		device.getVideo(rgbMat);
		device.getDepth(depthMat);
		detector.detect(rgbMat, keypoints);
		Mat im_with_keypoints;
		cv::drawKeypoints(rgbMat, keypoints, im_with_keypoints, Scalar(0,0,255),
			DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::imshow("rgb", im_with_keypoints);

			
		depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
		//cv::imshow("depth",depthf);
		char k = cvWaitKey(5);
		if( k == 27 ){
			cvDestroyWindow("rgb");
			cvDestroyWindow("depth");
			break;
		}
		if( k == 8 ) {
			std::ostringstream file;
			file << filename << i_snap << suffix;
			cv::imwrite(file.str(),rgbMat);
			i_snap++;
		}
		if(iter >= 1000) break;
		iter++;
	}
	
	device.stopVideo();
	device.stopDepth();
	return 0;*/
	return 0;
}

