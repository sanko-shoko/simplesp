//--------------------------------------------------------------------------------
// Copyright (c) 2017-2018, sanko-shoko. All rights reserved.
//--------------------------------------------------------------------------------

#ifndef __SPCV_UTIL_H__
#define __SPCV_UTIL_H__

#include "simplesp.h"
#include <opencv2/opencv.hpp>

namespace sp{

	//--------------------------------------------------------------------------------
	// convert
	//--------------------------------------------------------------------------------

	SP_CPUFUNC bool cvCnvImg(Mem<Byte> &dst, const cv::Mat &src){
		if (src.type() == CV_8UC1){
			cnvPtrToImg(dst, src.ptr(), src.size().width, src.size().height, 1);
		}
		if (src.type() == CV_8UC3){
			cv::Mat rgb;
			cv::cvtColor(src, rgb, CV_BGR2RGB);
			cnvPtrToImg(dst, rgb.ptr(), rgb.size().width, rgb.size().height, 3);
		}
		return true;
	}

	SP_CPUFUNC bool cvCnvImg(Mem<Col3> &dst, const cv::Mat &src){
		if (src.type() == CV_8UC1){
			cnvPtrToImg(dst, src.ptr(), src.size().width, src.size().height, 1);
		}
		if (src.type() == CV_8UC3){
			cv::Mat rgb;
			cv::cvtColor(src, rgb, CV_BGR2RGB);
			cnvPtrToImg(dst, rgb.ptr(), rgb.size().width, rgb.size().height, 3);
		}
		return true;
	}

	SP_CPUFUNC bool cvCnvImg(cv::Mat &dst, const Mem<Byte> &src){
		dst = cv::Mat(src.dsize[1], src.dsize[0], CV_8UC1);
		memcpy(dst.ptr(), src.ptr, src.size() * sizeof(Byte));
		return true;
	}

	SP_CPUFUNC bool cvCnvImg(cv::Mat &dst, const Mem<Col3> &src){
		cv::Mat rgb(src.dsize[1], src.dsize[0], CV_8UC3);
		memcpy(rgb.ptr(), src.ptr, src.size() * sizeof(Col3));

		cv::cvtColor(rgb, dst, CV_RGB2BGR);
		return true;
	}


	//--------------------------------------------------------------------------------
	// capture Mem2 image
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC bool cvCaptureImg(Mem<TYPE> &dst, cv::VideoCapture &cap, const int init = 0){
		if (cap.isOpened() == false){
			if (cap.open(init) == false){
				SP_PRINTF("could not find USB camera\n");
				return false;
			}
		}
		cv::Mat img;
		cap >> img;

		return cvCnvImg(dst, img);
	}

	SP_CPUFUNC void cvSetCaptureSize(cv::VideoCapture &cap, const int width, const int height){
		cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
	}

	//--------------------------------------------------------------------------------
	// load Mem2 image
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC bool cvLoadImg(Mem<TYPE> &dst, const char *path){
		const cv::Mat cvimg = cv::imread(path);
		return cvCnvImg(dst, cvimg);
	}

	//--------------------------------------------------------------------------------
	// save Mem2 image
	//--------------------------------------------------------------------------------

	template<typename TYPE>
	SP_CPUFUNC bool cvSaveImg(const Mem<TYPE> &src, const char *path){
		cv::Mat cvimg;
		cvCnvImg(cvimg, src);
				
		return cv::imwrite(path, cvimg);
	}

	//--------------------------------------------------------------------------------
	// show image
	//--------------------------------------------------------------------------------
	
	template<typename TYPE>
	SP_CPUFUNC void cvShowImg(const Mem<TYPE> &src, const char *name){
		cv::Mat cvimg;
		cvCnvImg(cvimg, src);

		cv::imshow(name, cvimg);
	}


}

#endif