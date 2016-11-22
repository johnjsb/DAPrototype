/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  License:
	  This software is licensed under GNU GPL v3.0
	  
******************************************************************************************/

#ifndef IMAGE_CAPTURER_H_INCLUDED
#define IMAGE_CAPTURER_H_INCLUDED

#include <mutex>
#include <atomic>
#include "opencv2/opencv.hpp"

void CaptureImageThread( cv::Mat *capture,
						 std::mutex *capturemutex,
						 std::atomic<bool> *exitsignal);

#endif // IMAGE_CAPTURER_H_INCLUDED
