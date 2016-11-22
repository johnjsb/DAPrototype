/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  License:
	  This software is licensed under GNU GPL v3.0
	  
******************************************************************************************/

#ifndef VIDEO_WRITER_H_INCLUDED
#define VIDEO_WRITER_H_INCLUDED

#include <mutex>
#include <atomic>
#include "opencv2/opencv.hpp"

void VideoWriterThread ( cv::Mat *orgimage,
                         std::mutex *capturemutex,
                         cv::Mat *modimage,
                         std::mutex *displaymutex,
                         std::atomic<bool> *exitsignal );
int fileShift( std::string filename,
               int numOfFiles );

#endif // VIDEO_WRITER_H_INCLUDED
