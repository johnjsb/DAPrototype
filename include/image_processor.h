/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  License:
	  This software is licensed under GNU GPL v3.0
	  
******************************************************************************************/

//Header guard
#ifndef IMAGE_PROCESSOR_H_INCLUDED
#define IMAGE_PROCESSOR_H_INCLUDED

//Standard libraries
#include <mutex>
#include <atomic>

//3rd party libraries
#include "opencv2/opencv.hpp"

//Project libraries
#include "process_values_class.h"

/*****************************************************************************************/
void ProcessImageThread( cv::Mat *orgimage,
                         std::mutex *capturemutex,
						 ProcessValues *processvalues,
                         std::atomic<bool> *exitsignal );

#endif // IMAGE_PROCESSOR_H_INCLUDED
