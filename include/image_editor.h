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
#ifndef IMAGE_EDITOR_H_INCLUDED
#define IMAGE_EDITOR_H_INCLUDED

//Standard libraries
#include <mutex>
#include <atomic>

//3rd party libraries
#include "opencv2/opencv.hpp"

//Project libraries
#include "process_values_class.h"

/*****************************************************************************************/
void ImageEditorThread( cv::Mat *orgimage,
                        std::mutex *capturemutex,
						cv::Mat *displayimage,
						std::mutex *displaymutex,
						ProcessValues *processvalues,
						std::atomic<bool> *exitsignal );

void OverlayImage( cv::Mat* overlay,
			       cv::Mat* src );
			   
std::string ConvertLatLong ( double latitude, 
	                         double longitude );

std::string GetDiagnosticString ( int ldwstatus, int fcwstatus, int gpsstatus );

#endif // IMAGE_EDITOR_H_INCLUDED
