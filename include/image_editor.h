#ifndef IMAGE_EDITOR_H_INCLUDED
#define IMAGE_EDITOR_H_INCLUDED

#include <mutex>
#include <atomic>
#include "opencv2/opencv.hpp"
#include "process_values_class.h"

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
