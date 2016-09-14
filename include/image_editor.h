#ifndef IMAGE_EDITOR_H_INCLUDED
#define IMAGE_EDITOR_H_INCLUDED

#include <mutex>
#include <atomic>
#include "opencv2/opencv.hpp"

void ImageEditorThread( cv::Mat *orgimage,
                        std::mutex *capturemutex,
						cv::Mat *displayimage,
						std::mutex *displaymutex,
						std::atomic<bool> *exitsignal );

void OverlayImage( cv::Mat* overlay,
			       cv::Mat* src,
			       double transparency );
			   
std::string ConvertLatLong ( double latitude, 
	                         double longitude );

std::string GetDiagnosticString ();

#endif // IMAGE_EDITOR_H_INCLUDED
