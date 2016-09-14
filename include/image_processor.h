#ifndef IMAGE_PROCESSOR_H_INCLUDED
#define IMAGE_PROCESSOR_H_INCLUDED

#include <mutex>
#include <atomic>
#include "opencv2/opencv.hpp"

void ProcessImageThread( cv::Mat *orgimage,
						 std::mutex *capturemutex,
						 std::atomic<bool> *exitsignal );

#endif // IMAGE_PROCESSOR_H_INCLUDED
