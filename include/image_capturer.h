#ifndef IMAGE_CAPTURER_H_INCLUDED
#define IMAGE_CAPTURER_H_INCLUDED

#include <mutex>
#include <atomic>
#include "opencv2/opencv.hpp"

void CaptureImageThread( cv::Mat *capture,
						 std::mutex *capturemutex,
						 std::atomic<bool> *exitsignal);

#endif // IMAGE_CAPTURER_H_INCLUDED
