#ifndef DISPLAY_HANDLER_H_INCLUDED
#define DISPLAY_HANDLER_H_INCLUDED

#include <mutex>
#include <atomic>
#include "opencv2/opencv.hpp"

void DisplayUpdateThread( cv::Mat *image,
						  std::mutex *displaymutex,
						  std::atomic<bool> *exitsignal);

#endif // DISPLAY_HANDLER_H_INCLUDED
