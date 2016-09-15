#ifndef IMAGE_PROCESSOR_H_INCLUDED
#define IMAGE_PROCESSOR_H_INCLUDED

#include <mutex>
#include <atomic>
#include "opencv2/opencv.hpp"
#include "process_values_class.h"

void ProcessImageThread( cv::Mat *orgimage,
                         std::mutex *capturemutex,
						 ProcessValues *processvalues,
                         std::atomic<bool> *exitsignal );

#endif // IMAGE_PROCESSOR_H_INCLUDED
