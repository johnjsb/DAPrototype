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
