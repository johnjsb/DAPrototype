#ifndef FRAMEQUEUE_H
#define FRAMEQUEUE_H

#include <mutex>
#include <queue>
#include "opencv2/opencv.hpp"

class FrameQueue
{
	public:
		struct cancelled {};
		FrameQueue();
		void Push( cv::Mat const& image );
		cv::Mat Pop();
		void Cancel();
		void Stop();
		void Restart();
		bool CheckCanceled();
		bool CheckPaused();
		bool CheckReleased();
		void ReleaseFile();
		FrameQueue& operator= (const FrameQueue &q);
		virtual ~FrameQueue();

	protected:

	private:
		std::queue<cv::Mat> queue_;
		std::mutex mutex_;
		bool canceled_;
		bool paused_;
		bool released_;
};

#endif // FRAMEQUEUE_H
