#include "process_values_class.h"
#include <atomic>
#include <mutex>
#include "lane_detect_processor.h"
#include "opencv2/opencv.hpp"

ProcessValues::ProcessValues() :
	ldwstatus_{0},
	ldwpwmvalue_{0},
	fcwstatus_{0},
	fcwpwmvalue_{0},
	gpsstatus_{0},
	gpsspeed_{0.0},
	latitude_{0.0},
	longitude_{0.0},
	forwarddistance_{0.0},
	timetocollision_{0.0},
	polygon_{ cv::Point(0,0), cv::Point(0,0), cv::Point(0,0), cv::Point(0,0) }
{

}

ProcessValues::~ProcessValues()
{

}

Polygon ProcessValues::GetPolygon()
{
    std::lock_guard<std::mutex> d_guard(polygonmutex_);
	return polygon_;
}

void ProcessValues::SetPolygon( Polygon &polygon )
{
    std::lock_guard<std::mutex> d_guard(polygonmutex_);
	std::copy(std::begin(polygon), std::end(polygon),
		std::begin(polygon_));
	return;	
}
