/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  License:
	  This software is licensed under GNU GPL v3.0
	  
******************************************************************************************/

//Standard libraries
#include <atomic>
#include <mutex>

//3rd party libraries
#include "opencv2/opencv.hpp"

//Project libraries
#include "lane_detect_processor.h"
#include "process_values_class.h"

/*****************************************************************************************/
ProcessValues::ProcessValues() :
	ldwstatus_{0},
	fcwstatus_{0},
	gpsstatus_{3},
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
