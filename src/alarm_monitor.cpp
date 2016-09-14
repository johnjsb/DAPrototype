#include <iostream>
#include <algorithm> 
#include <atomic>
#include "alarm_monitor.h"
#include "lane_detect_processor.h"
#include "pace_setter_class.h"
#include "xml_reader.h"
#include "opencv2/opencv.hpp"

namespace alarmdata {
	std::mutex polygonmutex;
	Polygon polygon { cv::Point(0,0), cv::Point(0,0), cv::Point(0,0), cv::Point(0,0) };
	//LDW Status
	//0 = inactive (disabled by xml or below speed)
	//1 = left deviation (OK)
	//2 = right deviation (OK)
	//3 = left warning
	//4 = right warning
	//5 = left alarm
	//6 = right alarm
	//-1 = error (no lane identified)
	std::atomic<int> ldwstatus{0};
	//Scaled over-threshold value for PWM
	//0-1023
	std::atomic<int> ldwpwmvalue{1023};
	//FCW Status
	//0 = inactive (disabled by xml or zero speed)
	//1 = fcw warning
	//2 = following too close warning
	//3 = fcw alarm
	//4 = following too close alarm
	//5 = driver pullahead notification
	//-1 = error (sensor error)	
	std::atomic<int> fcwstatus{0};
	//Scaled over-threshold value for PWM
	//0-1023
	std::atomic<int> fcwpwmvalue{1023};
	//GPS Status
	//0 = inactive (disabled by xml)
	//1 = no GPS lock
	//2 = gps locked and speed Ok (below LDW threshold)
	//3 = gps locked and speed Ok (above LDW threshold)
	//4 = gps locked and speeding warning				//Future
	//5 = gps locked and speeding alarm					//Future
	//-1 = error (sensor error)	
	std::atomic<int> gpsstatus{0};
	std::atomic<double> gpsspeed{0.0};
	std::atomic<double> latitude{0.0};
	std::atomic<double> longitude{0.0};
	std::atomic<double> forwarddistance{0.0};
	std::atomic<double> timetocollision{0.0};
	
}

void AlarmMonitorThread( std::atomic<bool> *exitsignal )
{

	std::cout << "Alarm monitor thread starting!" << std::endl;

	//Create thread variables
	int cyclecount{0};
	
	//create pace setter
	PaceSetter alarmpacer(std::max(settings::disp::updatefps,
		settings::cam::recfps), "alarm monitor");
	
	//Loop indefinitely
	while( !(*exitsignal) ) {
				
		alarmpacer.SetPace();
	}
	
	std::cout << "Exiting alarm monitor thread!" << std::endl;

}