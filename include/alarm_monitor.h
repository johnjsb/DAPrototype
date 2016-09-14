#pragma once

#ifndef ALARM_MONITOR_H_INCLUDED
#define ALARM_MONITOR_H_INCLUDED

#include <atomic>
#include <mutex>
#include "lane_detect_processor.h"
#include "opencv2/opencv.hpp"

namespace alarmdata {
	extern std::mutex polygonmutex;
	extern Polygon polygon;
	//LDW Status
	//0 = inactive (disabled by xml or below speed)
	//1 = left deviation (OK)
	//2 = right deviation (OK)
	//3 = left warning
	//4 = right warning
	//5 = left alarm
	//6 = right alarm
	//-1 = error (no lane identified)	
	extern std::atomic<int> ldwstatus;
	//Scaled over-threshold value for PWM
	//0-1023
	extern std::atomic<int> ldwpwmvalue;
	//FCW Status
	//0 = inactive (disabled by xml or zero speed)
	//1 = fcw warning
	//2 = following too close warning
	//3 = fcw alarm
	//4 = following too close alarm
	//5 = driver pullahead notification
	//-1 = error (sensor error)	
	extern std::atomic<int> fcwstatus;
	//Scaled over-threshold value for PWM
	//0-1023
	extern std::atomic<int> fcwpwmvalue;
	//GPS Status
	//0 = inactive (disabled by xml)
	//1 = no GPS lock
	//2 = gps locked and speed Ok (below LDW threshold)
	//3 = gps locked and speed Ok (above LDW threshold)
	//4 = gps locked and speeding warning				//Future
	//5 = gps locked and speeding alarm					//Future
	//-1 = error (sensor error)	
	extern std::atomic<int> gpsstatus;
	extern std::atomic<double> gpsspeed;
	extern std::atomic<double> latitude;
	extern std::atomic<double> longitude;
	extern std::atomic<double> forwarddistance;
	extern std::atomic<double> timetocollision;
	
}

void AlarmMonitorThread( std::atomic<bool> *exitsignal );

#endif // ALARM_MONITOR_H_INCLUDED
