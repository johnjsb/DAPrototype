#ifndef PROCESSVALUES_H
#define PROCESSVALUES_H

#include <atomic>
#include <mutex>
#include "lane_detect_processor.h"
#include "opencv2/opencv.hpp"

class ProcessValues
{
    public:
        ProcessValues();
        virtual ~ProcessValues();
		Polygon GetPolygon();
		void SetPolygon( Polygon &polygon );
		//LDW Status
		//0 = inactive (disabled by xml or below speed)
		//1 = left deviation (OK)
		//2 = right deviation (OK)
		//3 = left warning
		//4 = right warning
		//5 = left alarm
		//6 = right alarm
		//-1 = error (no lane identified)	
		std::atomic<int> ldwstatus_;
		//Scaled over-threshold value for PWM
		//0-1023
		std::atomic<int> ldwpwmvalue_;
		//FCW Status
		//0 = inactive (disabled by xml or zero speed)
		//1 = fcw warning
		//2 = following too close warning
		//3 = fcw alarm
		//4 = following too close alarm
		//5 = driver pullahead notification
		//-1 = error (sensor error)	
		std::atomic<int> fcwstatus_;
		//Scaled over-threshold value for PWM
		//0-1023
		std::atomic<int> fcwpwmvalue_;
		//GPS Status
		//0 = inactive (disabled by xml)
		//1 = no GPS lock
		//2 = gps locked and speed Ok (below LDW threshold)
		//3 = gps locked and speed Ok (above LDW threshold)
		//4 = gps locked and speeding warning				//Future
		//5 = gps locked and speeding alarm					//Future
		//-1 = error (sensor error)	
		std::atomic<int> gpsstatus_;
		std::atomic<double> gpsspeed_;
		std::atomic<double> latitude_;
		std::atomic<double> longitude_;
		std::atomic<double> forwarddistance_;
		std::atomic<double> timetocollision_;

    protected:

    private:
		Polygon polygon_;
		std::mutex polygonmutex_;
};

#endif // PROCESSVALUES_H
