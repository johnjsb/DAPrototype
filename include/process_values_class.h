/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  License:
	  This software is licensed under GNU GPL v3.0
	  
******************************************************************************************/

//Header guard
#ifndef PROCESSVALUES_H
#define PROCESSVALUES_H

//Standard libraries
#include <atomic>
#include <mutex>

//Project libraries
#include "lane_detect_processor.h"

//3rd party libraries
#include "opencv2/opencv.hpp"

/*****************************************************************************************/
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
		//FCW Status
		//0 = inactive (disabled by xml or zero speed)
		//1 = fcw warning
		//2 = following too close warning
		//3 = fcw alarm
		//4 = following too close alarm
		//5 = driver pullahead notification
		//-1 = error (sensor error)	
		std::atomic<int> fcwstatus_;
		//GPS Status
		//0 = inactive (disabled by xml)
		//1 = no GPS lock
		//2 = gps locked and speed Ok (below LDW threshold)
		//3 = gps locked and speed Ok (above LDW threshold)
		//4 = gps locked and speeding warning				//Future
		//5 = gps locked and speeding alarm					//Future
		//-1 = error (sensor error)	
		std::atomic<int> gpsstatus_;
		std::atomic<float> gpsspeed_;
		std::atomic<double> latitude_;
		std::atomic<double> longitude_;
		std::atomic<float> forwarddistance_;
		std::atomic<float> timetocollision_;

    protected:

    private:
		Polygon polygon_;
		std::mutex polygonmutex_;
};

#endif // PROCESSVALUES_H
