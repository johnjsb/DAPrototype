#include <iostream>
#include <algorithm> 
#include <mutex>
#include <atomic>
#include <deque>
#include <array>
#include "alarm_monitor.h"
#include "lane_detect_processor.h"
#include "pace_setter_class.h"
#include "xml_reader.h"
#include "opencv2/opencv.hpp"

void ProcessImageThread( cv::Mat *orgimage,
                         std::mutex *capturemutex,
                         std::atomic<bool> *exitsignal )
{

	std::cout << "Image processor thread starting!" << std::endl;
	
	//Check if LDW is enabled
	if ( !settings::ldw::enabled ) {
		std::cout << "LDW disabled, exiting image processor thread!" <<
			std::endl;
		return;
	}
	
	//Check image is initialized
	while ( orgimage->empty() ) {
		if (*exitsignal) {
			return;
		}	  
	}

	//Create thread variables
	std::deque<Polygon> pastpolygons;
	
	//create pace setter
	PaceSetter processorpacer( 50, "image processor");
//	PaceSetter processorpacer(std::max(settings::disp::updatefps,
//		settings::cam::recfps), "image processor");
	
	//Loop indefinitely
	while( !(*exitsignal) ) {
		if ( (alarmdata::gpsstatus > 2) && settings::ldw::enabled ) {
			//Get image
			capturemutex->lock();
			cv::Mat processimage{ orgimage->clone() };
			capturemutex->unlock();
			
			//Get lanes
			Polygon newpolygon;
			ProcessImage( processimage, newpolygon );
			AveragePolygon( newpolygon, pastpolygons, settings::ldw::samplestoaverage,
				settings::ldw::samplestokeep );	

			//Evaluate LDW
			if ( newpolygon[0] != cv::Point(0,0) ) {
				double deviationpix = 0.5*( newpolygon[0].x +
					newpolygon[1].x - settings::cam::pixwidth );
				double deviationper = 100.0 * deviationpix /
					static_cast<double>(settings::cam::pixwidth);			
				if ( 0.0 < deviationper && deviationper < settings::ldw::peroffsetwarning
					) {
					alarmdata::ldwstatus = 2;
					alarmdata::ldwpwmvalue = 1023 + static_cast<int>((1024.0*(deviationper -
						settings::ldw::peroffsetwarning)) /
						(settings::ldw::peroffsetwarning));
				} else if ( settings::ldw::peroffsetwarning < deviationper &&
					deviationper < settings::ldw::peroffsetalarm ) {
					alarmdata::ldwstatus = 4;
					alarmdata::ldwpwmvalue = 1023 + static_cast<int>((1024.0*(deviationper -
						settings::ldw::peroffsetwarning)) /
						(settings::ldw::peroffsetwarning));
				} else if ( settings::ldw::peroffsetalarm < deviationper ) {
					alarmdata::ldwstatus = 6;
					alarmdata::ldwpwmvalue = 1023;
				} else if ( 0.0 > deviationper && deviationper >
					(-1.0 * settings::ldw::peroffsetwarning) ) {
					alarmdata::ldwstatus = 1;
					alarmdata::ldwpwmvalue = 1023 - static_cast<int>((1024.0*(deviationper +
						settings::ldw::peroffsetwarning)) /
						(settings::ldw::peroffsetwarning));
				} else if ( (-1.0 * settings::ldw::peroffsetwarning) > deviationper &&
					deviationper > (-1.0 * settings::ldw::peroffsetalarm) ) {
					alarmdata::ldwstatus = 3;
					alarmdata::ldwpwmvalue = 1023 - static_cast<int>((1024.0*(deviationper +
						settings::ldw::peroffsetalarm)) /
						(settings::ldw::peroffsetalarm - settings::ldw::peroffsetwarning));
				} else if ( (-1.0 * settings::ldw::peroffsetalarm) > deviationper ) {
					alarmdata::ldwstatus = 5;
					alarmdata::ldwpwmvalue = 1023;
				}
			} else {
				alarmdata::ldwstatus = -1;
				alarmdata::ldwpwmvalue = 0;
			}
			if (alarmdata::ldwpwmvalue < 0) {
				alarmdata::ldwpwmvalue = 0;
			} else if (alarmdata::ldwpwmvalue > 1023) {
				alarmdata::ldwpwmvalue = 1023;
			}
			//Write new data
			alarmdata::polygonmutex.lock();
			std::copy(std::begin(newpolygon), std::end(newpolygon),
				std::begin(alarmdata::polygon));
			alarmdata::polygonmutex.unlock();
		} else {
			alarmdata::ldwstatus = 0;
		}
		
		processorpacer.SetPace();
	}
	
	std::cout << "Exiting image processor thread!" << std::endl;

}
