#include <iostream>
#include <algorithm> 
#include <mutex>
#include <atomic>
#include <deque>
#include <array>
#include "lane_detect_processor.h"
#include "pace_setter_class.h"
#include "process_values_class.h"
#include "xml_reader.h"
#include "opencv2/opencv.hpp"

void ProcessImageThread( cv::Mat *orgimage,
                         std::mutex *capturemutex,
						 ProcessValues *processvalues,
                         std::atomic<bool> *exitsignal )
{

	std::cout << "Image processor thread starting!" << '\n';
	
	//Check if LDW is enabled
	if ( !settings::ldw::kenabled ) {
		std::cout << "LDW disabled, exiting image processor thread!" <<
			'\n';
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
	PaceSetter processorpacer(settings::ldw::kupdatefps, "image processor");
	
	//Loop indefinitely
	while( !(*exitsignal) ) {
		if ( (processvalues->gpsstatus_ != 2) && settings::ldw::kenabled ) {
			//Get image
			capturemutex->lock();
			cv::Mat processimage{ *orgimage };
			capturemutex->unlock();
			
			//Get lanes
			Polygon newpolygon;
			ProcessImage( processimage, newpolygon );
			AveragePolygon( newpolygon, pastpolygons, settings::ldw::ksamplestoaverage,
				settings::ldw::ksamplestokeep );	

			//Evaluate LDW
			if ( newpolygon[0] != cv::Point(0,0) ) {
				double deviationpix = 0.5*( newpolygon[0].x +
					newpolygon[1].x - settings::cam::kpixwidth );
				double deviationper = 100.0 * deviationpix /
					static_cast<double>(newpolygon[1].x - newpolygon[0].x);
					//static_cast<double>(settings::cam::kpixwidth);			
				if ( 0.0 < deviationper && deviationper < settings::ldw::kperoffsetwarning
					) {
					processvalues->ldwstatus_ = 2;
					processvalues->ldwpwmvalue_ = 1023 + static_cast<int>((1024.0*(deviationper -
						settings::ldw::kperoffsetwarning)) /
						(settings::ldw::kperoffsetwarning));
				} else if ( settings::ldw::kperoffsetwarning < deviationper &&
					deviationper < settings::ldw::kperoffsetalarm ) {
					processvalues->ldwstatus_ = 4;
					processvalues->ldwpwmvalue_ = 1023 + static_cast<int>((1024.0*(deviationper -
						settings::ldw::kperoffsetwarning)) /
						(settings::ldw::kperoffsetwarning));
				} else if ( settings::ldw::kperoffsetalarm < deviationper ) {
					processvalues->ldwstatus_ = 6;
					processvalues->ldwpwmvalue_ = 1023;
				} else if ( 0.0 > deviationper && deviationper >
					(-1.0 * settings::ldw::kperoffsetwarning) ) {
					processvalues->ldwstatus_ = 1;
					processvalues->ldwpwmvalue_ = 1023 - static_cast<int>((1024.0*(deviationper +
						settings::ldw::kperoffsetwarning)) /
						(settings::ldw::kperoffsetwarning));
				} else if ( (-1.0 * settings::ldw::kperoffsetwarning) > deviationper &&
					deviationper > (-1.0 * settings::ldw::kperoffsetalarm) ) {
					processvalues->ldwstatus_ = 3;
					processvalues->ldwpwmvalue_ = 1023 - static_cast<int>((1024.0*(deviationper +
						settings::ldw::kperoffsetalarm)) /
						(settings::ldw::kperoffsetalarm - settings::ldw::kperoffsetwarning));
				} else if ( (-1.0 * settings::ldw::kperoffsetalarm) > deviationper ) {
					processvalues->ldwstatus_ = 5;
					processvalues->ldwpwmvalue_ = 1023;
				}
			} else {
				processvalues->ldwstatus_ = -1;
				processvalues->ldwpwmvalue_ = 0;
			}
			if (processvalues->ldwpwmvalue_ < 0) {
				processvalues->ldwpwmvalue_ = 0;
			} else if (processvalues->ldwpwmvalue_ > 1023) {
				processvalues->ldwpwmvalue_ = 1023;
			}
			//Write new data
			processvalues->SetPolygon(newpolygon);
		} else {
			processvalues->ldwstatus_ = 0;
		}
		
		processorpacer.SetPace();
	}
	
	std::cout << "Exiting image processor thread!" << '\n';

}
