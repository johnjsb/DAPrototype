#include <iostream>
#include <libgpsmm.h>
#include <atomic>
#include <deque>
#include "pace_setter_class.h"
#include "process_values_class.h"
#include "xml_reader.h"

void GpsPollingThread( ProcessValues *processvalues,
					   std::atomic<bool> *exitsignal)
{

	std::cout << "GPS polling thread starting!" << std::endl;

	//Create thread variables
	gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);
	std::deque<double> latitudevalues;
	std::deque<double> longitudevalues;
	std::deque<double> speedvalues;

    if (gps_rec.stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        std::cout << "No GPSD running. exiting GPS thread." << std::endl;
        return;
    }
	
	//create pace setter
	PaceSetter gpspacer(settings::comm::kpollrategps, "GPS polling");
	
	//Loop indefinitely
	while( !(*exitsignal) ) {
		struct gps_data_t* newdata;

		//if (!gps_rec.waiting(5000000)) {
		//	processvalues->gpsstatus_ = -1;
		//	std::cout << "GPS timeout." std::cout;
		//	continue;
		//}

		if ((newdata = gps_rec.read()) == NULL) {
			processvalues->gpsstatus_ = -1;
			std::cout << "GPS read error!" std::cout;
			continue;
		} else {
			if ( newdata->fix.mode > 1) {
				
				//Write values
				processvalues->latitude_ = Average(newdata->fix.latitude,
					latitudevalues, settings::gps::ksamplestoaverage);
				processvalues->longitude_ = Average(newdata->fix.longitude,
					longitudevalues, settings::gps::ksamplestoaverage);
				processvalues->gpsspeed_ = Average(newdata->fix.speed,
					speedvalues, settings::gps::ksamplestoaverage);
				if ( newdata->fix.speed > settings::ldw::kenablespeed ) {
					processvalues->gpsstatus_ =  3;
				} else {
					processvalues->gpsstatus_ =  2;
				}
				
			} else {
				processvalues->gpsstatus_ = 1;
				//std::cout << "No GPS fix." << std::endl;
			}
		}
		
		//ToDo - Future implementation of speeding notification

		gpspacer.SetPace();
	}
	
	std::cout << "Exiting GPS polling thread!" << std::endl;

}

double Average ( double value,
			     std::deque<double> &values,
			     int tokeep )
{
	values.push_back(value);
	if ( values.size() > tokeep ) {
		values.pop_front();
		for ( i = 1; i < values.size(); i++ ) {
			value += values[i];
		}
		value /= values.size();
	}
	return;
}