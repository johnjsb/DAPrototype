#include <iostream>
#include <atomic>
#include <deque>
#include <sys/time.h>
#include "pace_setter_class.h"
#include "process_values_class.h"
#include "xml_reader.h"
#include "gps_polling.h"

#ifdef __arm__									//Detect if compiling for raspberry pi
	#include <libgpsmm.h>
	#include "gps.h"
#endif

#define MPSTOMPHCONVERSION 2.237

void GpsPollingThread( ProcessValues *processvalues,
					   std::atomic<bool> *exitsignal )
{

	std::cout << "GPS polling thread starting!" << std::endl;
#ifdef __arm__									//Detect if compiling for raspberry pi
	//Create thread variables
	gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);
	std::deque<double> latitudevalues;
	std::deque<double> longitudevalues;
	std::deque<double> speedvalues;

    if (gps_rec.stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        std::cout << "No GPSD running. exiting GPS thread." << std::endl;
        return;
    }
    
    //Set poll rate 10hz
    gps_rec.send("$PMTK300,200,0,0,0,0*2F");

	//Get first data to set system time
	struct gps_data_t* firstdata;
	
	//Loop until first GPS lock to set system time
	while ((firstdata = gps_rec.read()) == NULL) {
		if (*exitsignal) {
			return;
		}	  
	}
	
	//After good read, set date/time
	if (gps_rec.read()) != NULL) {
		
	}
	
	//Convert gps_data_t* member 'time' to timeval
	timeval tv;
	double wholeseconds, decimalseconds;
	decimalseconds = modf(firstdata->time, &wholeseconds);
	tv.sec = static_cast<int32_t>(wholeseconds);
	tv.usec = static_cast<int32_t>(decimalseconds * 1000000.0);

	//Create timezone
	timezone tz{ timezone(300, DST_USA) };

	//Set system time
	settimeofday(&tv, &tz);
	
	//create pace setter
	PaceSetter gpspacer(settings::comm::kpollrategps, "GPS polling");
	
	//Loop indefinitely
	while( !(*exitsignal) ) {
		struct gps_data_t* newdata;

		if (!gps_rec.waiting(5000000)) {
			processvalues->gpsstatus_ = -1;
			std::cout << "GPS timeout." << std::cout;
			continue;
		}

		if ((newdata = gps_rec.read()) == NULL) {
			processvalues->gpsstatus_ = -1;
			std::cout << "GPS read error!" << std::cout;
			continue;
		} else {
			if ( newdata->fix.mode > 1) {
				
				//Write values
				processvalues->latitude_ = Average(newdata->fix.latitude,
					latitudevalues, settings::gps::ksamplestoaverage);
				processvalues->longitude_ = Average(newdata->fix.longitude,
					longitudevalues, settings::gps::ksamplestoaverage);
				processvalues->gpsspeed_ = MPSTOMPHCONVERSION * newdata->fix.speed;
				/*
				processvalues->gpsspeed_ = MPSTOMPHCONVERSION * Average(newdata->fix.speed,
					speedvalues, settings::gps::ksamplestoaverage);
				*/
				if ( processvalues->gpsspeed_ > settings::ldw::kenablespeed ) {
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
#else
	std::cout << "Hardware doesn't support GPS!" << std::endl;
#endif
	
	std::cout << "Exiting GPS polling thread!" << std::endl;
	return;

}

double Average ( double value,
			     std::deque<double> &values,
			     int tokeep )
{
	values.push_back(value);
	if ( values.size() > tokeep ) {
		values.pop_front();
		for ( int i = 1; i < values.size(); i++ ) {
			value += values[i];
		}
		value /= values.size();
	}
	return value;
}
