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
#include <iostream>
#include <atomic>
#include <deque>
#include <cmath>
#include <sys/time.h>

//3rd party libraries
#ifdef __arm__									//Detect if compiling for raspberry pi
	#include <libgpsmm.h>
#endif

//Project libraries
#include "pace_setter_class.h"
#include "process_values_class.h"
#include "xml_reader.h"
#include "gps_polling.h"

//Preprocessor
#define MPSTOMPHCONVERSION 2.237

/*****************************************************************************************/
void GpsPollingThread( ProcessValues *processvalues,
					   std::atomic<bool> *exitsignal )
{

	std::cout << "GPS polling thread starting!" << '\n';
#ifdef __arm__									//Detect if compiling for raspberry pi
	//Create thread variables
	gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);
	processvalues->gpsstatus_ = GPS_NO_LOCK;
	
	//Check that gpsd service is running
    if (gps_rec.stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        std::cout << "No GPSD running. exiting GPS thread." << '\n';
        return;
    }

    //Get first reading to set time
	struct gps_data_t* gpsdata{ gps_rec.read() };
	
	//Set baud rate 115200
	if (gps_send(gpsdata,"$PMTK251,115200*1F\r\n") >= 0) {
		std::cout << "GPS baud rate set to 115200" << '\n';
	} else {
		std::cout << "GPS baud rate setting failed!" << '\n';
	}
	
	//Update every 100 ms
	if (gps_send(gpsdata,"$PMTK220,100*2F\r\n") >= 0) {
		std::cout << "GPS update rate set to 10hz" << '\n';
	} else {
		std::cout << "GPS update rate setting failed!" << '\n';
	}
	
	//Measure every 200 ms
	if (gps_send(gpsdata,"$PMTK300,200,0,0,0,0*2F\r\n") >= 0) {
		std::cout << "GPS measure rate set to 5hz" << '\n';
	} else {
		std::cout << "GPS measure rate setting failed!" << '\n';
	}
	
	//Set speed threshold @ 2.0 m/s
	if (gps_send(gpsdata,"$PMTK397,2.0*3F\r\n") >= 0) {
		std::cout << "GPS speed threshold set to 2.0 m/s" << '\n';
	} else {
		std::cout << "GPS speed threshold setting failed!" << '\n';
	}

	//create pace setter
	PaceSetter gpspacer(settings::comm::kpollrategps, "GPS polling");

	//Loop until first GPS lock to set system time
	while ( (gpsdata == NULL) ||
			(gpsdata->fix.mode <= 1) ||
			(gpsdata->fix.time < 1) ||
			std::isnan(gpsdata->fix.time) ) {
		if (*exitsignal) {
			return;
		}
		gpspacer.SetPace();
		gpsdata = gps_rec.read();
	}

	//Convert gps_data_t* member 'time' to timeval
	timeval tv;
	double wholeseconds, decimalseconds, offsettime;
	offsettime = gpsdata->fix.time - (5.0 * 3600.0);
	decimalseconds = modf(offsettime, &wholeseconds);
	tv.tv_sec = static_cast<int32_t>(wholeseconds);
	tv.tv_usec = static_cast<int32_t>(decimalseconds * 1000000.0);

	//Set system time - THIS IS CAUSING CRASHES, WHY?
	if ( settimeofday(&tv, NULL) >= 0) {
		std::cout << "Time set succesful!" << '\n';
	} else {
		std::cout << "Time set failure!" << '\n';
	}

	//Loop indefinitely
	while( !(*exitsignal) ) {
		if (!gps_rec.waiting(2000000)) {
			processvalues->gpsstatus_ = GPS_ERROR;
			std::cout << "GPS timeout." << '\n';
		} else if ((gpsdata = gps_rec.read()) == NULL) {
			processvalues->gpsstatus_ = GPS_ERROR;
			std::cout << "GPS read error!" << '\n';
		} else {
			if ( gpsdata->fix.mode > 1) {
				//Write values
				processvalues->latitude_ = gpsdata->fix.latitude;
				processvalues->longitude_ = gpsdata->fix.longitude;
				processvalues->gpsspeed_ = MPSTOMPHCONVERSION * gpsdata->fix.speed;
				if ( processvalues->gpsspeed_ > settings::ldw::kenablespeed ) {
					processvalues->gpsstatus_ =  GPS_LOCK_LDW_ON;
				} else {
					processvalues->gpsstatus_ =  GPS_LOCK_LDW_OFF;
				}
				
			} else {
				processvalues->gpsstatus_ = GPS_NO_LOCK;
			}
		}

		//gpspacer.SetPace();
	}
#else
	std::cout << "Hardware doesn't support GPS!" << '\n';
#endif
	
	std::cout << "Exiting GPS polling thread!" << '\n';
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
