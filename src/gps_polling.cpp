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
#include <errno.h>
#include <sys/time.h>
#include <exception>
#include <string>
#include <thread>

//3rd party libraries
#include <libgpsmm.h>

//Project libraries
#include "pace_setter_class.h"
#include "process_values_class.h"
#include "xml_reader.h"
#include "gps_polling.h"

//Preprocessor
#define MPSTOMPHCONVERSION 2.237
#define RETRIES 3						//gpsmm.read() consistently fails twice after boot

/*****************************************************************************************/
void GpsPollingThread( ProcessValues *processvalues,
					   std::atomic<bool> *exitsignal )
{

	std::cout << "GPS polling thread starting!" << '\n';

	//Create thread variables
	processvalues->gpsstatus_ = GPS_NO_LOCK;
	bool timeset{ false };
	int maxwaittime{ 3000000 / settings::comm::kpollrategps };
	gpsmm gpsrecv("localhost", DEFAULT_GPSD_PORT);
	struct gps_data_t* gpsdata{ gpsrecv.read() };
	int tries{ 0 };
	while ( !gpsdata ) {
		gpsdata = gpsrecv.read();
		std::this_thread::sleep_for( std::chrono::seconds(2) );
		if ( tries > RETRIES ) {
			std::cout << "Failed to get valid gps read in " << RETRIES <<
						 " attempts "<< '\n';
			std::cout << "Exiting GPS polling thread!" << '\n';
			return;
		}
		tries++;
	}
	
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

	//Loop indefinitely
	while( !(*exitsignal) ) {
		try {
			//Get data
			gpsdata = gpsrecv.read();
			
			//Set time - commented out due to crashing after boot
			/*
			 * if ( !timeset ) timeset = SetTime(gpsdata);
			 */
			//Evaluate
			if ( !gpsrecv.waiting(maxwaittime) ) {
				processvalues->gpsstatus_ = GPS_ERROR;
				std::cout << "GPS timeout." << '\n';
			} else if ( gpsdata == NULL ) {
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

			//Set Pace
			gpspacer.SetPace();
		} catch ( const std::exception& ex ) {
			std::cout << "GPS Polling thread threw exception: "<< ex.what() << '\n';
		} catch ( const std::string& str ) {
			std::cout << "GPS Polling thread threw exception: "<< str << '\n';
		} catch (...) {
			std::cout << "GPS Polling thread threw exception of unknown type!" << '\n';
		}
	}
	
	std::cout << "Exiting GPS polling thread!" << '\n';
	return;
}

/*****************************************************************************************/
bool SetTime( struct gps_data_t* data )
{
	try {
		if ( (data == NULL) ||
			 (data->fix.mode <= 1) ||
			 (data->fix.time < 1) ||
			 std::isnan(data->fix.time) ) return false;

		//Convert gps_data_t* member 'time' to timeval
		double offsettime{ data->fix.time - (5.0 * 3600.0) }; 	//5.0 hr offset for EST
		double seconds{ 0.0 };
		double microseconds{ 1000000.0 * modf(offsettime, &seconds) };
		const timeval tv{ static_cast<time_t>(seconds),
						  static_cast<suseconds_t>(microseconds) };

		//Set system time - THIS IS CAUSING CRASHES, WHY?
		if ( settimeofday(&tv, NULL) >= 0) {
			std::cout << "Time set successful!" << '\n';
			return true;
		} else {
			std::cout << "Time set failure!" << '\n';
			return false;
		}
	} catch ( const std::exception& ex ) {
		std::cout << "GPS time setting threw exception: "<< ex.what() << '\n';
	} catch ( const std::string& str ) {
		std::cout << "GPS time setting threw exception: "<< str << '\n';
	} catch (...) {
		std::cout << "GPS time setting threw exception of unknown type!" << '\n';
	}
	
	return false;
}

