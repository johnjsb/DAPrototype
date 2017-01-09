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
#include "process_values_class.h"
#include "xml_reader.h"
#include "gps_polling.h"

//Preprocessor
#define MPSTOMPHCONVERSION 2.237
#define RETRIES 3						//gpsmm.read() consistently fails twice after boot

/*****************************************************************************************/
bool GpsPollingSetup( gpsmm* gpsrecv )
{
	//Check that gpsd service is running
	if ( gpsrecv->stream(WATCH_ENABLE|WATCH_JSON) == NULL ) {
		std::cout << "No GPSD running. GPS setup failed." << '\n';
		return false;
	}

	//Get first reading to set time
	struct gps_data_t* gpsdata{ gpsrecv->read() };
	int tries{ 0 };
	while ( !gpsdata ) {
		gpsdata = gpsrecv->read();
		std::this_thread::sleep_for( std::chrono::seconds(2) );
		if ( tries > RETRIES ) {
			std::cout << "Failed to get valid gps read in " << RETRIES <<
						 " attempts "<< '\n';
			return false;
		}
		tries++;
	}

	//Set baud rate 115200
	if ( gps_send(gpsdata,"$PMTK251,115200*1F\r\n") >= 0 ) {
		std::cout << "GPS baud rate set to 115200" << '\n';
	} else {
		std::cout << "GPS baud rate setting failed!" << '\n';
	}

	//Update every 100 ms
	if ( gps_send(gpsdata,"$PMTK220,100*2F\r\n") >= 0 ) {
		std::cout << "GPS update rate set to 10hz" << '\n';
	} else {
		std::cout << "GPS update rate setting failed!" << '\n';
	}

	//Measure every 200 ms
	if ( gps_send(gpsdata,"$PMTK300,200,0,0,0,0*2F\r\n") >= 0 ) {
		std::cout << "GPS measure rate set to 5hz" << '\n';
	} else {
		std::cout << "GPS measure rate setting failed!" << '\n';
	}

	//Set speed threshold @ 2.0 m/s
	if ( gps_send(gpsdata,"$PMTK397,2.0*3F\r\n") >= 0 ) {
		std::cout << "GPS speed threshold set to 2.0 m/s" << '\n';
	} else {
		std::cout << "GPS speed threshold setting failed!" << '\n';
	}

	return true;
}

/*****************************************************************************************/
void GpsPolling( ProcessValues& processvalues,
				 gpsmm* gpsrecv,
				 bool& timeset )
{
	//static bool timeset{ false };
	try {
		//Get data
		struct gps_data_t* gpsdata{ gpsrecv->read() };

		//Set time
		if ( !timeset ) timeset = SetTime(gpsdata);
		
		//Evaluate
		if ( !gpsrecv->waiting(2000000) ) {
			processvalues.gpsstatus_ = GPS_ERROR;
			std::cout << "GPS timeout." << '\n';
		} else if ( gpsdata == NULL ) {
			processvalues.gpsstatus_ = GPS_ERROR;
			std::cout << "GPS read error!" << '\n';
		} else {
			if ( gpsdata->fix.mode > 1) {
				//Write values
				processvalues.latitude_ = gpsdata->fix.latitude;
				processvalues.longitude_ = gpsdata->fix.longitude;
				processvalues.gpsspeed_ = MPSTOMPHCONVERSION * gpsdata->fix.speed;
				if ( processvalues.gpsspeed_ > settings::ldw::kenablespeed ) {
					processvalues.gpsstatus_ =  GPS_LOCK_LDW_ON;
				} else {
					processvalues.gpsstatus_ =  GPS_LOCK_LDW_OFF;
				}

			} else {
				processvalues.gpsstatus_ = GPS_NO_LOCK;
			}
		}
	} catch ( const std::exception& ex ) {
		std::cout << "GPS polling threw exception: "<< ex.what() << '\n';
	} catch ( const std::string& str ) {
		std::cout << "GPS polling threw exception: "<< str << '\n';
	} catch (...) {
		std::cout << "GPS polling threw exception of unknown type!" << '\n';
	}

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
