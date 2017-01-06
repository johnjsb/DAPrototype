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
#include <thread>
#include <float.h>
#include <exception>
#include <string>

//3rd party libraries
#include <wiringPi.h>
#include <wiringPiI2C.h>

//Project libraries
#include "pace_setter_class.h"
#include "process_values_class.h"
#include "fcw_tracker_class.h"
#include "xml_reader.h"

//Preprocessor
#define FEETPERCENTIMETER 0.0328084f

extern "C" {
	#include "lidarLite.h"
}

/*****************************************************************************************/
int LidarPollingSetup()
{
	try {
		//Setup I2C
		int dacmodule { wiringPiI2CSetup(0x62) };
		if ( dacmodule < 0 ) std::cout << "I2C Setup Error" << '\n';
		return dacmodule;
	} catch ( const std::exception& ex ) {
		std::cout << "Lidar polling setup threw exception: "<< ex.what() << '\n';
		return -1;
	} catch ( const std::string& str ) {
		std::cout << "Lidar polling setup threw exception: "<< str << '\n';
		return -2;
	} catch (...) {
		std::cout << "Lidar polling setup threw exception of unknown type!" << '\n';
		return -3;
	}
}

void LidarPolling( ProcessValues& processvalues,
				   int dacmodule,
				   int pullaheadcount,
				   FcwTracker* fcwtracker )
{
	try {
		//Create thread variables
		bool vehiclemoving{ false };
		int pullaheaddelay{ settings::comm::kpollratelidar / 2 };	//500ms
		int timeoutdelay{ settings::comm::kpollratelidar / 2 };		//500ms
		int timeoutcount{ 0 };

		//Read FCW distance
		int fcwresult{ -1 };
		bool readerror{ true };
		fcwresult = lidar_read(dacmodule);
		if ( fcwresult == USHRT_MAX ) {
			readerror = true;
		} else {
			if ( (FEETPERCENTIMETER * fcwresult) >
				 settings::fcw::kdistanceoffset ) {
				fcwtracker->Update( FEETPERCENTIMETER * fcwresult,
								   processvalues.gpsspeed_ );
				timeoutcount = 0;
				readerror = false;
			} else {
				timeoutcount++;
				if ( timeoutcount <= timeoutdelay ) {
					readerror = false;
				}
			}

			//Check if vehicle is moving
			if ( processvalues.gpsspeed_ > 1.0 ) {
				vehiclemoving = true;

			} else {
				vehiclemoving = false;
			}
		}

		//Update everything
		processvalues.forwarddistance_ = fcwtracker->followingdistance_;
		//if ( fcwtracker->followingtime_ < fcwtracker->timetocollision_ ) {
			processvalues.timetocollision_ = fcwtracker->followingtime_;
		//} else {
		//	processvalues.timetocollision_ = fcwtracker->timetocollision_;			
		//}

		//FCW Status
		if ( readerror ) {
			processvalues.fcwstatus_ = FCW_ERROR;
		/*
		} else if ( (1000 * fcwtracker->timetocollision_) <
					settings::fcw::kmscollisionwarning ) {
			processvalues.fcwstatus_ = FCW_WARNING;
		*/
		} else if ( vehiclemoving &&
					((1000 * fcwtracker->followingtime_) <
					settings::fcw::kmsfollowdistwarning) &&
					((1000 * fcwtracker->followingtime_) >
					settings::fcw::kmsfollowdistalarm) ) {
			processvalues.fcwstatus_ = FCW_TAILGATE_WARNING;
		/*
		} else if ( (1000 * fcwtracker->timetocollision_) <
					settings::fcw::kmscollisionalarm ) {
			processvalues.fcwstatus_ = FCW_ALARM;
		*/
		} else if ( vehiclemoving &&
					(1000 * fcwtracker->followingtime_) <
					settings::fcw::kmsfollowdistalarm ) {
			processvalues.fcwstatus_ = FCW_TAILGATE_ALARM;
		} else {
			processvalues.fcwstatus_ = FCW_ACTIVE;
		}
/*
	//Check for driver pullahead
	if ( !vehiclemoving &&
		 (fcwtracker->acceleration_ > 0.1) &&
		 (pullaheadcount > pullaheaddelay) ) {
		processvalues.fcwstatus_ = FCW_PULL_AHEAD_WARNING;
	} else if ( !vehiclemoving && (fcwtracker->acceleration_ > 0.1) ) {
		pullaheadcount++;
	} else {
		pullaheadcount = 0;
	}
*/

	} catch ( const std::exception& ex ) {
		std::cout << "Lidar Polling thread threw exception: "<< ex.what() << '\n';
	} catch ( const std::string& str ) {
		std::cout << "Lidar Polling thread threw exception: "<< str << '\n';
	} catch (...) {
		std::cout << "Lidar Polling thread threw exception of unknown type!" << '\n';
	}

	return;
}
