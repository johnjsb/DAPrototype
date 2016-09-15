#include <iostream>
#include <libgpsmm.h>
#include <atomic>
#include "pace_setter_class.h"
#include "process_values_class.h"
#include "xml_reader.h"

void GpsPollingThread( ProcessValues *processvalues,
					   std::atomic<bool> *exitsignal)
{

	std::cout << "GPS polling thread starting!" << std::endl;

	//Create thread variables
	gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);

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
				alarmmonitor::latitude = newdata->fix.latitude;
				alarmmonitor::longitude = newdata->fix.longitude;
				alarmmonitor::gpsspeed = newdata->fix.speed;
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