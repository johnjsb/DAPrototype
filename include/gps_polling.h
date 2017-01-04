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
#ifndef GPS_POLLING_H_INCLUDED
#define GPS_POLLING_H_INCLUDED

//Standard libraries
#include <atomic>

//Project libraries
#include "process_values_class.h"

/*****************************************************************************************/
gpsmm GpsPollingSetup();
void GpsPolling( ProcessValues& processvalues, gpsmm* gps_rec );
bool SetTime( struct gps_data_t* data );

#endif // GPS_POLLING_H_INCLUDED
