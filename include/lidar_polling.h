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
#ifndef LIDAR_POLLING_H_INCLUDED
#define LIDAR_POLLING_H_INCLUDED

//Standard libraries
#include <atomic>

//Project libraries
#include "process_values_class.h"

/*****************************************************************************************/
void LidarPolingThread( ProcessValues *processvalues,
						std::atomic<bool> *exitsignal );

#endif // LIDAR_POLLING_H_INCLUDED
