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
#ifndef GPIO_HANDLER_H_INCLUDED
#define GPIO_HANDLER_H_INCLUDED

//Standard libraries
#include <atomic>

//Project libraries
#include "process_values_class.h"

/*****************************************************************************************/
bool GpioHandlerSetup();
void GpioHandler( ProcessValues& processvalues,
				  std::atomic<bool>& exitsignal );

#endif // GPIO_HANDLER_H_INCLUDED
