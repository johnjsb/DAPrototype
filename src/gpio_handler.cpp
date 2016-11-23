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
#include <stdlib.h>								//For auto shutdown

//3rd party libraries
#ifdef __arm__									//Detect if compiling for raspberry pi
	#include <wiringPi.h>						//For Raspberry Pi
	#include <wiringPiI2C.h>
#endif

//Project libraries
#include "pace_setter_class.h"
#include "process_values_class.h"
#include "xml_reader.h"

//Preprocessor

//Physical pin numbers
#define BUZZERPIN 27
#define POWERINPUTPIN 4
#define POWEROUTPUTPIN 5
#define LEFTALARMPIN 17
#define LEFTWARNINGPIN 13
#define LEFTOKPIN 12
#define RIGHTALARMPIN 23
#define RIGHTWARNINGPIN 22
#define RIGHTOKPIN 18
#define FORWARDALARMPIN 26
#define FORWARDWARNINGPIN 25
#define FORWARDOKPIN 24
#define CENTERPIN 37

/*****************************************************************************************/
void GpioHandlerThread( ProcessValues *processvalues,
						std::atomic<bool> *exitsignal,
						std::atomic<bool> *shutdownsignal )
{

	std::cout << "GPIO handler thread starting!" << '\n';
	
	//Check if enabled
	if ( !settings::gpio::kenabled ) {
		std::cout << "GPIO disabled, exiting!" << '\n';
		return;
	}

	//Create thread variables
	bool warning{ false };
	bool alarm{ false };
	int buzzerinterval{  settings::comm::kpollrategpio / 2  };	//500ms
	int blinkinterval{  settings::comm::kpollrategpio / 2  };	//500ms
	int buzzercount{ 0 };
	int blinkercount{ 0 };
	#ifdef __arm__									//Detect if compiling for raspberry pi
	wiringPiSetupGpio();
	pinMode(POWERINPUTPIN, INPUT); 
	pinMode(POWEROUTPUTPIN, OUTPUT); 
	digitalWrite(POWEROUTPUTPIN, 1);				//Switch to delayed power
	pinMode(BUZZERPIN, OUTPUT);
	pinMode(LEFTALARMPIN, PWM_OUTPUT);
	pinMode(LEFTWARNINGPIN, PWM_OUTPUT);
	pinMode(LEFTOKPIN, PWM_OUTPUT);
	pinMode(RIGHTALARMPIN, PWM_OUTPUT);
	pinMode(RIGHTWARNINGPIN, PWM_OUTPUT);
	pinMode(RIGHTOKPIN, PWM_OUTPUT);
	pinMode(FORWARDALARMPIN, PWM_OUTPUT);
	pinMode(FORWARDWARNINGPIN, PWM_OUTPUT);
	pinMode(FORWARDOKPIN, PWM_OUTPUT);
	pinMode(CENTERPIN, OUTPUT);
	#endif
	int inputfailcount{ 0 };
	
	//create pace setter
	PaceSetter gpiopacer(settings::comm::kpollrategpio, "GPIO handler");
	
	//Loop indefinitely
	for(;;) {
		//Check for Warnings
		if ( (processvalues->ldwstatus_ > LDW_RIGHT_DEVIATION_OK) ||
			 (processvalues->fcwstatus_ > FCW_ACTIVE) ||
			 (processvalues->gpsstatus_ > GPS_LOCK_LDW_ON) ) {
			warning = true;
		} else {
			warning = alarm = false;
		}
		//Check for Alarms
		if ( (processvalues->ldwstatus_ > LDW_RIGHT_DEVIATION_WARNING) ||
			 (processvalues->fcwstatus_ > FCW_TAILGATE_WARNING) ||
			 (processvalues->gpsstatus_ > GPS_SPEED_WARNING) ) {
			alarm = true;
		}
				
		#ifdef __arm__							//Detect if compiling for raspberry pi

			//Shutdown logic on power loss
			if ( !digitalRead(POWERINPUTPIN) && settings::gpio::kautoshutdown ) {
				std::cout << "Power loss detected, exiting!" << '\n';
				break;
			}
			
			//Set buzzer
			if ( alarm && settings::gen::kenbuzzer ) {
				digitalWrite(BUZZERPIN, 1);
			} else if ( !alarm && warning && settings::gen::kenbuzzer ) {
				buzzercount++;
				if ( buzzercount % buzzerinterval != 0) continue;
				if (digitalRead(BUZZERPIN)) {
				digitalWrite(BUZZERPIN, 1);
				} else {
					digitalWrite(BUZZERPIN, 0);
				}
			} else {
				digitalWrite(BUZZERPIN, 0);
				buzzercount = buzzerinterval - 1;
			}

			//Set center LED
			if ( (processvalues->ldwstatus_ >= LDW_INACTIVE) &&
				 (processvalues->fcwstatus_ >= FCW_ACTIVE) &&
				 (processvalues->gpsstatus_ > GPS_NO_LOCK) ) {
					digitalWrite(CENTERPIN, 1);
			} else if ( (processvalues->ldwstatus_ >= LDW_INACTIVE) &&
						(processvalues->fcwstatus_ >= FCW_ACTIVE) &&
						(processvalues->gpsstatus_ == GPS_INACTIVE) ) {
				blinkercount++;
				if ( blinkercount % blinkinterval != 0) continue;
					if (digitalRead(CENTERPIN)) {
					digitalWrite(CENTERPIN, 1);
				} else {
					digitalWrite(CENTERPIN, 0);
				}
				} else {
				digitalWrite(CENTERPIN, 0);
			}
		#endif
	
        gpiopacer.SetPace();
    }
	
	*exitsignal = true;
	
	while( !(*shutdownsignal) ) {
		//Just wait for video writer thread to exit
	}
	#ifdef __arm__									//Detect if compiling for raspberry pi
	digitalWrite(POWEROUTPUTPIN, 0);				//Kill power to RPi

//	system ("sudo shutdown -h now");				//Shutdown RPi
	#endif
	std::cout << "Exiting GPIO handler thread!" << '\n';
	return;

}
