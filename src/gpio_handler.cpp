#include <iostream>
#include <atomic>
#include <stdlib.h>			//For auto shutdown
//#include <wiringPi.h>
//#include <wiringPiI2C.h>
#include "pace_setter_class.h"
#include "xml_reader.h"
#include "alarm_monitor.h"

#define BUZZERPIN 17
#define POWERINPUTPIN 7

void GpioHandlerThread( std::atomic<bool> *exitsignal,
                        std::atomic<bool> *shutdownsignal )
{

	std::cout << "GPIO handler thread starting!" << std::endl;
	
	//Check if enabled
	if ( !settings::gpio::enabled ) {
		std::cout << "GPIO disabled, exiting!" << std::endl;
		return;
	}

	//Create thread variables
	bool warning{false};
	bool alarm{false};
	int buzzerinterval{ settings::comm::pollrategpio/2 };	//500ms
	int buzzercount{0};
//    wiringPiSetup();
//	  pinMode(POWERINPUTPIN, INPUT); 
//	  pinMode(BUZZERPIN, OUTPUT); 
	int inputfailcount{0};
	
	//create pace setter
	PaceSetter gpiopacer(settings::comm::pollrategpio, "GPIO handler");
	
	//Loop indefinitely
	//for(;;) {
    for(int i = 0; i < 2000; i++) {		//For testing
		//Check for Warnings
		if ( (alarmdata::ldwstatus > 2) || (alarmdata::fcwstatus > 0) ||
			(alarmdata::gpsstatus > 3) ) {
			warning = true;
		} else {
			warning = alarm = false;
		}
		//Check for Alarms
		if ( (alarmdata::ldwstatus > 4) || (alarmdata::fcwstatus > 2) ||
			(alarmdata::gpsstatus > 4) ) {
			alarm = true;
		}
		
		//Set buzzer
		if ( alarm && settings::gen::enbuzzer ) {
			//digitalWrite(BUZZERPIN, 1);
		} else if ( !alarm && warning && settings::gen::enbuzzer ) {
			buzzercount++;
			if ( buzzercount % buzzerinterval != 0) continue;
			/*
			if (digitalRead(BUZZERPIN) {
				digitalWrite(BUZZERPIN, 1);
			} else {
				digitalWrite(BUZZERPIN, 0);
			}
			*/
		} else {
			//digitalWrite(BUZZERPIN, 0);
			buzzercount = buzzerinterval - 1;
		}
	/*
		if ( digitalRead(POWERINPUTPIN) || !settings::gpio::autoshutdown ) {
			inputfailcount = 0;
		} else if (!digitalRead(POWERINPUTPIN) && (inputfailcount < 3)) {	//3 exits!
			inputfailcount++;
		} else {
			break;				//This exits and shuts down the Pi
		}
	*/
	
        gpiopacer.SetPace();
    }
	
	*exitsignal = true;
	
	while( !(*shutdownsignal) ) {
		//Just wait for video writer thread to exit
	}

	system ("sudo shutdown -h now");
	
	std::cout << "Exiting GPIO handler thread!" << std::endl;

}