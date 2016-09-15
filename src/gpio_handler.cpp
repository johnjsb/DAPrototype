#include <iostream>
#include <atomic>
#include <stdlib.h>			//For auto shutdown
//#include <wiringPi.h>
//#include <wiringPiI2C.h>
#include "pace_setter_class.h"
#include "process_values_class.h"
#include "xml_reader.h"

#define BUZZERPIN 17
#define POWERINPUTPIN 7
#define POWEROUTPUTPIN 29

void GpioHandlerThread( ProcessValues *processvalues,
						std::atomic<bool> *exitsignal,
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
	int blinkinterval{ settings::comm::pollrategpio/2 };	//500ms
	int buzzercount{0};
	int blinkercount{0};
//	wiringPiSetup();
//	pinMode(POWERINPUTPIN, INPUT); 
//	pinMode(POWEROUTPUTPIN, OUTPUT); 
//	digitalWrite(POWEROUTPUTPIN, 1);	//Switch to delayed power
//	pinMode(BUZZERPIN, OUTPUT);
//	pinMode(LEFTALARMPIN, PWM_OUTPUT);
//	pinMode(LEFTWARNINGPIN, PWM_OUTPUT);
//	pinMode(LEFTOKPIN, PWM_OUTPUT);
//	pinMode(RIGHTALARMPIN, PWM_OUTPUT);
//	pinMode(RIGHTWARNINGPIN, PWM_OUTPUT);
//	pinMode(RIGHTOKPIN, PWM_OUTPUT);
//	pinMode(FORWARDALARMPIN, PWM_OUTPUT);
//	pinMode(FORWARDWARNINGPIN, PWM_OUTPUT);
//	pinMode(FORWARDOKPIN, PWM_OUTPUT);
//	pinMode(CENTERPIN, OUTPUT);
	int inputfailcount{0};
	
	//create pace setter
	PaceSetter gpiopacer(settings::comm::pollrategpio, "GPIO handler");
	
	//Loop indefinitely
//	for(;;) {
    for(int i = 0; i < 2000; i++) {		//For testing
		//Check for Warnings
		if ( (processvalues->ldwstatus_ > 2) || (processvalues->fcwstatus_ > 0) ||
			(processvalues->gpsstatus_ > 3) ) {
			warning = true;
		} else {
			warning = alarm = false;
		}
		//Check for Alarms
		if ( (processvalues->ldwstatus_ > 4) || (processvalues->fcwstatus_ > 2) ||
			(processvalues->gpsstatus_ > 4) ) {
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
		
		//Set PWM for LDW LED's
		switch (processvalues->ldwstatus_){
			case 1:
/*
				pwmWrite (LEFTALARMPIN, 0);
				pwmWrite (LEFTWARNINGPIN, 0);
				pwmWrite (LEFTOKPIN, processvalues->ldwpwmvalue_);
				pwmWrite (RIGHTALARMPIN, 0);
				pwmWrite (RIGHTWARNINGPIN, 0);
				pwmWrite (RIGHTOKPIN, 0);
*/
				break;
			case 2:
/*
				pwmWrite (LEFTALARMPIN, 0);
				pwmWrite (LEFTWARNINGPIN, 0);
				pwmWrite (LEFTOKPIN, 0);
				pwmWrite (RIGHTALARMPIN, 0);
				pwmWrite (RIGHTWARNINGPIN, 0);
				pwmWrite (RIGHTOKPIN, processvalues->ldwpwmvalue_);
*/
				break;
			case 3:
/*
				pwmWrite (LEFTALARMPIN, 0);
				pwmWrite (LEFTWARNINGPIN, processvalues->ldwpwmvalue_);
				pwmWrite (LEFTOKPIN, 1023);
				pwmWrite (RIGHTALARMPIN, 0);
				pwmWrite (RIGHTWARNINGPIN, 0);
				pwmWrite (RIGHTOKPIN, 0);
*/
				break;
			case 4:
/*
				pwmWrite (LEFTALARMPIN, 0);
				pwmWrite (LEFTWARNINGPIN, 0);
				pwmWrite (LEFTOKPIN, 0);
				pwmWrite (RIGHTALARMPIN, 0);
				pwmWrite (RIGHTWARNINGPIN, processvalues->ldwpwmvalue_);
				pwmWrite (RIGHTOKPIN, 1023);
*/
				break;
			case 5:
/*
				pwmWrite (LEFTALARMPIN, processvalues->ldwpwmvalue_);
				pwmWrite (LEFTWARNINGPIN, 1023);
				pwmWrite (LEFTOKPIN, 1023);
				pwmWrite (RIGHTALARMPIN, 0);
				pwmWrite (RIGHTWARNINGPIN, 0);
				pwmWrite (RIGHTOKPIN, 0);
*/
				break;
			case 6:
/*
				pwmWrite (LEFTALARMPIN, 0);
				pwmWrite (LEFTWARNINGPIN, 0);
				pwmWrite (LEFTOKPIN, 0);
				pwmWrite (RIGHTALARMPIN, processvalues->ldwpwmvalue_);
				pwmWrite (RIGHTWARNINGPIN, 1023);
				pwmWrite (RIGHTOKPIN, 1023);
*/
				break;
			default:
/*
				pwmWrite (LEFTALARMPIN, 0);
				pwmWrite (LEFTWARNINGPIN, 0);
				pwmWrite (LEFTOKPIN, 0);
				pwmWrite (RIGHTALARMPIN, 0);
				pwmWrite (RIGHTWARNINGPIN, 0);
				pwmWrite (RIGHTOKPIN, 0);
*/
				break;
		}
		
		//Set PWM for FCW LED's
		switch ( processvalues->fcwstatus_ ) {
			case 0:
//				pwmWrite (FORWARDALARMPIN, 0);
//				pwmWrite (FORWARDWARNINGPIN, 0);
//				pwmWrite (FORWARDOKPIN, processvalues->fcwpwmvalue_);
				break;
			case 1:
//				pwmWrite (FORWARDALARMPIN, 0);
//				pwmWrite (FORWARDWARNINGPIN, processvalues->fcwpwmvalue_);
//				pwmWrite (FORWARDOKPIN, 1023);
				break;
			case 2:
//				pwmWrite (FORWARDALARMPIN, 0);
//				pwmWrite (FORWARDWARNINGPIN, processvalues->fcwpwmvalue_);
//				pwmWrite (FORWARDOKPIN, 1023);
				break;
			case 3:
//				pwmWrite (FORWARDALARMPIN, processvalues->fcwpwmvalue_);
//				pwmWrite (FORWARDWARNINGPIN, 1023);
//				pwmWrite (FORWARDOKPIN, 1023);
				break;
			case 4:
//				pwmWrite (FORWARDALARMPIN, processvalues->fcwpwmvalue_);
//				pwmWrite (FORWARDWARNINGPIN, 1023);
//				pwmWrite (FORWARDOKPIN, 1023);
				break;
			default:
//				pwmWrite (FORWARDALARMPIN, 0);
//				pwmWrite (FORWARDWARNINGPIN, 0);
//				pwmWrite (FORWARDOKPIN, 0);
				break;
		}
		
		//Set center LED
		if ( (processvalues->ldwstatus_ >= 0) && (processvalues->fcwstatus_ >= 0) &&
			(processvalues->gpsstatus_ > 1) ) {
//				digitalWrite(CENTERPIN, 1);
		} else if ( (processvalues->ldwstatus_ >= 0) && (processvalues->fcwstatus_ >= 0) &&
			(processvalues->gpsstatus_ == 0) ) {
			blinkercount++;
			if ( blinkercount % blinkinterval != 0) continue;
/*
			if (digitalRead(CENTERPIN) {
				digitalWrite(CENTERPIN, 1);
			} else {
				digitalWrite(CENTERPIN, 0);
			}
*/
		} else {
			//digitalWrite(CENTERPIN, 0);
		}
		
/*
		//Shutdown logic on power loss
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
//	digitalWrite(POWEROUTPUTPIN, 0);	//Kill power to RPi

//	system ("sudo shutdown -h now");	//Shutdown RPi
	
	std::cout << "Exiting GPIO handler thread!" << std::endl;

}