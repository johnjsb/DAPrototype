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
#define POWEROUTPUTPIN 29

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
		
		//Set PWM for LDW LED's
		switch (alarmdata::ldwstatus){
			case 1:
				/*
				pwmWrite (LEFTALARMPIN, 0);
				pwmWrite (LEFTWARNINGPIN, 0);
				pwmWrite (LEFTOKPIN, alarmdata::ldwpwmvalue);
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
				pwmWrite (RIGHTOKPIN, alarmdata::ldwpwmvalue);
				*/
				break;
			case 3:
				/*
				pwmWrite (LEFTALARMPIN, 0);
				pwmWrite (LEFTWARNINGPIN, alarmdata::ldwpwmvalue);
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
				pwmWrite (RIGHTWARNINGPIN, alarmdata::ldwpwmvalue);
				pwmWrite (RIGHTOKPIN, 1023);
				*/
				break;
			case 5:
				/*
				pwmWrite (LEFTALARMPIN, alarmdata::ldwpwmvalue);
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
				pwmWrite (RIGHTALARMPIN, alarmdata::ldwpwmvalue);
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
		switch ( alarmdata::fcwstatus ) {
			case 0:
				//pwmWrite (FORWARDALARMPIN, 0);
				//pwmWrite (FORWARDWARNINGPIN, 0);
				//pwmWrite (FORWARDOKPIN, alarmdata::fcwpwmvalue);
				break;
			case 1:
				//pwmWrite (FORWARDALARMPIN, 0);
				//pwmWrite (FORWARDWARNINGPIN, alarmdata::fcwpwmvalue);
				//pwmWrite (FORWARDOKPIN, 1023);
				break;
			case 2:
				//pwmWrite (FORWARDALARMPIN, 0);
				//pwmWrite (FORWARDWARNINGPIN, alarmdata::fcwpwmvalue);
				//pwmWrite (FORWARDOKPIN, 1023);
				break;
			case 3:
				//pwmWrite (FORWARDALARMPIN, alarmdata::fcwpwmvalue);
				//pwmWrite (FORWARDWARNINGPIN, 1023);
				//pwmWrite (FORWARDOKPIN, 1023);
				break;
			case 4:
				//pwmWrite (FORWARDALARMPIN, alarmdata::fcwpwmvalue);
				//pwmWrite (FORWARDWARNINGPIN, 1023);
				//pwmWrite (FORWARDOKPIN, 1023);
				break;
			default:
				//pwmWrite (FORWARDALARMPIN, 0);
				//pwmWrite (FORWARDWARNINGPIN, 0);
				//pwmWrite (FORWARDOKPIN, 0);
				break;
		}
		
		//Set center LED
		if ( (alarmdata::ldwstatus >= 0) && (alarmdata::fcwstatus >= 0) &&
			(alarmdata::gpsstatus > 1) ) {
			//digitalWrite(CENTERPIN, 1);
		} else if ( (alarmdata::ldwstatus >= 0) && (alarmdata::fcwstatus >= 0) &&
			(alarmdata::gpsstatus == 0) ) {
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

	system ("sudo shutdown -h now");
	
	std::cout << "Exiting GPIO handler thread!" << std::endl;

}