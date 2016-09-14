#include <iostream>
#include <atomic>
//#include <wiringPi.h>
//#include <wiringPiI2C.h>
#include "pace_setter_class.h"
#include "fcw_tracker_class.h"
#include "xml_reader.h"
#include "alarm_monitor.h"
/*
int readInput(int fd, int reg)
{
    wiringPiI2CReadReg8(fd, reg);
    return wiringPiI2CReadReg8(fd, reg);
}
*/
void LidarPolingThread( std::atomic<bool> *exitsignal )
{

	std::cout << "Lidar polling thread starting!" << std::endl;
	
	//Create thread variables
	double followingdistance{0.0};
	bool vehiclemoving{false};
	int pullaheaddelay{ settings::comm::pollratelidar/2 };	//500ms
	int pullaheadcount{ 0 };
	FcwTracker fcwtracker( settings::fcw::distanceoffset,
		settings::fcw::samplestoaverage );
	/*
	//Setup I2C
    wiringPiSetupGpio();
    int dacModule = wiringPiI2CSetup(0x48);
    if (dacModule < 0)
    {
        cout << "I2C Setup Error" << endl;
        return 0;
    }

    int i;
    int A[4]        = {0,       0,      0,      0};
    int A_Reg[4]    = {0x40,    0x41,   0x42,   0x43};
	*/
	
	//create pace setter
	PaceSetter lidarpacer(settings::comm::pollratelidar, "lidar polling");
	
	//Loop indefinitely
	while( !(*exitsignal) ) {
		//Check if vehicle is moving
		if ( alarmdata::gpsspeed > 1.0 ) {
			vehiclemoving = true;
			
		} else {
			vehiclemoving = false;
		}
		
		/*
        for (i=0;i<4;i++) A[i] = readInput(dacModule, A_Reg[i]);

        wiringPiI2CWriteReg8(dacModule, 0x40, (A[0]+A[1]+A[2]+A[3])/4);

        cout << endl;
        for (i=0;i<4;i++) cout << i << " : " << A[i] << endl;
		*/
		
		fcwtracker.Update( followingdistance, alarmdata::gpsspeed);
		
		//Update everything
		alarmdata::forwarddistance = fcwtracker.followingdistance_;
		if ( fcwtracker.followingtime_ < fcwtracker.timetocollision_ ) {
			alarmdata::timetocollision = fcwtracker.followingtime_;
		} else {
			alarmdata::timetocollision = fcwtracker.timetocollision_;			
		}
		
		//FCW Status
		//0 = inactive (disabled by xml or zero speed)
		//1 = fcw warning
		//2 = following too close warning
		//3 = fcw alarm
		//4 = following too close alarm
		//5 = driver ahead takeoff notification				//Future
		//-1 = error (sensor error)
		if ( (1000*fcwtracker.timetocollision_) < settings::fcw::mscollisionwarning ) {
			alarmdata::fcwstatus = 1;
			alarmdata::fcwpwmvalue = 1023 + static_cast<int>((1024.0*(1000*
				fcwtracker.timetocollision_ - settings::fcw::mscollisionalarm)) /
				(settings::fcw::mscollisionalarm - settings::fcw::mscollisionwarning));
		} else if ( vehiclemoving && (1000*fcwtracker.followingtime_) <
			settings::fcw::msfollowdistwarning ){
			alarmdata::fcwstatus = 2;
			alarmdata::fcwpwmvalue = 1023 + static_cast<int>((1024.0*(1000*
				fcwtracker.followingtime_ - settings::fcw::msfollowdistalarm)) /
				(settings::fcw::msfollowdistalarm - settings::fcw::msfollowdistwarning));
		} else if ( (1000*fcwtracker.timetocollision_) < settings::fcw::mscollisionalarm ){
			alarmdata::fcwstatus = 3;
			alarmdata::fcwpwmvalue = 1023;
		} else if ( vehiclemoving && (1000*fcwtracker.followingtime_) <
			settings::fcw::msfollowdistalarm ){
			alarmdata::fcwstatus = 4;
			alarmdata::fcwpwmvalue = 1023;
		} else if ( false ){	//ToDo - Comm check & takeoff notice
			alarmdata::fcwstatus = -1;
			alarmdata::fcwpwmvalue = 0;
		} else {
			alarmdata::fcwstatus = 0;
			alarmdata::fcwpwmvalue = 0;
		}
		if (alarmdata::fcwpwmvalue < 0) {
			alarmdata::fcwpwmvalue = 0;
		} else if (alarmdata::fcwpwmvalue > 1023) {
			alarmdata::fcwpwmvalue = 1023;
		}
		if ( !vehiclemoving || (fcwtracker.timetocollision_ < fcwtracker.followingtime_) &&
			(alarmdata::fcwstatus == 0) ) {
			alarmdata::fcwpwmvalue = 1023 + static_cast<int>((1024.0*(1000*
				fcwtracker.timetocollision_ - settings::fcw::mscollisionwarning)) /
				(settings::fcw::mscollisionwarning));			
		} else if ( vehiclemoving && (fcwtracker.timetocollision_ >
			fcwtracker.followingtime_) && (alarmdata::fcwstatus == 0) ) {
			alarmdata::fcwpwmvalue = 1023 + static_cast<int>((1024.0*(1000*
				fcwtracker.followingtime_ - settings::fcw::msfollowdistwarning)) /
				(settings::fcw::msfollowdistwarning));			
		}
		
		//Check for driver pullahead
		if ( !vehiclemoving && (fcwtracker.acceleration_ > 0.1) && (
			alarmdata::fcwstatus = 0) && (pullaheadcount > pullaheaddelay) ) {
			alarmdata::fcwstatus = 5;
		} else if ( !vehiclemoving && (fcwtracker.acceleration_ > 0.1) && (
			alarmdata::fcwstatus = 0) ) {
			pullaheadcount++;
		} else {
			pullaheadcount = 0;
		}
		
		//Setpace
		lidarpacer.SetPace();
	}
	
	std::cout << "Exiting Lidar polling thread!" << std::endl;

}