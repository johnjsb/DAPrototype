#include <iostream>
#include <atomic>
//#include <wiringPi.h>
//#include <wiringPiI2C.h>
#include "pace_setter_class.h"
#include "process_values_class.h"
#include "fcw_tracker_class.h"
#include "xml_reader.h"
/*
int readInput(int fd, int reg)
{
    wiringPiI2CReadReg8(fd, reg);
    return wiringPiI2CReadReg8(fd, reg);
}
*/
void LidarPolingThread( ProcessValues *processvalues,
						std::atomic<bool> *exitsignal )
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
		if ( processvalues->gpsspeed_ > 1.0 ) {
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
		
		fcwtracker.Update( followingdistance, processvalues->gpsspeed_);
		
		//Update everything
		processvalues->forwarddistance_ = fcwtracker.followingdistance_;
		if ( fcwtracker.followingtime_ < fcwtracker.timetocollision_ ) {
			processvalues->timetocollision_ = fcwtracker.followingtime_;
		} else {
			processvalues->timetocollision_ = fcwtracker.timetocollision_;			
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
			processvalues->fcwstatus_ = 1;
			processvalues->fcwpwmvalue_ = 1023 + static_cast<int>((1024.0*(1000*
				fcwtracker.timetocollision_ - settings::fcw::mscollisionalarm)) /
				(settings::fcw::mscollisionalarm - settings::fcw::mscollisionwarning));
		} else if ( vehiclemoving && (1000*fcwtracker.followingtime_) <
			settings::fcw::msfollowdistwarning ){
			processvalues->fcwstatus_ = 2;
			processvalues->fcwpwmvalue_ = 1023 + static_cast<int>((1024.0*(1000*
				fcwtracker.followingtime_ - settings::fcw::msfollowdistalarm)) /
				(settings::fcw::msfollowdistalarm - settings::fcw::msfollowdistwarning));
		} else if ( (1000*fcwtracker.timetocollision_) < settings::fcw::mscollisionalarm ){
			processvalues->fcwstatus_ = 3;
			processvalues->fcwpwmvalue_ = 1023;
		} else if ( vehiclemoving && (1000*fcwtracker.followingtime_) <
			settings::fcw::msfollowdistalarm ){
			processvalues->fcwstatus_ = 4;
			processvalues->fcwpwmvalue_ = 1023;
		} else if ( false ){	//ToDo - Comm check & takeoff notice
			processvalues->fcwstatus_ = -1;
			processvalues->fcwpwmvalue_ = 0;
		} else {
			processvalues->fcwstatus_ = 0;
			processvalues->fcwpwmvalue_ = 0;
		}
		if (processvalues->fcwpwmvalue_ < 0) {
			processvalues->fcwpwmvalue_ = 0;
		} else if (processvalues->fcwpwmvalue_ > 1023) {
			processvalues->fcwpwmvalue_ = 1023;
		}
		if ( !vehiclemoving || (fcwtracker.timetocollision_ < fcwtracker.followingtime_) &&
			(processvalues->fcwstatus_ == 0) ) {
			processvalues->fcwpwmvalue_ = 1023 + static_cast<int>((1024.0*(1000*
				fcwtracker.timetocollision_ - settings::fcw::mscollisionwarning)) /
				(settings::fcw::mscollisionwarning));			
		} else if ( vehiclemoving && (fcwtracker.timetocollision_ >
			fcwtracker.followingtime_) && (processvalues->fcwstatus_ == 0) ) {
			processvalues->fcwpwmvalue_ = 1023 + static_cast<int>((1024.0*(1000*
				fcwtracker.followingtime_ - settings::fcw::msfollowdistwarning)) /
				(settings::fcw::msfollowdistwarning));			
		}
		
		//Check for driver pullahead
		if ( !vehiclemoving && (fcwtracker.acceleration_ > 0.1) && (
			processvalues->fcwstatus_ = 0) && (pullaheadcount > pullaheaddelay) ) {
			processvalues->fcwstatus_ = 5;
		} else if ( !vehiclemoving && (fcwtracker.acceleration_ > 0.1) && (
			processvalues->fcwstatus_ = 0) ) {
			pullaheadcount++;
		} else {
			pullaheadcount = 0;
		}
		
		//Setpace
		lidarpacer.SetPace();
	}
	
	std::cout << "Exiting Lidar polling thread!" << std::endl;

}