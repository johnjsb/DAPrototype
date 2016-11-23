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
#ifndef FCWTRACKER_H
#define FCWTRACKER_H

//Standard libraries
#include <chrono>
#include <deque>

/*****************************************************************************************/
class FcwTracker
{
    public:
		double followingdistance_;  		//ft (averaged)
		double followingtime_;  			//s
		double timetocollision_;			//s
		double acceleration_;				//ft/s^2 (averaged)
        FcwTracker( double distanceoffset,
					int samplestoaverage = 3 );
        void Update( double distance,
					 double speed );
        virtual ~FcwTracker();

    protected:

    private:
		std::deque<double> distances_;		//ft (relative to forward object)
		std::deque<double> velocities_;		//ft/s (relative to forward object)
		std::deque<double> accelerations_;	//ft/s^2 (relative to forward object)
		double velocity_;					//ft/s (averaged)
		const int ksamplestoaverage_;		//number of samples for noise reduction
        uint32_t sampletime_;				//usec (time between samples)
        std::chrono::high_resolution_clock::time_point lastsampletime_;
		double Average ( std::deque<double> deque );
};

#endif // FCWTRACKER_H
