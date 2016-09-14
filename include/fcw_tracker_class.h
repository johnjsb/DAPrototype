#ifndef FCWTRACKER_H
#define FCWTRACKER_H

#include <chrono>
#include <deque>

class FcwTracker
{
    public:
		double followingdistance_;  		//m (averaged)
		double followingtime_;  			//s
		double timetocollision_;			//s
		double acceleration_;				//m/s^2 (averaged)
        FcwTracker( double distanceoffset,
					int samplestoaverage = 3 );
        void Update( double distance,
					 double speed );
        virtual ~FcwTracker();

    protected:

    private:
		std::deque<double> distances_;		//m (relative to forward object)
		std::deque<double> velocities_;		//m/s (relative to forward object)
		std::deque<double> accelerations_;	//m/s^2 (relative to forward object)
		double velocity_;					//m/s (averaged)
		const double kdistanceoffset_;		//m (windshield to bumper distance)
		const int ksamplestoaverage_;		//number of samples for noise reduction
        long sampletime_;					//usec (time between samples)
        std::chrono::high_resolution_clock::time_point lastsampletime_;
		double Average ( std::deque<double> deque );
};

#endif // FCWTRACKER_H
