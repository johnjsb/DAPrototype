#ifndef PACESETTER_H
#define PACESETTER_H

#include <chrono>
#include <string>

class PaceSetter
{
    public:
        PaceSetter( int fps, std::string threadname );
        void SetPace();
        virtual ~PaceSetter();

    protected:

    private:
        long targetusec_;
		long sleeptime_;
		int missedcount_;
		const std::string threadname_;
        std::chrono::high_resolution_clock::time_point lastsettime_;
};

#endif // PACESETTER_H
