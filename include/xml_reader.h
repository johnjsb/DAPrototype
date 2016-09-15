#pragma once

#ifndef XML_READER_H_INCLUDED
#define XML_READER_H_INCLUDED

#include <string>

int ReadXmlSettings();

namespace settings{
	extern const int readsuccess;
    namespace gen {
        extern const bool enbuzzer;
        extern const bool debugscreen;
        extern const bool debugterminal;
    }
    namespace cam{
		extern const bool recordorgimage;
		extern const std::string filepath;
		extern const std::string orgfilename;
		extern const std::string modfilename;
        extern const int pixwidth;
        extern const int pixheight;
        extern const bool showspeed;
        extern const bool showloc;
        extern const bool shadelanes;
        extern const int recfps;
        extern const int filestokeep;
        extern const int minperfile;
    }
    namespace disp{
        extern const bool enabled;
        extern const int pixwidth;
        extern const int pixheight;
        extern const int updatefps;
    }
    namespace comm{
        extern const int pollrategps;
        extern const int pollratelidar;
        extern const int pollrategpio;
    }
    namespace gps{
        extern const int samplestoaverage;
    }
    namespace ldw{
        extern const bool enabled;
		extern const int enablespeed;
        extern const int samplestokeep;
        extern const int samplestoaverage;
        extern const int peroffsetwarning;
        extern const int peroffsetalarm;
        extern const int msuntilwarning;
        extern const int msuntilalarm;
        extern const int updatefps;
    }
    namespace fcw{
        extern const bool enabled;
        extern const int samplestoaverage;
		extern const double distanceoffset;
        extern const int msfollowdistwarning;
        extern const int msfollowdistalarm;
        extern const int mscollisionwarning;
        extern const int mscollisionalarm;
    }
    namespace gpio{
        extern const bool enabled;
        extern const bool autoshutdown;
    }
}

#endif // XML_READER_H_INCLUDED
