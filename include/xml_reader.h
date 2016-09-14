#pragma once

#ifndef XML_READER_H_INCLUDED
#define XML_READER_H_INCLUDED

#include <string>

namespace settings{
    namespace gen {
        extern bool enbuzzer;
        extern bool debugscreen;
        extern bool debugterminal;
    }
    namespace cam{
		extern bool recordorgimage;
		extern std::string filepath;
		extern std::string orgfilename;
		extern std::string modfilename;
        extern int pixwidth;
        extern int pixheight;
        extern bool showspeed;
        extern bool showloc;
        extern bool shadelanes;
        extern int recfps;
        extern int filestokeep;
        extern int minperfile;
    }
    namespace disp{
        extern bool enabled;
        extern int pixwidth;
        extern int pixheight;
        extern int updatefps;
    }
    namespace comm{
        extern int pollrategps;
        extern int pollratelidar;
        extern int pollrategpio;
    }
    namespace gps{
        extern int samplestoaverage;
    }
    namespace ldw{
        extern bool enabled;
		extern int enablespeed;
        extern int samplestokeep;
        extern int samplestoaverage;
        extern int peroffsetwarning;
        extern int peroffsetalarm;
        extern int msuntilwarning;
        extern int msuntilalarm;
        extern int updatefps;
    }
    namespace fcw{
        extern bool enabled;
        extern int samplestoaverage;
		extern double distanceoffset;
        extern int msfollowdistwarning;
        extern int msfollowdistalarm;
        extern int mscollisionwarning;
        extern int mscollisionalarm;
    }
    namespace gpio{
        extern bool enabled;
        extern bool autoshutdown;
    }
}

int ReadXmlSettings();

#endif // XML_READER_H_INCLUDED
