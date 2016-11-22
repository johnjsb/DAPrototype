/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  License:
	  This software is licensed under GNU GPL v3.0
	  
******************************************************************************************/

#pragma once

//Header guard
#ifndef XML_READER_H_INCLUDED
#define XML_READER_H_INCLUDED

//Standard libraries
#include <string>

/*****************************************************************************************/
int ReadXmlSettings();

namespace settings{
	extern const int kreadsuccess;
    namespace gen {
        extern const bool kenbuzzer;
        extern const bool kdebugscreen;
        extern const bool kdebugterminal;
    }
    namespace cam{
		extern const bool krecordorgimage;
		extern const std::string kfilepath;
		extern const std::string korgfilename;
		extern const std::string kmodfilename;
        extern const int kpixwidth;
        extern const int kpixheight;
        extern const bool kshowspeed;
        extern const bool kshowloc;
        extern const bool kshadelanes;
        extern const int krecfps;
        extern const int kfilestokeep;
        extern const int kminperfile;
    }
    namespace disp{
        extern const bool kenabled;
        extern const int kpixwidth;
        extern const int kpixheight;
        extern const int kupdatefps;
    }
    namespace comm{
        extern const int kpollrategps;
        extern const int kpollratelidar;
        extern const int kpollrategpio;
    }
    namespace gps{
        extern const int ksamplestoaverage;
    }
    namespace ldw{
        extern const bool kenabled;
		extern const int kenablespeed;
        extern const int ksamplestokeep;
        extern const int ksamplestoaverage;
        extern const int kperoffsetwarning;
        extern const int kperoffsetalarm;
        extern const int kmsuntilwarning;
        extern const int kmsuntilalarm;
        extern const int kupdatefps;
    }
    namespace fcw{
        extern const bool kenabled;
        extern const int ksamplestoaverage;
		extern const double kdistanceoffset;
        extern const int kmsfollowdistwarning;
        extern const int kmsfollowdistalarm;
        extern const int kmscollisionwarning;
        extern const int kmscollisionalarm;
    }
    namespace gpio{
        extern const bool kenabled;
        extern const bool kautoshutdown;
    }
}

#endif // XML_READER_H_INCLUDED
