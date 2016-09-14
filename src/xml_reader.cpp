#include <iostream>
#include <string>
#include "tinyxml2.h"
#include "xml_reader.h"


#include <cstdlib>
#include <cstring>
#include <ctime>
#include <direct.h>
//#include <crtdbg.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <io.h>
#include <sys/stat.h>

namespace settings{
    namespace gen {
        bool enbuzzer{true};
        bool debugscreen{false};
        bool debugterminal{false};
    }
    namespace cam{
		bool recordorgimage{false};
		std::string filepath{"/home/pi/videos/"};
		std::string orgfilename{"in.avi"};
		std::string modfilename{"out.avi"};
        int pixwidth{640};
        int pixheight{480};
        bool showspeed{true};
        bool showloc{true};
        bool shadelanes{true};
        int recfps{10};
        int filestokeep{20};
        int minperfile{30};
    }
    namespace disp{
        bool enabled{true};
        int pixwidth{800};
        int pixheight{480};
        int updatefps{10};
    }
    namespace comm{
        int pollrategps{10};
        int pollratelidar{10};
        int pollrategpio{10};
    }
    namespace gps{
        int samplestoaverage{5};
    }
    namespace ldw{
        bool enabled{true};
		int enablespeed{0};
        int samplestokeep{7};
        int samplestoaverage{4};
        int peroffsetwarning{15};
        int peroffsetalarm{20};
        int msuntilwarning{500};
        int msuntilalarm{500};
        int updatefps{10};
    }
    namespace fcw{
        bool enabled{true};
        int samplestoaverage{3};
		double distanceoffset{0.0};
        int msfollowdistwarning{2500};
        int msfollowdistalarm{2000};
        int mscollisionwarning{300};
        int mscollisionalarm{300};
    }
    namespace gpio{
        bool enabled{true};
        bool autoshutdown{true};
	}
}
    

using namespace tinyxml2;
using namespace std;

int ReadXmlSettings()
{

	XMLDocument doc;
	doc.LoadFile( "../RPiDashcamSettings.xml");
	if ( doc.ErrorID() != 0 ) {		
		//cout << "XML file read error: " << doc.ErrorID();
		//cout << " Using defaults" << endl;
		return -1;
	}
	
	try{
		XMLElement* attributeApproachElement = doc.FirstChildElement()->
			FirstChildElement( "GEN" );
		(*(*attributeApproachElement).FirstChildElement( "enBuzzer" )).QueryBoolText( 
			&settings::gen::enbuzzer );
		(*(*attributeApproachElement).FirstChildElement( "debugScreen" )).QueryBoolText( 
			&settings::gen::debugscreen );		
		(*(*attributeApproachElement).FirstChildElement( "debugTerminal" )).QueryBoolText( 
			&settings::gen::debugterminal );

		attributeApproachElement = doc.FirstChildElement()->
			FirstChildElement( "CAM" );
		(*(*attributeApproachElement).FirstChildElement( "recordOrgImage" )).QueryBoolText( 
			&settings::cam::recordorgimage );
		settings::cam::filepath = (*(*attributeApproachElement).FirstChildElement(
			"filePath" )).GetText();
		settings::cam::orgfilename = (*(*attributeApproachElement).FirstChildElement(
			"orgFileName" )).GetText();
		settings::cam::modfilename = (*(*attributeApproachElement).FirstChildElement(
			"modFileName" )).GetText();
		(*(*attributeApproachElement).FirstChildElement( "pixWidth" )).QueryIntText( 
			&settings::cam::pixwidth );
		(*(*attributeApproachElement).FirstChildElement( "pixHeight" )).QueryIntText( 
			&settings::cam::pixheight );	
		(*(*attributeApproachElement).FirstChildElement( "showSpeed" )).QueryBoolText( 
			&settings::cam::showspeed );	
		(*(*attributeApproachElement).FirstChildElement( "showLoc" )).QueryBoolText( 
			&settings::cam::showloc );		
		(*(*attributeApproachElement).FirstChildElement( "shadeLanes" )).QueryBoolText( 
			&settings::cam::shadelanes );	
		(*(*attributeApproachElement).FirstChildElement( "recFPS" )).QueryIntText( 
			&settings::cam::recfps );		
		(*(*attributeApproachElement).FirstChildElement( "filesToKeep" )).QueryIntText( 
			&settings::cam::filestokeep );	
		(*(*attributeApproachElement).FirstChildElement( "minPerFile" )).QueryIntText( 
			&settings::cam::minperfile );

		attributeApproachElement = doc.FirstChildElement()->
			FirstChildElement( "DISP" );
		(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
			&settings::disp::enabled );
		(*(*attributeApproachElement).FirstChildElement( "pixWidth" )).QueryIntText( 
			&settings::disp::pixwidth );
		(*(*attributeApproachElement).FirstChildElement( "pixHeight" )).QueryIntText( 
			&settings::disp::pixheight );
		(*(*attributeApproachElement).FirstChildElement( "updateFPS" )).QueryIntText( 
			&settings::disp::updatefps );

		attributeApproachElement = doc.FirstChildElement()->
			FirstChildElement( "COMM" );
		(*(*attributeApproachElement).FirstChildElement( "pollRateGPS" )).QueryIntText( 
			&settings::comm::pollrategps );
		(*(*attributeApproachElement).FirstChildElement( "pollRateLIDAR" )).QueryIntText( 
			&settings::comm::pollratelidar );
		(*(*attributeApproachElement).FirstChildElement( "pollRateGpio" )).QueryIntText( 
			&settings::comm::pollrategpio );	

		attributeApproachElement = doc.FirstChildElement()->
			FirstChildElement( "LDW" );	
		(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
			&settings::ldw::enabled );
		(*(*attributeApproachElement).FirstChildElement( "enableSpeed" )).QueryIntText( 
			&settings::ldw::enablespeed );
		(*(*attributeApproachElement).FirstChildElement( "samplesToKeep" )).QueryIntText( 
			&settings::ldw::samplestokeep );
		(*(*attributeApproachElement).FirstChildElement( "samplesToAverage" )).QueryIntText( 
			&settings::ldw::samplestoaverage );
		(*(*attributeApproachElement).FirstChildElement( "perOffsetWarning" )).QueryIntText( 
			&settings::ldw::peroffsetwarning );	
		(*(*attributeApproachElement).FirstChildElement( "perOffsetAlarm" )).QueryIntText( 
			&settings::ldw::peroffsetalarm );	
		(*(*attributeApproachElement).FirstChildElement( "msUntilWarning" )).QueryIntText( 
			&settings::ldw::msuntilwarning );		
		(*(*attributeApproachElement).FirstChildElement( "msUntilAlarm" )).QueryIntText( 
			&settings::ldw::msuntilalarm );		
		(*(*attributeApproachElement).FirstChildElement( "updateFPS" )).QueryIntText( 
			&settings::ldw::updatefps );		
			
		attributeApproachElement = doc.FirstChildElement()->
			FirstChildElement( "FCW" );
		(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
			&settings::fcw::enabled );
		(*(*attributeApproachElement).FirstChildElement( "samplesToAverage" )).QueryIntText( 
			&settings::fcw::samplestoaverage );
		(*(*attributeApproachElement).FirstChildElement( "distanceOffset" )).QueryDoubleText( 
			&settings::fcw::distanceoffset );
		(*(*attributeApproachElement).FirstChildElement( "msFollowDistWarning" )).QueryIntText( 
			&settings::fcw::msfollowdistwarning );
		(*(*attributeApproachElement).FirstChildElement( "msFollowDistAlarm" )).QueryIntText( 
			&settings::fcw::msfollowdistalarm );
		(*(*attributeApproachElement).FirstChildElement( "msCollisionWarning" )).QueryIntText( 
			&settings::fcw::mscollisionwarning );
		(*(*attributeApproachElement).FirstChildElement( "msCollisionAlarm" )).QueryIntText( 
			&settings::fcw::mscollisionalarm );

		attributeApproachElement = doc.FirstChildElement()->
			FirstChildElement( "GPIO" );
		(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
			&settings::gpio::enabled );
		(*(*attributeApproachElement).FirstChildElement( "autoShutdown" )).QueryBoolText( 
			&settings::gpio::autoshutdown );		
	} catch ( exception E ) {
		//cout << "XML read error: " << endl;	
		return -2;	
	}
		
	//cout << "XML settings read successfully!" << endl;
		
	return 1;
}
