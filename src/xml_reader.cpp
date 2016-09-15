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

//Placeholder variables for constant initialization

//GEN
static bool genenbuzzer{true};
static bool gendebugscreen{false};
static bool gendebugterminal{false};
//CAM
static bool camrecordorgimage{false};
static std::string camfilepath{"/home/pi/videos/"};
static std::string camorgfilename{"in.avi"};
static std::string cammodfilename{"out.avi"};
static int campixwidth{640};
static int campixheight{480};
static bool camshowspeed{true};
static bool camshowloc{true};
static bool camshadelanes{true};
static int camrecfps{10};
static int camfilestokeep{20};
static int camminperfile{30};
//DISP
static bool dispenabled{true};
static int disppixwidth{800};
static int disppixheight{480};
static int dispupdatefps{10};
//COMM
static int commpollrategps{10};
static int commpollratelidar{10};
static int commpollrategpio{10};
//GPS
static int gpssamplestoaverage{5};
//LDW
static bool ldwenabled{true};
static int ldwenablespeed{0};
static int ldwsamplestokeep{7};
static int ldwsamplestoaverage{4};
static int ldwperoffsetwarning{15};
static int ldwperoffsetalarm{20};
static int ldwmsuntilwarning{500};
static int ldwmsuntilalarm{500};
static int ldwupdatefps{10};
//FCW
static bool fcwenabled{true};
static int fcwsamplestoaverage{3};
static double fcwdistanceoffset{0.0};
static int fcwmsfollowdistwarning{2500};
static int fcwmsfollowdistalarm{2000};
static int fcwmscollisionwarning{300};
static int fcwmscollisionalarm{300};
//GPIO
static bool gpioenabled{true};
static bool gpioautoshutdown{true};
	
using namespace tinyxml2;
using namespace std;

int ReadXmlSettings()
{
    std::cout << "Reading XML..." << std::endl;
	XMLDocument doc;
	doc.LoadFile( "../RPiDashcamSettings.xml");
	if ( doc.ErrorID() != 0 ) {
		return -1;
	} else {
		try{
			XMLElement* attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "GEN" );
			(*(*attributeApproachElement).FirstChildElement( "enBuzzer" )).QueryBoolText( 
				&genenbuzzer );
			(*(*attributeApproachElement).FirstChildElement( "debugScreen" )).QueryBoolText( 
				&gendebugscreen );		
			(*(*attributeApproachElement).FirstChildElement( "debugTerminal" )).QueryBoolText( 
				&gendebugterminal );

			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "CAM" );
			(*(*attributeApproachElement).FirstChildElement( "recordOrgImage" )).QueryBoolText( 
				&camrecordorgimage );
			camfilepath = (*(*attributeApproachElement).FirstChildElement(
				"filePath" )).GetText();
			camorgfilename = (*(*attributeApproachElement).FirstChildElement(
				"orgFileName" )).GetText();
			cammodfilename = (*(*attributeApproachElement).FirstChildElement(
				"modFileName" )).GetText();
			(*(*attributeApproachElement).FirstChildElement( "pixWidth" )).QueryIntText( 
				&campixwidth );
			(*(*attributeApproachElement).FirstChildElement( "pixHeight" )).QueryIntText( 
				&campixheight );	
			(*(*attributeApproachElement).FirstChildElement( "showSpeed" )).QueryBoolText( 
				&camshowspeed );	
			(*(*attributeApproachElement).FirstChildElement( "showLoc" )).QueryBoolText( 
				&camshowloc );		
			(*(*attributeApproachElement).FirstChildElement( "shadeLanes" )).QueryBoolText( 
				&camshadelanes );	
			(*(*attributeApproachElement).FirstChildElement( "recFPS" )).QueryIntText( 
				&camrecfps );		
			(*(*attributeApproachElement).FirstChildElement( "filesToKeep" )).QueryIntText( 
				&camfilestokeep );	
			(*(*attributeApproachElement).FirstChildElement( "minPerFile" )).QueryIntText( 
				&camminperfile );

			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "DISP" );
			(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
				&dispenabled );
			(*(*attributeApproachElement).FirstChildElement( "pixWidth" )).QueryIntText( 
				&disppixwidth );
			(*(*attributeApproachElement).FirstChildElement( "pixHeight" )).QueryIntText( 
				&disppixheight );
			(*(*attributeApproachElement).FirstChildElement( "updateFPS" )).QueryIntText( 
				&dispupdatefps );

			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "COMM" );
			(*(*attributeApproachElement).FirstChildElement( "pollRateGPS" )).QueryIntText( 
				&commpollrategps );
			(*(*attributeApproachElement).FirstChildElement( "pollRateLIDAR" )).QueryIntText( 
				&commpollratelidar );
			(*(*attributeApproachElement).FirstChildElement( "pollRateGpio" )).QueryIntText( 
				&commpollrategpio );	

			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "LDW" );	
			(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
				&ldwenabled );
			(*(*attributeApproachElement).FirstChildElement( "enableSpeed" )).QueryIntText( 
				&ldwenablespeed );
			(*(*attributeApproachElement).FirstChildElement( "samplesToKeep" )).QueryIntText( 
				&ldwsamplestokeep );
			(*(*attributeApproachElement).FirstChildElement( "samplesToAverage" )).QueryIntText( 
				&ldwsamplestoaverage );
			(*(*attributeApproachElement).FirstChildElement( "perOffsetWarning" )).QueryIntText( 
				&ldwperoffsetwarning );	
			(*(*attributeApproachElement).FirstChildElement( "perOffsetAlarm" )).QueryIntText( 
				&ldwperoffsetalarm );	
			(*(*attributeApproachElement).FirstChildElement( "msUntilWarning" )).QueryIntText( 
				&ldwmsuntilwarning );		
			(*(*attributeApproachElement).FirstChildElement( "msUntilAlarm" )).QueryIntText( 
				&ldwmsuntilalarm );		
			(*(*attributeApproachElement).FirstChildElement( "updateFPS" )).QueryIntText( 
				&ldwupdatefps );		
				
			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "FCW" );
			(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
				&fcwenabled );
			(*(*attributeApproachElement).FirstChildElement( "samplesToAverage" )).QueryIntText( 
				&fcwsamplestoaverage );
			(*(*attributeApproachElement).FirstChildElement( "distanceOffset" )).QueryDoubleText( 
				&fcwdistanceoffset );
			(*(*attributeApproachElement).FirstChildElement( "msFollowDistWarning" )).QueryIntText( 
				&fcwmsfollowdistwarning );
			(*(*attributeApproachElement).FirstChildElement( "msFollowDistAlarm" )).QueryIntText( 
				&fcwmsfollowdistalarm );
			(*(*attributeApproachElement).FirstChildElement( "msCollisionWarning" )).QueryIntText( 
				&fcwmscollisionwarning );
			(*(*attributeApproachElement).FirstChildElement( "msCollisionAlarm" )).QueryIntText( 
				&fcwmscollisionalarm );

			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "GPIO" );
			(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
				&gpioenabled );
			(*(*attributeApproachElement).FirstChildElement( "autoShutdown" )).QueryBoolText( 
				&gpioautoshutdown );		
		} catch ( exception E ) {
			return -2;	
		}
	}
		
	return 1;
}

//Initialize all namespace constants
namespace settings{
	const int readsuccess(ReadXmlSettings());
	namespace gen {
		const bool enbuzzer{genenbuzzer};
		const bool debugscreen{gendebugscreen};
		const bool debugterminal{gendebugterminal};
	}
	namespace cam{
		const bool recordorgimage{camrecordorgimage};
		const std::string filepath{camfilepath};
		const std::string orgfilename{camorgfilename};
		const std::string modfilename{cammodfilename};
		const int pixwidth{campixwidth};
		const int pixheight{campixheight};
		const bool showspeed{camshowspeed};
		const bool showloc{camshowloc};
		const bool shadelanes{camshadelanes};
		const int recfps{camrecfps};
		const int filestokeep{camfilestokeep};
		const int minperfile{camminperfile};
	}
	namespace disp{
		const bool enabled{dispenabled};
		const int pixwidth{disppixwidth};
		const int pixheight{disppixheight};
		const int updatefps{dispupdatefps};
	}
	namespace comm{
		const int pollrategps{commpollrategps};
		const int pollratelidar{commpollratelidar};
		const int pollrategpio{commpollrategpio};
	}
	namespace gps{
		const int samplestoaverage{gpssamplestoaverage};
	}
	namespace ldw{
		const bool enabled{ldwenabled};
		const int enablespeed{ldwenablespeed};
		const int samplestokeep{ldwsamplestokeep};
		const int samplestoaverage{ldwsamplestoaverage};
		const int peroffsetwarning{ldwperoffsetwarning};
		const int peroffsetalarm{ldwperoffsetalarm};
		const int msuntilwarning{ldwmsuntilwarning};
		const int msuntilalarm{ldwmsuntilalarm};
		const int updatefps{ldwupdatefps};
	}
	namespace fcw{
		const bool enabled{enabled};
		const int samplestoaverage{fcwsamplestoaverage};
		const double distanceoffset{fcwdistanceoffset};
		const int msfollowdistwarning{fcwmsfollowdistwarning};
		const int msfollowdistalarm{fcwmsfollowdistalarm};
		const int mscollisionwarning{fcwmscollisionwarning};
		const int mscollisionalarm{fcwmscollisionalarm};
	}
	namespace gpio{
		const bool enabled{gpioenabled};
		const bool autoshutdown{gpioautoshutdown};
	}
}
