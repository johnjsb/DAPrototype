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
bool genenbuzzer{true};
bool gendebugscreen{false};
bool gendebugterminal{false};
//CAM
bool camrecordorgimage{false};
std::string camfilepath{"/home/pi/videos/"};
std::string camorgfilename{"in.avi"};
std::string cammodfilename{"out.avi"};
int campixwidth{640};
int campixheight{480};
bool camshowspeed{true};
bool camshowloc{true};
bool camshadelanes{true};
int camrecfps{10};
int camfilestokeep{20};
int camminperfile{30};
//DISP
bool dispenabled{true};
int disppixwidth{800};
int disppixheight{480};
int dispupdatefps{10};
//COMM
int commpollrategps{10};
int commpollratelidar{10};
int commpollrategpio{10};
//GPS
int gpssamplestoaverage{5};
//LDW
bool ldwenabled{true};
int ldwenablespeed{0};
int ldwsamplestokeep{7};
int ldwsamplestoaverage{4};
int ldwperoffsetwarning{15};
int ldwperoffsetalarm{20};
int ldwmsuntilwarning{500};
int ldwmsuntilalarm{500};
int ldwupdatefps{10};
//FCW
bool fcwenabled{true};
int fcwsamplestoaverage{3};
double fcwdistanceoffset{0.0};
int fcwmsfollowdistwarning{2500};
int fcwmsfollowdistalarm{2000};
int fcwmscollisionwarning{300};
int fcwmscollisionalarm{300};
//GPIO
bool gpioenabled{true};
bool gpioautoshutdown{true};
	
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
