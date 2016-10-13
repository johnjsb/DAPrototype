#include <iostream>
#include <string>
#include "tinyxml2.h"
#include "xml_reader.h"

//GEN
static bool g_genenbuzzer{true};
static bool g_gendebugscreen{false};
static bool g_gendebugterminal{false};
//CAM
static bool g_camrecordorgimage{false};
static std::string g_camfilepath{"/home/pi/videos/"};
static std::string g_camorgfilename{"in.avi"};
static std::string g_cammodfilename{"out.avi"};
static int g_campixwidth{640};
static int g_campixheight{480};
static bool g_camshowspeed{true};
static bool g_camshowloc{true};
static bool g_camshadelanes{true};
static int g_camrecfps{10};
static int g_camfilestokeep{20};
static int g_camminperfile{30};
//DISP
static bool g_dispenabled{true};
static int g_disppixwidth{800};
static int g_disppixheight{480};
static int g_dispupdatefps{10};
//COMM
static int g_commpollrategps{10};
static int g_commpollratelidar{10};
static int g_commpollrategpio{10};
//GPS
static int g_gpssamplestoaverage{5};
//LDW
static bool g_ldwenabled{true};
static int g_ldwenablespeed{0};
static int g_ldwsamplestokeep{7};
static int g_ldwsamplestoaverage{4};
static int g_ldwperoffsetwarning{15};
static int g_ldwperoffsetalarm{20};
static int g_ldwmsuntilwarning{500};
static int g_ldwmsuntilalarm{500};
static int g_ldwupdatefps{10};
//FCW
static bool g_fcwenabled{true};
static int g_fcwsamplestoaverage{3};
static double g_fcwdistanceoffset{0.0};
static int g_fcwmsfollowdistwarning{2500};
static int g_fcwmsfollowdistalarm{2000};
static int g_fcwmscollisionwarning{300};
static int g_fcwmscollisionalarm{300};
//GPIO
static bool g_gpioenabled{true};
static bool g_gpioautoshutdown{true};
	
using namespace tinyxml2;
using namespace std;

int ReadXmlSettings()
{
    std::cout << "Reading XML..." << std::endl;
	XMLDocument doc;
	doc.LoadFile( "/RPiDashcamSettings.xml");
	if ( doc.ErrorID() != 0 ) {
		return -1;
	} else {
		try{
			XMLElement* attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "GEN" );
			(*(*attributeApproachElement).FirstChildElement( "enBuzzer" )).QueryBoolText( 
				&g_genenbuzzer );
			(*(*attributeApproachElement).FirstChildElement( "debugScreen" )).QueryBoolText( 
				&g_gendebugscreen );		
			(*(*attributeApproachElement).FirstChildElement( "debugTerminal" )).QueryBoolText( 
				&g_gendebugterminal );

			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "CAM" );
			(*(*attributeApproachElement).FirstChildElement( "recordOrgImage" )).QueryBoolText( 
				&g_camrecordorgimage );
			g_camfilepath = (*(*attributeApproachElement).FirstChildElement(
				"filePath" )).GetText();
			g_camorgfilename = (*(*attributeApproachElement).FirstChildElement(
				"orgFileName" )).GetText();
			g_cammodfilename = (*(*attributeApproachElement).FirstChildElement(
				"modFileName" )).GetText();
			(*(*attributeApproachElement).FirstChildElement( "pixWidth" )).QueryIntText( 
				&g_campixwidth );
			(*(*attributeApproachElement).FirstChildElement( "pixHeight" )).QueryIntText( 
				&g_campixheight );	
			(*(*attributeApproachElement).FirstChildElement( "showSpeed" )).QueryBoolText( 
				&g_camshowspeed );	
			(*(*attributeApproachElement).FirstChildElement( "showLoc" )).QueryBoolText( 
				&g_camshowloc );		
			(*(*attributeApproachElement).FirstChildElement( "shadeLanes" )).QueryBoolText( 
				&g_camshadelanes );	
			(*(*attributeApproachElement).FirstChildElement( "recFPS" )).QueryIntText( 
				&g_camrecfps );		
			(*(*attributeApproachElement).FirstChildElement( "filesToKeep" )).QueryIntText( 
				&g_camfilestokeep );	
			(*(*attributeApproachElement).FirstChildElement( "minPerFile" )).QueryIntText( 
				&g_camminperfile );

			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "DISP" );
			(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
				&g_dispenabled );
			(*(*attributeApproachElement).FirstChildElement( "pixWidth" )).QueryIntText( 
				&g_disppixwidth );
			(*(*attributeApproachElement).FirstChildElement( "pixHeight" )).QueryIntText( 
				&g_disppixheight );
			(*(*attributeApproachElement).FirstChildElement( "updateFPS" )).QueryIntText( 
				&g_dispupdatefps );

			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "COMM" );
			(*(*attributeApproachElement).FirstChildElement( "pollRateGPS" )).QueryIntText( 
				&g_commpollrategps );
			(*(*attributeApproachElement).FirstChildElement( "pollRateLIDAR" )).QueryIntText( 
				&g_commpollratelidar );
			(*(*attributeApproachElement).FirstChildElement( "pollRateGpio" )).QueryIntText( 
				&g_commpollrategpio );	

			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "LDW" );	
			(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
				&g_ldwenabled );
			(*(*attributeApproachElement).FirstChildElement( "enableSpeed" )).QueryIntText( 
				&g_ldwenablespeed );
			(*(*attributeApproachElement).FirstChildElement( "samplesToKeep" )).QueryIntText( 
				&g_ldwsamplestokeep );
			(*(*attributeApproachElement).FirstChildElement( "samplesToAverage" )).QueryIntText( 
				&g_ldwsamplestoaverage );
			(*(*attributeApproachElement).FirstChildElement( "perOffsetWarning" )).QueryIntText( 
				&g_ldwperoffsetwarning );	
			(*(*attributeApproachElement).FirstChildElement( "perOffsetAlarm" )).QueryIntText( 
				&g_ldwperoffsetalarm );	
			(*(*attributeApproachElement).FirstChildElement( "msUntilWarning" )).QueryIntText( 
				&g_ldwmsuntilwarning );		
			(*(*attributeApproachElement).FirstChildElement( "msUntilAlarm" )).QueryIntText( 
				&g_ldwmsuntilalarm );		
			(*(*attributeApproachElement).FirstChildElement( "updateFPS" )).QueryIntText( 
				&g_ldwupdatefps );		
				
			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "FCW" );
			(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
				&g_fcwenabled );
			(*(*attributeApproachElement).FirstChildElement( "samplesToAverage" )).QueryIntText( 
				&g_fcwsamplestoaverage );
			(*(*attributeApproachElement).FirstChildElement( "distanceOffset" )).QueryDoubleText( 
				&g_fcwdistanceoffset );
			(*(*attributeApproachElement).FirstChildElement( "msFollowDistWarning" )).QueryIntText( 
				&g_fcwmsfollowdistwarning );
			(*(*attributeApproachElement).FirstChildElement( "msFollowDistAlarm" )).QueryIntText( 
				&g_fcwmsfollowdistalarm );
			(*(*attributeApproachElement).FirstChildElement( "msCollisionWarning" )).QueryIntText( 
				&g_fcwmscollisionwarning );
			(*(*attributeApproachElement).FirstChildElement( "msCollisionAlarm" )).QueryIntText( 
				&g_fcwmscollisionalarm );

			attributeApproachElement = doc.FirstChildElement()->
				FirstChildElement( "GPIO" );
			(*(*attributeApproachElement).FirstChildElement( "enabled" )).QueryBoolText( 
				&g_gpioenabled );
			(*(*attributeApproachElement).FirstChildElement( "autoShutdown" )).QueryBoolText( 
				&g_gpioautoshutdown );		
		} catch ( exception E ) {
			return -2;	
		}
	}
		
	return 1;
}

//Initialize all namespace constants
namespace settings{
	const int kreadsuccess(ReadXmlSettings());
	namespace gen {
		const bool kenbuzzer{g_genenbuzzer};
		const bool kdebugscreen{g_gendebugscreen};
		const bool kdebugterminal{g_gendebugterminal};
	}
	namespace cam{
		const bool krecordorgimage{g_camrecordorgimage};
		const std::string kfilepath{g_camfilepath};
		const std::string korgfilename{g_camorgfilename};
		const std::string kmodfilename{g_cammodfilename};
		const int kpixwidth{g_campixwidth};
		const int kpixheight{g_campixheight};
		const bool kshowspeed{g_camshowspeed};
		const bool kshowloc{g_camshowloc};
		const bool kshadelanes{g_camshadelanes};
		const int krecfps{g_camrecfps};
		const int kfilestokeep{g_camfilestokeep};
		const int kminperfile{g_camminperfile};
	}
	namespace disp{
		const bool kenabled{g_dispenabled};
		const int kpixwidth{g_disppixwidth};
		const int kpixheight{g_disppixheight};
		const int kupdatefps{g_dispupdatefps};
	}
	namespace comm{
		const int kpollrategps{g_commpollrategps};
		const int kpollratelidar{g_commpollratelidar};
		const int kpollrategpio{g_commpollrategpio};
	}
	namespace gps{
		const int ksamplestoaverage{g_gpssamplestoaverage};
	}
	namespace ldw{
		const bool kenabled{g_ldwenabled};
		const int kenablespeed{g_ldwenablespeed};
		const int ksamplestokeep{g_ldwsamplestokeep};
		const int ksamplestoaverage{g_ldwsamplestoaverage};
		const int kperoffsetwarning{g_ldwperoffsetwarning};
		const int kperoffsetalarm{g_ldwperoffsetalarm};
		const int kmsuntilwarning{g_ldwmsuntilwarning};
		const int kmsuntilalarm{g_ldwmsuntilalarm};
		const int kupdatefps{g_ldwupdatefps};
	}
	namespace fcw{
		const bool kenabled{g_fcwenabled};
		const int ksamplestoaverage{g_fcwsamplestoaverage};
		const double kdistanceoffset{g_fcwdistanceoffset};
		const int kmsfollowdistwarning{g_fcwmsfollowdistwarning};
		const int kmsfollowdistalarm{g_fcwmsfollowdistalarm};
		const int kmscollisionwarning{g_fcwmscollisionwarning};
		const int kmscollisionalarm{g_fcwmscollisionalarm};
	}
	namespace gpio{
		const bool kenabled{g_gpioenabled};
		const bool kautoshutdown{g_gpioautoshutdown};
	}
}
