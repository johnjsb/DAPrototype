/******************************************************************************************
  Date:    12.08.2016
  Author:  Nathan Greco (Nathan.Greco@gmail.com)

  Project:
      DAPrototype: Driver Assist Prototype
	  http://github.com/NateGreco/DAPrototype.git

  License:
	  This software is licensed under GNU GPL v3.0
	  
******************************************************************************************/

//Standard libraries
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <deque>
#include <algorithm>
#include <math.h>

//3rd party libraries
#include "opencv2/core/core.hpp"

//Project libraries
#include "lane_detect_constants.h"
#include "lane_detect_processor.h"

//Preprocessor
#ifndef M_PI
    #define M_PI 3.14159265359f
#endif
#ifndef M_PI_2
    #define M_PI_2 1.57079632679f
#endif
#ifndef M_PI_4
    #define M_PI_4 0.78539816339f
#endif
#ifndef M_1_PI
	#define M_1_PI 0.31830988618f
#endif
#define DEGREESPERRADIAN 57.2957795131f

/*****************************************************************************************/
//Main function
void ProcessImage ( cv::Mat& image,
                    Polygon& polygon )
{
//-----------------------------------------------------------------------------------------
//Image manipulation
//-----------------------------------------------------------------------------------------
	//Create mat of ROI to improve performance
	cv::Mat houghlinesmat{ image.size(), image.type(), cv::Scalar(0) };
	image( cv::Rect(0,
					lanedetectconstants::k_ystartposition,
					image.cols,
					image.rows -
					lanedetectconstants::k_ystartposition)).copyTo(houghlinesmat(cv::Rect(0,
					lanedetectconstants::k_ystartposition,
					image.cols,
					image.rows -
					lanedetectconstants::k_ystartposition)));
					
	//Change to grayscale
	cv::cvtColor( houghlinesmat, houghlinesmat, CV_BGR2GRAY );
	
	//Blur to reduce noise
    cv::blur( houghlinesmat(cv::Rect(0,
									 lanedetectconstants::k_ystartposition,
									 houghlinesmat.cols,
									 houghlinesmat.rows -
									 lanedetectconstants::k_ystartposition)),
			  houghlinesmat(cv::Rect(0,
									 lanedetectconstants::k_ystartposition,
									 houghlinesmat.cols,
									 houghlinesmat.rows -
									 lanedetectconstants::k_ystartposition)),
			  cv::Size(3,3) );
	
	//Auto threshold values for canny edge detection
	cv::Scalar mean;     
	cv::Scalar std;
	cv::meanStdDev( houghlinesmat(cv::Rect(0,
										   lanedetectconstants::k_ystartposition,
										   houghlinesmat.cols,
										   houghlinesmat.rows -
										   lanedetectconstants::k_ystartposition)),
					mean,
					std );
	double lowerthreshold{ lanedetectconstants::k_contrastscalefactor * std[0] };
	
	//Canny edge detection
    cv::Canny( houghlinesmat, houghlinesmat, lowerthreshold, 3 * lowerthreshold );

//-----------------------------------------------------------------------------------------
//Use Probalistic Hough Lines
//-----------------------------------------------------------------------------------------
	//Probalistic Houghlines
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP( houghlinesmat,
					 lines,
					 lanedetectconstants::k_rho,
					 lanedetectconstants::k_theta,
					 lanedetectconstants::k_threshold,
					 lanedetectconstants::k_minimumsize,
					 lanedetectconstants::k_maxlinegap );

//-----------------------------------------------------------------------------------------
//Filter and sort lines
//-----------------------------------------------------------------------------------------	
	std::vector<EvaluatedLine> evaluatedlines;
	for ( cv::Vec4i &line : lines ) {
		EvaluateLine( line, evaluatedlines );
	}

//-----------------------------------------------------------------------------------------
//Filter and sort all evaluated lines
//-----------------------------------------------------------------------------------------	
	std::vector<EvaluatedLine> leftlines;
	std::vector<EvaluatedLine> rightlines;
	SortLines( evaluatedlines, houghlinesmat.cols, leftlines, rightlines );

//-----------------------------------------------------------------------------------------
//Find highest scoring pair of lines
//-----------------------------------------------------------------------------------------	
	Polygon bestpolygon( lanedetectconstants::defaultpolygon );
	float maxscore{ lanedetectconstants::k_lowestscorelimit };
	EvaluatedLine leftline;
	EvaluatedLine rightline;
	
	//Find best score
	for ( EvaluatedLine &leftevaluatedline : leftlines ) {
		for ( EvaluatedLine &rightevaluatedline : rightlines ) {
			//Create polygon
			Polygon newpolygon( lanedetectconstants::defaultpolygon );
			FindPolygon( newpolygon,
						 leftevaluatedline,
						 rightevaluatedline,
						 houghlinesmat.cols,
						 houghlinesmat.rows );
				
			//If invalid polygon created, goto next
			if ( newpolygon == lanedetectconstants::defaultpolygon) continue;
			
			//Score
			float score{ Score(newpolygon,
						 leftevaluatedline,
						 rightevaluatedline,
						 houghlinesmat.cols) };
			
			//If highest score update
			if ( score > maxscore ) {
				leftline = leftevaluatedline;
				rightline = rightevaluatedline;
				maxscore = score;
				bestpolygon = newpolygon;
			}
		}
	}

	//Set bottom of polygon equal to optimal polygon
	if ( bestpolygon != lanedetectconstants::defaultpolygon) {
		FindPolygon( bestpolygon,
					 leftline,
					 rightline,
					 houghlinesmat.cols,
					 houghlinesmat.rows,
					 true );
	}
	
//-----------------------------------------------------------------------------------------
//Return results
//-----------------------------------------------------------------------------------------	
	std::copy( std::begin(bestpolygon),
			   std::end(bestpolygon),
			   std::begin(polygon) );
	return;
}

/*****************************************************************************************/	
bool CheckAngle( const cv::Point center,
				 const float angle )
{
	//Get angle fron contour center to vanishing point
	float vanishingpointangle{ FastArcTan2((lanedetectconstants::k_vanishingpointy -
											 center.y),
											(lanedetectconstants::k_vanishingpointx -
											 center.x)) };
	if (vanishingpointangle < 0.0f) {
		vanishingpointangle += 180.0f;
	}

	//Check difference against limit and return result
	if ( fabs(angle - vanishingpointangle) >
		 lanedetectconstants::k_maxvanishingpointangle ) {
		return true;
	} else {
		return false;
	}
}

/*****************************************************************************************/	
void EvaluateLine( const cv::Vec4i& line,
				   std::vector<EvaluatedLine>& evaluatedlines )
{	
	//Calculate center point
	cv::Point center{ cv::Point((line[0] + line[2]) / 2, (line[1] + line[3]) / 2) };
									
	//Filter by screen position
	if ( center.y < (lanedetectconstants::k_verticallimit)) return;

	//Filter by angle
	float angle{ FastArcTan2(line[3] - line[1], line[2] - line[0]) };
	if (angle < 0.0f) {
		angle += 180.0f;
	}
	
	//Check that angle points to vanishing point
	if ( CheckAngle(center, angle) ) return;

	evaluatedlines.push_back( EvaluatedLine{line, angle, center} );
	return;
}

/*****************************************************************************************/
void SortLines( const std::vector<EvaluatedLine>& evaluatedlines,
			    const int imagewidth,
			    std::vector<EvaluatedLine>& leftlines,
			    std::vector<EvaluatedLine>& rightlines )
{
	for ( const EvaluatedLine &evaluatedline : evaluatedlines ) {
		//Push into either left or right evaluated line set
		if ( evaluatedline.center.x < (imagewidth * 0.6f) ) {
			leftlines.push_back( evaluatedline );
		} 
		if ( evaluatedline.center.x > (imagewidth * 0.4f) ) {
			rightlines.push_back( evaluatedline );
		}
	}
	return;
}

/*****************************************************************************************/
void FindPolygon( Polygon& polygon,
                  const EvaluatedLine& leftevaluatedline,
				  const EvaluatedLine& rightevaluatedline,
                  const int imagewidth,
                  const int imageheight,
				  bool useoptimaly )
{
	//Check for correct left/right assignment
	if ( leftevaluatedline.center.x > rightevaluatedline.center.x ) return;
	
	//Define slopes
	float leftslopeinverse, rightslopeinverse;
	//Check for undefined slopes
	if ( leftevaluatedline.line[3] != leftevaluatedline.line[1] ) {
		leftslopeinverse = static_cast<float>(leftevaluatedline.line[2] -
											  leftevaluatedline.line[0]) /
						   static_cast<float>(leftevaluatedline.line[3] -
											  leftevaluatedline.line[1]);
	} else {
		if ( leftevaluatedline.line[2] > leftevaluatedline.line[0] ) {
			leftslopeinverse = FLT_MAX;
		} else if ( leftevaluatedline.line[2] < leftevaluatedline.line[0] ) {
			leftslopeinverse = -FLT_MAX;
		} else {
			leftslopeinverse = 0.0f;
		}
	}
	if ( rightevaluatedline.line[3] != rightevaluatedline.line[1] ) {
		rightslopeinverse = static_cast<float>(rightevaluatedline.line[2] -
											   rightevaluatedline.line[0]) /
							static_cast<float>(rightevaluatedline.line[3] -
											   rightevaluatedline.line[1]);
	} else {
		if ( rightevaluatedline.line[2] > rightevaluatedline.line[0] ) {
			rightslopeinverse = FLT_MAX;
		} else if ( rightevaluatedline.line[2] < rightevaluatedline.line[0] ) {
			rightslopeinverse = -FLT_MAX;
		} else {
			rightslopeinverse = 0.0f;
		}
	}

	//Check shape before continuing
	if ( (leftslopeinverse > 0.0f) && (rightslopeinverse < 0.0f) ) return;
	
	//Calculate optimal bottom points
	cv::Point bottomleftoptimal{ cv::Point(leftevaluatedline.center.x + 
										   (imageheight - leftevaluatedline.center.y) *
										   leftslopeinverse,
										   imageheight) };
	cv::Point bottomrightoptimal{ cv::Point(rightevaluatedline.center.x +
										    (imageheight - rightevaluatedline.center.y) *
											rightslopeinverse,
											imageheight) };

	//Filter by max offset from center
	if ( (abs(bottomleftoptimal.x + bottomrightoptimal.x - imagewidth) / 2) >
		 lanedetectconstants::k_maxoffsetfromcenter ) return; 

	//Filter based on width of polygon
	int roadwidth{ bottomrightoptimal.x - bottomleftoptimal.x };
	if ( roadwidth < lanedetectconstants::k_minroadwidth ) return;
	if ( roadwidth > lanedetectconstants::k_maxroadwidth ) return;
	
	//Get point extremes
	int maxyleft{ std::max(leftevaluatedline.line[1],
							  leftevaluatedline.line[3]) };
	int maxyright{ std::max(rightevaluatedline.line[1],
							  rightevaluatedline.line[3]) };
	int minyleft{ std::min(leftevaluatedline.line[1],
							  leftevaluatedline.line[3]) };
	int minyright{ std::min(rightevaluatedline.line[1],
							  rightevaluatedline.line[3]) };
	int maxyactual{ std::max(maxyleft, maxyright) };
	int miny{ std::min(minyleft, minyright) };
	if ( miny < (imageheight / 2) ) miny = imageheight / 2; 
	int maxy;	
	if ( useoptimaly ) {
		maxy = imageheight;
	} else {
		maxy = maxyactual;
	}
	
	//Handle polygon intersection
	if ( polygon[0].x > polygon[1].x ) return;
	
	//Construct polygon
	if ( useoptimaly ) {
		polygon[0] = bottomleftoptimal;
		polygon[1] = bottomrightoptimal;
	} else {
		polygon[0] = cv::Point(leftevaluatedline.center.x +
							   (maxy - leftevaluatedline.center.y) *
							   leftslopeinverse,
							   maxy);
		polygon[1] = cv::Point(rightevaluatedline.center.x +
							   (maxy - rightevaluatedline.center.y) *
							   rightslopeinverse,
							   maxy);
	}
	polygon[2] = cv::Point( rightevaluatedline.center.x -
							(rightevaluatedline.center.y - miny) *
							rightslopeinverse,
							miny );
	polygon[3] = cv::Point( leftevaluatedline.center.x -
							(leftevaluatedline.center.y - miny) *
							leftslopeinverse,
							miny );
							
	//Handle polygon intersection
	if ( polygon[3].x > polygon[2].x ) {
		//Check for invalid slopes
		if ( (leftslopeinverse == 0.0f) || (rightslopeinverse == 0.0f) ) return;
		//Use intersection point for both - y=mx+b
		float bleft{ leftevaluatedline.center.y -
					 leftevaluatedline.center.x / leftslopeinverse };
		float bright{ rightevaluatedline.center.y -
					 rightevaluatedline.center.x / rightslopeinverse };
		int x{ static_cast<int>((bright - bleft) /
								((1.0f / leftslopeinverse) -
								 (1.0f / rightslopeinverse))) };
		int y{ static_cast<int>(((1.0f / leftslopeinverse) * x) + bleft) };
		polygon[3] = polygon[2] = cv::Point(x, y);
	}

	return;
}

/*****************************************************************************************/
float Score( const Polygon& polygon,
             const EvaluatedLine& leftevaluatedline,
			 const EvaluatedLine& rightevaluatedline,
			 const int imagewidth )
{
	float heightwidthratio{ static_cast<float>(polygon[0].y - polygon[3].y) /
							static_cast<float>(polygon[1].x - polygon[0].x) };
	float centeroffset{ static_cast<float>(fabs((imagewidth -
												(polygon[0].x + polygon[1].x)) *
												0.5f)) };
	float angleoffset{ 0.5f * static_cast<float>(fabs(180.0f -
													  leftevaluatedline.angle -
													   rightevaluatedline.angle)) };
	
	return lanedetectconstants::k_weightedheightwidth * heightwidthratio +
		   lanedetectconstants::k_weightedangleoffset * angleoffset +
		   lanedetectconstants::k_weightedcenteroffset * centeroffset;
}

/*****************************************************************************************/
void AveragePolygon ( Polygon& polygon,
                      std::deque<Polygon>& pastpolygons,
					  const int samplestoaverage,
					  const int samplestokeep )
{
	//FIFO
	pastpolygons.push_back( polygon );
	if ( pastpolygons.size() > samplestokeep ) {
		pastpolygons.pop_front();
	}

	//Sum nonzero
	Polygon averagepolygon( lanedetectconstants::defaultpolygon );
	int nonzerocount{0};
	for ( Polygon &polygon : pastpolygons ) {
		if ( polygon == lanedetectconstants::defaultpolygon) continue;
		nonzerocount++;
		for ( int i = 0; i < polygon.size(); i++ ) {
			averagepolygon[i].x += polygon[i].x;
			averagepolygon[i].y += polygon[i].y;
		}
	}	
	if ( nonzerocount == 0 ) return;

	//Average nonzero
	for ( int i = 0; i < polygon.size(); i++ ) {
		averagepolygon[i].x /= nonzerocount;
		averagepolygon[i].y /= nonzerocount;
	}
	
	//if not enough nonzero polygons, return
	if ( nonzerocount <= samplestoaverage ) {
		std::copy( std::begin(averagepolygon),
				   std::end(averagepolygon),
				   std::begin(polygon) );
		return;
	}

	//Find differences
	std::vector<PolygonDifferences> polygondifferences;
	for ( Polygon &ipolygon : pastpolygons ) {
		polygondifferences.push_back( PolygonDifferences{ipolygon,
														 abs(averagepolygon[0].x -
															 ipolygon[0].x) + 
														 abs(averagepolygon[1].x -
															 ipolygon[1].x)} );
	}

	//Sort
	sort( polygondifferences.begin(),
		  polygondifferences.end(),
		  [](const PolygonDifferences& a,
			 const PolygonDifferences& b )
		  { return a.differencefromaverage < b.differencefromaverage; } );

	//Sum closest values
	averagepolygon = lanedetectconstants::defaultpolygon;
	for ( int i = 0; i < samplestoaverage; i++ ) {
		for (int j = 0; j < 4; j++) {
			averagepolygon[j].x += polygondifferences[i].polygon[j].x;
			averagepolygon[j].y += polygondifferences[i].polygon[j].y;
		}
	}
	//Average closest values
	for ( int i = 0; i < polygon.size(); i++ ) {
		averagepolygon[i].x /= samplestoaverage;
		averagepolygon[i].y /= samplestoaverage;
	}
	std::copy( std::begin(averagepolygon),
			   std::end(averagepolygon),
			   std::begin(polygon));
	return;
}

/*****************************************************************************************/
float FastArcTan2( const float y,
				   const float x )
{
	//Check if 90 or 0
	if ( y == 0.0f ) return 0.0f;
	if ( x == 0.0f ) return 90.0f;

	//Calculate
	float a( std::min(fabs(x),fabs(y)) / std::max(fabs(x),fabs(y)) );
	float s{ a * a };
	float angle( (((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a) );
	if ( fabs(y) > fabs(x) ) angle = M_PI_2 - angle;
	if ( x < 0 ) angle = M_PI - angle;
	if ( y < 0 ) angle *= -1.0;
	
	//Convert from radians
	angle *= DEGREESPERRADIAN;
	if ( angle < 0 ) angle += 180.0;
	
	//return
	return angle;
}
