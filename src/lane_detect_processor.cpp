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
namespace lanedetectconstants {
	//Image evaluation
	float k_contrastscalefactor{ 0.26f };
	
	//Segment filtering
	uint16_t k_segmentminimumsize{ 30 };			//Relative to image size, must change
	uint16_t k_verticalsegmentlimit{ 250 };			//Relative to image size, must change
	float k_segmentminimumangle{ 20.0f };
	//float k_segmentlengthwidthratio{ 3.5f };
	
	//Contour construction filter
	float k_segmentsanglewindow{ 34.0f };
	
	//Contour filtering
	uint16_t k_minimumsize{ 36 };					//Relative to image size, must change
	float k_minimumangle{ 24.0f };
	//float k_lengthwidthratio{ 4.5f };
	
	//Polygon filtering
    uint16_t k_minroadwidth{ 400 };					//Relative to image size, must change
    uint16_t k_maxroadwidth{ 680 };					//Relative to image size, must change
	
	//Scoring
	float k_anglefromcenter{ 30.0f };
	uint16_t k_minimumpolygonheight{ 12 };			//Relative to image size, must change
	float k_lowestscorelimit{ -50.0f };				//Relative to image size, must change
	float k_heightwidthscalefactor{ 520.0f };		//Relative to image size, must change

}

//Main function
void ProcessImage ( cv::Mat& image,
                    Polygon& polygon )
{
//-----------------------------------------------------------------------------------------
//Image manipulation
//-----------------------------------------------------------------------------------------
	//Change to grayscale
	cv::cvtColor( image, image, CV_BGR2GRAY );
	
	//Blur to reduce noise
    cv::blur( image, image, cv::Size(3,3) );
	
//-----------------------------------------------------------------------------------------
//Find contours
//-----------------------------------------------------------------------------------------
	//Auto threshold values for canny edge detection
	cv::Scalar mean;     
	cv::Scalar std;
	cv::meanStdDev(image, mean, std);
	double lowerthreshold{ lanedetectconstants::k_contrastscalefactor * std[0] };
	
	//Canny edge detection
    cv::Canny( image, image, lowerthreshold, 3 * lowerthreshold );
	//int erosion_size = 1;   
	//cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS,
	//					  cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), 
	//					  cv::Point(erosion_size, erosion_size) );
	//Dilate
	//cv::dilate( image, image, element );
	//Erode
	//cv::erode( image, image, element );
	std::vector<Contour> detectedcontours;
    std::vector<cv::Vec4i> detectedhierarchy;
    cv::findContours( image,
					  detectedcontours,
					  detectedhierarchy,
					  CV_RETR_CCOMP,
					  CV_CHAIN_APPROX_SIMPLE );
		
//-----------------------------------------------------------------------------------------
//Evaluate contours
//-----------------------------------------------------------------------------------------	
	std::vector<EvaluatedContour> evaluatedchildsegments;
	std::vector<EvaluatedContour> evaluatedparentsegments; 
    for ( int i = 0; i < detectedcontours.size(); i++ ) {
        if ( detectedhierarchy[i][3] > -1 ) {
			EvaluateSegment( detectedcontours[i], evaluatedchildsegments );
        } else {
			EvaluateSegment( detectedcontours[i], evaluatedparentsegments );
		}
    }

//-----------------------------------------------------------------------------------------
//Construct from segments
//-----------------------------------------------------------------------------------------	
    //std::vector<std::vector<cv::Point>> constructedcontours;
	//ConstructFromSegments( evaluatedchildsegments, constructedcontours );

//-----------------------------------------------------------------------------------------
//Evaluate constructed segments
//-----------------------------------------------------------------------------------------	
	//for ( Contour contour : constructedcontours ) {
	//	EvaluateSegment( contour, evaluatedparentsegments );
	//}
	
//-----------------------------------------------------------------------------------------
//Filter and sort all evaluated contours
//-----------------------------------------------------------------------------------------	
	std::vector<EvaluatedContour> leftcontours;
	std::vector<EvaluatedContour> rightcontours;
	SortContours( evaluatedparentsegments, image.cols, leftcontours, rightcontours );
	SortContours( evaluatedchildsegments, image.cols, leftcontours, rightcontours );
	
//-----------------------------------------------------------------------------------------
//Find highest scoring pair of contours
//-----------------------------------------------------------------------------------------	
	Polygon bestpolygon{ cv::Point(0,0),
						 cv::Point(0,0),
						 cv::Point(0,0),
						 cv::Point(0,0) };
	float maxscore{ lanedetectconstants::k_lowestscorelimit };
	EvaluatedContour leftcontour;
	EvaluatedContour rightcontour;
	
	//Find best score
	for ( EvaluatedContour &leftevaluatedcontour : leftcontours ) {
		for ( EvaluatedContour &rightevaluatedcontour : rightcontours ) {
			//Check sum angle
			if ( (fabs(180.0f - leftevaluatedcontour.angle - rightevaluatedcontour.angle) *
				  0.5f) > lanedetectconstants::k_anglefromcenter ) continue;
			
			Polygon newpolygon{ cv::Point(0,0),
								cv::Point(0,0),
								cv::Point(0,0),
								cv::Point(0,0) };
			FindPolygon( newpolygon,
						 leftevaluatedcontour,
						 rightevaluatedcontour,
						 image.rows );
				
			//If invalid polygon created, goto next
			if ( newpolygon[0] == cv::Point(0,0) ) continue;
			
			//Score
			float score{ Score(newpolygon, image.cols) };
			
			//If highest score update
			if ( score > maxscore ) {
				leftcontour = leftevaluatedcontour;
				rightcontour = rightevaluatedcontour;
				maxscore = score;
				bestpolygon = newpolygon;
			}
		}
	}

	//Set bottom of polygon equal to optimal polygon
	if ( bestpolygon[0] != cv::Point(0,0) ) {
		FindPolygon( bestpolygon, leftcontour, rightcontour, image.rows, true );
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
void EvaluateSegment( const Contour& contour,
					  std::vector<EvaluatedContour>& evaluatedsegments )
{	
	//Filter by size, only to prevent exception when creating ellipse or fitline
	if ( contour.size() < lanedetectconstants::k_segmentminimumsize ) return;
		
	//Calculate center point
	cv::Point center { std::accumulate(contour.begin(),	contour.end(), cv::Point(0,0)) };
	center = cv::Point(center.x / contour.size(), center.y / contour.size());
									
	//Filter by screen position
	if ( center.y < (lanedetectconstants::k_verticalsegmentlimit)) return;
	
	//Create ellipse
	//cv::RotatedRect ellipse{ fitEllipse(contour) };
	
	//Filter by length (ellipse vs segment?)
	//if ( ellipse.size.height < lanedetectconstants::k_segmentminimumsize ) return;
	
	//Calculate length to width ratio
	//float lengthwidthratio{ ellipse.size.height / ellipse.size.width };
	
	//Filter by length to width ratio
	//if ( lengthwidthratio < lanedetectconstants::k_segmentlengthwidthratio ) return;

	//Create fitline
	cv::Vec4f fitline;
	cv::fitLine(contour, fitline, CV_DIST_L2, 0, 0.1, 0.1 );

	//Filter by angle
	float angle{ FastArcTan2(fitline[1], fitline[0]) };
	if (angle < 0.0f) {
		angle += 180.0f;
	}
	
	if (angle < 90.0f) {
		if ( angle < lanedetectconstants::k_segmentminimumangle ) return;
	} else {
		if ( angle > (180.0f - lanedetectconstants::k_segmentminimumangle) ) return;
	}

	evaluatedsegments.push_back( EvaluatedContour{contour,
	//											  ellipse,
	//											  lengthwidthratio,
												  angle,
												  fitline,
												  center} );
	return;
}

/*****************************************************************************************/	
void ConstructFromSegments( const  std::vector<EvaluatedContour>& evaluatedsegments,
                            std::vector<Contour>& constructedcontours )
{
    for ( const EvaluatedContour &segcontour1 : evaluatedsegments ) {
		for ( const EvaluatedContour &segcontour2 : evaluatedsegments ) {
			if ( segcontour1.fitline == segcontour2.fitline ) continue;
			float angledifference1( fabs(segcontour1.angle -	segcontour2.angle) );
			if ( angledifference1 > lanedetectconstants::k_segmentsanglewindow ) continue;
			float createdangle { FastArcTan2((segcontour1.center.y -
											  segcontour2.center.y),
											 (segcontour1.center.x -
											  segcontour2.center.x)) };
			if ( createdangle < 90.0f ) {
				if ( createdangle < lanedetectconstants::k_segmentminimumangle ) return;
			} else {
				if ( createdangle > (180.0f -
					 lanedetectconstants::k_segmentminimumangle) ) return;
			}
			float angledifference2( fabs(createdangle -	segcontour1.angle) );
			if ( angledifference2 > lanedetectconstants::k_segmentsanglewindow ) continue;
			float angledifference3( fabs(createdangle -	segcontour2.angle) );
			if ( angledifference3 > lanedetectconstants::k_segmentsanglewindow ) continue;
			Contour newcontour{ segcontour1.contour };
			newcontour.insert( newcontour.end(),
							   segcontour2.contour.begin(),
							   segcontour2.contour.end() );
			constructedcontours.push_back( newcontour );
		}
    }	
	return;
}

/*****************************************************************************************/
void SortContours( const std::vector<EvaluatedContour>& evaluatedsegments,
                   const int imagewidth,
				   std::vector<EvaluatedContour>& leftcontours,
				   std::vector<EvaluatedContour>& rightcontours )
{
	for ( const EvaluatedContour &evaluatedcontour : evaluatedsegments ) {
		//Filter by length (ellipse vs segment?)
		//if ( evaluatedcontour.ellipse.size.height < lanedetectconstants::k_minimumsize )
		//	continue;
		if ( evaluatedcontour.contour.size() < lanedetectconstants::k_minimumsize ) {
			continue;
		}
		
		//Filter by length to width ratio
		//if ( evaluatedcontour.lengthwidthratio < lanedetectconstants::k_lengthwidthratio )
		//	continue;
		
		//Push into either left or right evaluated contour set
		if ( evaluatedcontour.center.x < (imagewidth * 0.6f) ) {
			//Filter by angle
			if ( evaluatedcontour.angle > (180.0f - lanedetectconstants::k_minimumangle) ) {
				continue;
			}
			if ( evaluatedcontour.angle < 75.0f ) continue;
			leftcontours.push_back( evaluatedcontour );
		} 
		if ( evaluatedcontour.center.x > (imagewidth * 0.4f) ) {
			//Filter by angle
			if ( evaluatedcontour.angle < lanedetectconstants::k_minimumangle) continue;
			if ( evaluatedcontour.angle > 105.0f ) continue;
			rightcontours.push_back( evaluatedcontour );
		}
	}
	return;
}

/*****************************************************************************************/
void FindPolygon( Polygon& polygon,
                  const EvaluatedContour& leftcontour,
				  const EvaluatedContour& rightcontour,
                  const int imageheight,
				  bool useoptimaly )
{
	//Check for correct left/right assignment
	if ( leftcontour.center.x > rightcontour.center.x ) return;
	
	//Define slopes
	float leftslopeinverse { leftcontour.fitline[0] / leftcontour.fitline[1] };
	float rightslopeinverse { rightcontour.fitline[0] / rightcontour.fitline[1] };
	
	//Check shape before continuing
	if ( (leftslopeinverse > 0.0f) && (rightslopeinverse < 0.0f) ) return;
	
	//Calculate optimal bottom points
	cv::Point bottomleftoptimal{ cv::Point(leftcontour.center.x + 
										   (imageheight - leftcontour.center.y) *
										   leftslopeinverse,
										   imageheight) };
	cv::Point bottomrightoptimal{ cv::Point(rightcontour.center.x +
										    (imageheight - rightcontour.center.y) *
											rightslopeinverse,
											imageheight) };
	
	//Perform filtering based on width of polygon with optimal maxy
	int roadwidth{ bottomrightoptimal.x - bottomleftoptimal.x };
	if ( roadwidth < lanedetectconstants::k_minroadwidth ) return;
	if ( roadwidth > lanedetectconstants::k_maxroadwidth ) return;
	
	//Get point extremes
	auto minmaxyleft = std::minmax_element( leftcontour.contour.begin(),
											leftcontour.contour.end(),
											[]( const cv::Point& lhs,
												const cv::Point& rhs )
											{ return lhs.y < rhs.y; } );
	auto minmaxyright = std::minmax_element( rightcontour.contour.begin(),
											 rightcontour.contour.end(),
											 []( const cv::Point& lhs,
												 const cv::Point& rhs )
											 { return lhs.y < rhs.y; } );
	int maxyactual{ std::max(minmaxyleft.second->y, minmaxyright.second->y) };
	int miny{ std::max(minmaxyleft.first->y, minmaxyright.first->y) };
	int maxy;	
	if ( useoptimaly ) {
		maxy = imageheight;
	} else {
		maxy = maxyactual;
	}
	
	//Filter by height
	if ( (maxyactual - miny) < lanedetectconstants::k_minimumpolygonheight ) return;
	
	cv::Point topright{ cv::Point(rightcontour.center.x -
								  (rightcontour.center.y - miny) *
								  rightslopeinverse,
								  miny) };
	cv::Point topleft{ cv::Point(leftcontour.center.x -
								 (leftcontour.center.y - miny) *
								 leftslopeinverse,
								 miny) };
		
	//Construct polygon
	if ( useoptimaly ) {
		polygon[0] = bottomleftoptimal;
		polygon[1] = bottomrightoptimal;
	} else {
		polygon[0] = cv::Point(leftcontour.center.x +
							   (maxy - leftcontour.center.y) *
							   leftslopeinverse,
							   maxy);
		polygon[1] = cv::Point(rightcontour.center.x +
							   (maxy - rightcontour.center.y) *
							   rightslopeinverse,
							   maxy);
	}
	polygon[2] = topright;
	polygon[3] = topleft;

	return;
}

/*****************************************************************************************/
float Score( const Polygon& polygon ,
			 const int imagewidth )
{
	
	float heightwidthratio{ static_cast<float>(polygon[0].y - polygon[3].y) /
							static_cast<float>(polygon[1].x - polygon[0].x) };
	float centeroffset{ static_cast<float>(fabs((imagewidth -
												(polygon[0].x + polygon[1].x)) *
												0.5f)) };
	
	return lanedetectconstants::k_heightwidthscalefactor * heightwidthratio - centeroffset;
}

/*****************************************************************************************/
void AveragePolygon ( Polygon& polygon,
                      std::deque<Polygon>& pastpolygons,
					  int samplestoaverage,
					  int samplestokeep )
{
	//FIFO
	pastpolygons.push_back( polygon );
	if ( pastpolygons.size() > samplestokeep ) {
		pastpolygons.pop_front();
	}

	//Sum nonzero
	Polygon averagepolygon { cv::Point(0,0),
							 cv::Point(0,0),
							 cv::Point(0,0),
							 cv::Point(0,0) };
	int nonzerocount{0};
	for ( Polygon &ipolygon : pastpolygons ) {
		if ( ipolygon[0] == cv::Point(0,0) ) continue;
		nonzerocount++;
		for ( int i = 0; i < ipolygon.size(); i++ ) {
			averagepolygon[i].x += ipolygon[i].x;
			averagepolygon[i].y += ipolygon[i].y;
		}
	}	
	if ( nonzerocount == 0 ) return;

	//Average nonzero
	for ( int i = 0; i < polygon.size(); i++ ) {
		averagepolygon[i].x /= nonzerocount;
		averagepolygon[i].y /= nonzerocount;
	}
	
	//if not enough nonzero polygons, return
	if ( nonzerocount < samplestoaverage ) {
		std::copy( std::begin(averagepolygon),
				   std::end(averagepolygon),
				   std::begin(polygon) );
		return;
	}

	//Find differences
	std::vector<PolygonDifferences> polygondifferences;
	for ( Polygon &ipolygon : pastpolygons ) {
		float differencefromaverage{0.0f};
		for ( int i = 0; i < ipolygon.size(); i++ ) {
			differencefromaverage += fabs(averagepolygon[i].x - ipolygon[i].x);
			differencefromaverage += fabs(averagepolygon[i].y - ipolygon[i].y);
		}
		polygondifferences.push_back( PolygonDifferences { ipolygon,
														   differencefromaverage } );
	}

	//Sort
	sort( polygondifferences.begin(),
		  polygondifferences.end(),
		  [](const PolygonDifferences& a,
			 const PolygonDifferences& b )
		  { return a.differencefromaverage < b.differencefromaverage; } );

	//Sum closest values
	averagepolygon = { cv::Point(0,0),
					   cv::Point(0,0),
					   cv::Point(0,0),
					   cv::Point(0,0) };
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
