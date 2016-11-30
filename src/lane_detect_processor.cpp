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
#define POLYGONSCALING 1.0f

/*****************************************************************************************/
namespace lanedetectconstants {
	//Image evaluation
	float kcontrastscalefactor{ 0.450f };
	
	//Polygon filtering
	Polygon optimalpolygon{ cv::Point(110,480),
							cv::Point(690,480),
							cv::Point(390,250),
							cv::Point(410,250) };
	uint16_t koptimumwidth{ static_cast<uint16_t>(optimalpolygon[1].x -
												  optimalpolygon[0].x) };
	uint16_t kroadwithtolerance{ 80 };
    uint16_t kminroadwidth{ static_cast<uint16_t>(koptimumwidth - kroadwithtolerance) };
    uint16_t kmaxroadwidth{ static_cast<uint16_t>(koptimumwidth + kroadwithtolerance) };
	
	//Segment filtering
	uint16_t ksegmentellipseheight{ 16 };			//In terms of pixels, future change
	uint16_t kverticalsegmentlimit{ static_cast<uint16_t>(optimalpolygon[2].y) };
	float ksegmentminimumangle{ 26.0f };
	float ksegmentlengthwidthratio{ 4.0f };
	
	//Contour construction filter
	float ksegmentsanglewindow{ 34.0f };
	
	//Contour filtering
	uint16_t kellipseheight{ 24 };					//In terms of pixels, future change
	float kminimumangle{ 26.0f };
	float klengthwidthratio{ 6.5f };
	
	//Scoring
	float kanglefromcenter{ 26.0f };
	uint16_t kminimumpolygonheight{ 10 };
	float klowestscorelimit{ -100.0f };
	float kheightwidthscalefactor{ 500.0f };

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
	float lowerthreshold{ lanedetectconstants::kcontrastscalefactor * std[0] };
	
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
			EvaluateSegment( detectedcontours[i], image.rows, evaluatedchildsegments );
        } else {
			EvaluateSegment( detectedcontours[i], image.rows, evaluatedparentsegments );
		}
    }

//-----------------------------------------------------------------------------------------
//Construct from segments
//-----------------------------------------------------------------------------------------	
    std::vector<std::vector<cv::Point>> constructedcontours;
	ConstructFromSegments( evaluatedchildsegments, constructedcontours );

//-----------------------------------------------------------------------------------------
//Evaluate constructed segments
//-----------------------------------------------------------------------------------------	
	for ( Contour contour : constructedcontours ) {
		EvaluateSegment( contour, image.rows, evaluatedparentsegments );
	}
	
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
	float maxscore{ lanedetectconstants::klowestscorelimit };
	EvaluatedContour leftcontour;
	EvaluatedContour rightcontour;
	
	//Create optimal polygon mat
	cv::Mat optimalmat{ cv::Mat(POLYGONSCALING * image.rows,
						POLYGONSCALING * image.cols,
						CV_8UC1,
						cv::Scalar(0)) };
	cv::Point cvpointarray[4];
	for  (int i =0; i < 4; i++ ) {
		cvpointarray[i] = cv::Point(POLYGONSCALING *
									lanedetectconstants::optimalpolygon[i].x,
									POLYGONSCALING *
									lanedetectconstants::optimalpolygon[i].y);
	}
	cv::fillConvexPoly( optimalmat, cvpointarray, 4,  cv::Scalar(1) );
	
	//Find best score
	for ( EvaluatedContour &leftevaluatedcontour : leftcontours ) {
		for ( EvaluatedContour &rightevaluatedcontour : rightcontours ) {
			//Check sum angle
			if ( (fabs(180.0f - leftevaluatedcontour.angle - rightevaluatedcontour.angle) *
				  0.5f) > lanedetectconstants::kanglefromcenter ) continue;
			
			Polygon newpolygon{ cv::Point(0,0),
								cv::Point(0,0),
								cv::Point(0,0),
								cv::Point(0,0) };
			FindPolygon( newpolygon,
						 leftevaluatedcontour,
						 rightevaluatedcontour );
				
			//If invalid polygon created, goto next
			if ( newpolygon[0] == cv::Point(0,0) ) continue;
			
			//Score
			//float score{ PercentMatch(newpolygon, optimalmat) };
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
		FindPolygon( bestpolygon, leftcontour, rightcontour, true );
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
                      const int imageheight,
					  std::vector<EvaluatedContour>& evaluatedsegments )
{	
	//Filter by size, only to prevent exception when creating ellipse or fitline
	if ( contour.size() < 5 ) return;
		
	//Create ellipse
	cv::RotatedRect ellipse{ fitEllipse(contour) };
	
	//Filter by screen position
	if ( ellipse.center.y < (lanedetectconstants::kverticalsegmentlimit)) return;
	
	//Filter by length (ellipse vs segment?)
	if ( ellipse.size.height < lanedetectconstants::ksegmentellipseheight ) return;
	
	//Calculate length to width ratio
	float lengthwidthratio{ ellipse.size.height / ellipse.size.width };
	
	//Filter by length to width ratio
	if ( lengthwidthratio < lanedetectconstants::ksegmentlengthwidthratio ) return;

	//Create fitline
	cv::Vec4f fitline;
	cv::fitLine(contour, fitline, CV_DIST_L2, 0, 0.1, 0.1 );

	//Filter by angle
	float angle{ FastArcTan2(fitline[1], fitline[0]) };
	if (angle < 0.0f) {
		angle = ellipse.angle + 180.0f;
	}
	
	if (angle < 90.0f) {
		if ( angle < lanedetectconstants::ksegmentminimumangle ) return;
	} else {
		if ( angle > (180.0f - lanedetectconstants::ksegmentminimumangle) ) return;
	}

	evaluatedsegments.push_back( EvaluatedContour{contour,
												  ellipse,
												  lengthwidthratio,
												  angle,
												  fitline} );
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
			if ( angledifference1 > lanedetectconstants::ksegmentsanglewindow ) continue;
			float createdangle { FastArcTan2((segcontour1.ellipse.center.y -
											  segcontour2.ellipse.center.y),
											 (segcontour1.ellipse.center.x -
											  segcontour2.ellipse.center.x)) };
			if ( createdangle < 90.0f ) {
				if ( createdangle < lanedetectconstants::ksegmentminimumangle ) return;
			} else {
				if ( createdangle > (180.0f -
					 lanedetectconstants::ksegmentminimumangle) ) return;
			}
			float angledifference2( fabs(createdangle -	segcontour1.angle) );
			if ( angledifference2 > lanedetectconstants::ksegmentsanglewindow ) continue;
			float angledifference3( fabs(createdangle -	segcontour2.angle) );
			if ( angledifference3 > lanedetectconstants::ksegmentsanglewindow ) continue;
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
		if ( evaluatedcontour.ellipse.size.height < lanedetectconstants::kellipseheight )
			continue;
		
		//Filter by length to width ratio
		if ( evaluatedcontour.lengthwidthratio < lanedetectconstants::klengthwidthratio )
			continue;
		
		//Push into either left or right evaluated contour set
		if ( evaluatedcontour.ellipse.center.x < (imagewidth * 0.6f) ) {
			//Filter by angle
			if ( evaluatedcontour.angle > (180.0f - lanedetectconstants::kminimumangle) ) {
				continue;
			}
			if ( evaluatedcontour.angle < 75.0f ) continue;
			leftcontours.push_back( evaluatedcontour );
		} 
		if ( evaluatedcontour.ellipse.center.x > (imagewidth * 0.4f) ) {
			//Filter by angle
			if ( evaluatedcontour.angle < lanedetectconstants::kminimumangle) continue;
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
				  bool useoptimaly )
{
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
	int	maxyoptimal{ lanedetectconstants::optimalpolygon[0].y };
	int maxyactual{ std::max(minmaxyleft.second->y, minmaxyright.second->y) };
	int miny{ std::max(minmaxyleft.first->y, minmaxyright.first->y) };
	int maxy;	
	if ( useoptimaly ) {
		maxy = maxyoptimal;
	} else {
		maxy = maxyactual;
	}
	
	//Filter by height
	if ( (maxyactual - miny) < lanedetectconstants::kminimumpolygonheight ) {
		return;
	}
	
	//Define slopes
	float evaluatedleftslope { leftcontour.fitline[1] / leftcontour.fitline[0] };
	float evaluatedrightslope { rightcontour.fitline[1] / rightcontour.fitline[0] };

	//Calculate center points
	cv::Point sum { std::accumulate(leftcontour.contour.begin(),
									leftcontour.contour.end(),
									cv::Point(0,0)) };
	cv::Point leftcenter{ cv::Point(sum.x / leftcontour.contour.size(),
									sum.y / leftcontour.contour.size()) };
	sum = std::accumulate( rightcontour.contour.begin(),
						   rightcontour.contour.end(),
						   cv::Point(0,0) );
	cv::Point rightcenter{ cv::Point(sum.x / rightcontour.contour.size(),
									sum.y / rightcontour.contour.size()) };
	
	//Calculate optimal bottom points
	cv::Point bottomleftoptimal{ cv::Point(leftcenter.x +(maxyoptimal - leftcenter.y) /
										   evaluatedleftslope,
										   maxyoptimal) };
	cv::Point bottomrightoptimal{ cv::Point(rightcenter.x +	(maxyoptimal - rightcenter.y) /
											evaluatedrightslope,
											maxyoptimal) };
	
	//Perform filtering based on width of polygon with optimal maxy
	int roadwidth{ bottomrightoptimal.x - bottomleftoptimal.x };
	if ( roadwidth < lanedetectconstants::kminroadwidth ) return;
	if ( roadwidth > lanedetectconstants::kmaxroadwidth ) return;
	
	cv::Point topright{ cv::Point(rightcenter.x - (rightcenter.y - miny) / evaluatedrightslope,
								  miny) };
	cv::Point topleft{ cv::Point(leftcenter.x - (leftcenter.y - miny) / evaluatedleftslope,
								 miny) };
		
	//Check validity of shape
	if ( !((evaluatedleftslope > 0.0f) && (evaluatedrightslope < 0.0f)) &&
		 ((bottomleftoptimal.x < bottomrightoptimal.x) && (topleft.x < topright.x)) ) {

		//Construct polygon
		if ( useoptimaly ) {
			polygon[0] = bottomleftoptimal;
			polygon[1] = bottomrightoptimal;
		} else {
			polygon[0] = cv::Point(leftcenter.x + (maxy - leftcenter.y) / evaluatedleftslope,
								   maxy);
			polygon[1] = cv::Point(rightcenter.x + (maxy - rightcenter.y) / evaluatedrightslope,
								   maxy);
		}
		polygon[2] = topright;
		polygon[3] = topleft;
	}

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
	
	return lanedetectconstants::kheightwidthscalefactor * heightwidthratio - centeroffset;
}

/*****************************************************************************************/
float PercentMatch( const Polygon& polygon,
					const cv::Mat& optimalmat )
{
	//Create blank mat
	cv::Mat polygonmat{ cv::Mat(optimalmat.rows,
								optimalmat.cols,
								CV_8UC1,
								cv::Scalar(0)) };
	
	//Draw polygon
	cv::Point cvpointarray[4];
	for  (int i =0; i < 4; i++ ) {
		cvpointarray[i] = cv::Point(POLYGONSCALING * polygon[i].x,
									POLYGONSCALING * polygon[i].y);
	}
	cv::fillConvexPoly( polygonmat, cvpointarray, 4,  cv::Scalar(2) );

	//Add together
	polygonmat += optimalmat;
	
	//Evaluate result
	uint32_t excessarea{ 0 };
	uint32_t overlaparea{ 0 };
	for ( int i = 0; i < polygonmat.rows; i++ ) {
		uchar* p { polygonmat.ptr<uchar>(i) };
		for ( int j = 0; j < polygonmat.cols; j++ ) {
			switch ( p[j] )
			{
				case 1:
					excessarea++;
					break;
				case 2:
					excessarea++;
					break;
				case 3:
					overlaparea++;
					break;
			}
		}
	}
	return (100.0f * overlaparea) / (overlaparea + excessarea);
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
