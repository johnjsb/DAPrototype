//standard libraries
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <deque>
#include <algorithm>
#include <math.h>

//3rd party libraries
#include "opencv2/core/core.hpp"

//project libraries
#include "lane_detect_constants.h"
#include "lane_detect_processor.h"

//Preprocessor literals
#ifndef M_PI
    #define M_PI 3.14159265359
#endif
#ifndef M_PI_2
    #define M_PI_2 1.57079632679
#endif
#ifndef M_PI_4
    #define M_PI_4 0.78539816339
#endif
#ifndef M_1_PI
	#define M_1_PI 0.31830988618
#endif
#define DEGREESPERRADIAN 57.2957795131
#define POLYGONSCALING 0.1

namespace lanedetectconstants {
	//Image evaluation
	uint16_t lowercannythreshold{ 40 };
	
	//Polygon filtering
	Polygon optimalpolygon{ cv::Point(100,400),
							cv::Point(540,400),
							cv::Point(340,250),
							cv::Point(300,250) };
	uint16_t koptimumwidth{ static_cast<uint16_t>(optimalpolygon[1].x -
												  optimalpolygon[0].x) };
	uint16_t kroadwithtolerance{ 100 };
    uint16_t kminroadwidth{ static_cast<uint16_t>(koptimumwidth - kroadwithtolerance) };
    uint16_t kmaxroadwidth{ static_cast<uint16_t>(koptimumwidth + kroadwithtolerance) };
	
	//Segment filtering
	uint16_t ksegmentellipseheight{ 10 };			//In terms of pixels, future change
	uint16_t kverticalsegmentlimit{ static_cast<uint16_t>(optimalpolygon[2].y) };
	float ksegmentminimumangle{ 26.0f };
	float ksegmentlengthwidthratio{ 2.4f };
	
	//Contour construction filter
	float ksegmentsanglewindow{ 37.0f };
	
	//Contour filtering
	uint16_t kellipseheight{ 25 };					//In terms of pixels, future change
	float kminimumangle{ 25.0f };
	float klengthwidthratio{ 5.55f };
	
	//Scoring
	float kanglefromcenter{ 30.0f };
	float klowestscorelimit{ 10.0f };

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
    //double otsuthreshval = cv::threshold( image, image, 0, 255,
	//										CV_THRESH_BINARY | CV_THRESH_OTSU );
	//Canny edge detection
    //cv::Canny( image, image, otsuthreshval * 0.5, otsuthreshval );
    cv::Canny( image, image,
			   lanedetectconstants::lowercannythreshold,
			   3 * lanedetectconstants::lowercannythreshold );
	std::vector<Contour> detectedcontours;
    std::vector<cv::Vec4i> detectedhierarchy;
    cv::findContours( image, detectedcontours, detectedhierarchy,
					  CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
		
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
	Contour leftcontour;
	Contour rightcontour;
	
	//Create optimal polygon mat
	cv::Mat optimalmat{ cv::Mat(POLYGONSCALING * image.rows,
						POLYGONSCALING * image.cols,
						CV_8UC1,
						cv::Scalar(0)) };
	cv::Point cvpointarray[4];
	for  (int i =0; i < 4; i++ ) {
		cvpointarray[i] = cv::Point(POLYGONSCALING *
						  lanedetectconstants::optimalpolygon[i].x,
						  POLYGONSCALING * lanedetectconstants::optimalpolygon[i].y);
	}
	cv::fillConvexPoly( optimalmat, cvpointarray, 4,  cv::Scalar(1) );
	
	//Find best score
	for ( EvaluatedContour &leftevaluatedontour : leftcontours ) {
		for ( EvaluatedContour &rightevaluatedcontour : rightcontours ) {
			//Check sum angle
			if ( (fabs(180.0f - leftevaluatedontour.angle + rightevaluatedcontour.angle) *
					   0.5f) > lanedetectconstants::kanglefromcenter ) return;
			
			Polygon newpolygon{ cv::Point(0,0),
								cv::Point(0,0),
								cv::Point(0,0),
								cv::Point(0,0) };
			FindPolygon( newpolygon,
						 leftevaluatedontour.contour,
						 rightevaluatedcontour.contour );
				
			//If invalid polygon created, goto next
			if ( newpolygon[0] == cv::Point(0,0) ) continue;
			
			//Score
			float score{ PercentMatch(newpolygon, optimalmat) };
			
			//If highest score update
			if ( score > maxscore ) {
				leftcontour = leftevaluatedontour.contour;
				rightcontour = rightevaluatedcontour.contour;
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
	/*
	//Create fitline
	cv::Vec4f fitline;
	cv::fitLine(contour, fitline, CV_DIST_L2, 0, 0.1, 0.1 );

	//Filter by angle
	float angle{ FastArcTan2(fitline[1], fitline[0]) };
	*/
	float angle;
	if (ellipse.angle > 90.0f) {
		angle = ellipse.angle - 90.0f;
	} else {
		angle = ellipse.angle + 90.0f;
	}
	
	if (angle < 90.0f) {
		if ( angle < lanedetectconstants::ksegmentminimumangle ) return;
	} else {
		if ( angle > (180.0f - lanedetectconstants::ksegmentminimumangle) ) return;
	}

	evaluatedsegments.push_back( EvaluatedContour{contour,
												  ellipse,
												  lengthwidthratio,
												  angle} );
	//											    angle, fitline} );
	return;
}

/*****************************************************************************************/	
void ConstructFromSegments( const  std::vector<EvaluatedContour>& evaluatedsegments,
                            std::vector<Contour>& constructedcontours )
{
    for ( const EvaluatedContour &segcontour1 : evaluatedsegments ) {
		for ( const EvaluatedContour &segcontour2 : evaluatedsegments ) {
			//if ( segcontour1.ellipse == segcontour2.ellipse ) continue;
			if ( segcontour1.contour == segcontour2.contour ) continue;
			//if ( segcontour1.fitline == segcontour2.fitline ) continue;
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
		if ( evaluatedcontour.ellipse.center.x < (imagewidth * 0.5f) ) {
			//Filter by angle
			if ( evaluatedcontour.angle > (180.0f - lanedetectconstants::kminimumangle) ) {
				return;
			}
			if ( evaluatedcontour.angle < 75.0f ) return;
			leftcontours.push_back( evaluatedcontour );
		} else {
			//Filter by angle
			if ( evaluatedcontour.angle < lanedetectconstants::kminimumangle) return;
			if ( evaluatedcontour.angle > 105.0f ) return;
			rightcontours.push_back( evaluatedcontour );
		}
	}
	return;
}

/*****************************************************************************************/
void FindPolygon( Polygon& polygon,
                  const Contour& leftcontour,
				  const Contour& rightcontour,
				  bool useoptimaly )
{
	//Get point extremes
	auto minmaxyleft = std::minmax_element( leftcontour.begin(),
											leftcontour.end(),
											[]( const cv::Point& lhs,
												const cv::Point& rhs )
												{ return lhs.y < rhs.y; } );
	auto minmaxyright = std::minmax_element( rightcontour.begin(),
											 rightcontour.end(),
											 []( const cv::Point& lhs,
												 const cv::Point& rhs )
												 { return lhs.y < rhs.y; } );
	int leftmaxx{ minmaxyleft.second->x },
		leftminx{ minmaxyleft.first->x },
		leftmaxy{ minmaxyleft.second->y },
		leftminy{ minmaxyleft.first->y },
		rightmaxx{ minmaxyright.second->x },
		rightminx{ minmaxyright.first->x },
		rightmaxy{ minmaxyright.second->y },
		rightminy{ minmaxyright.first->y };
	int	maxyoptimal{ lanedetectconstants::optimalpolygon[0].y };
	int maxyactual{ std::max(minmaxyleft.second->y, minmaxyright.second->y) };
	int miny{ std::max(minmaxyleft.first->y, minmaxyright.first->y) };
	int maxy;	
	if ( useoptimaly ) {
		maxy = maxyoptimal;
	} else {
		maxy = maxyactual;
	}
	
	//Define slopes
	float leftslope;
	if ((leftmaxx - leftminx) == 0) {
		leftslope = FLT_MAX;
	} else {
		leftslope = static_cast<float>(leftmaxy-leftminy) / static_cast<float>(
			leftmaxx - leftminx);
	}
	float rightslope;
	if ((rightmaxx - rightminx) == 0) {
		rightslope = FLT_MAX;
	} else {
		rightslope = static_cast<float>(rightmaxy-rightminy) / static_cast<float>(
			rightmaxx - rightminx);
	}

	//Calculate center points
    cv::Point leftcenter{ cv::Point((leftmaxx + leftminx) * 0.5f,
									(leftmaxy + leftminy) * 0.5f) };
    cv::Point rightcenter{ cv::Point((rightmaxx + rightminx) * 0.5f,
									 (rightmaxy + rightminy) * 0.5f) };
	
	//Calculate optimal bottom points
	cv::Point bottomleftoptimal{ cv::Point(leftcenter.x +(maxyoptimal - leftcenter.y)/leftslope,
										   maxyoptimal) };
	cv::Point bottomrightoptimal{ cv::Point(rightcenter.x +	(maxyoptimal - rightcenter.y)/rightslope,
											maxyoptimal) };
	
	//Perform filtering based on width of polygon with optimal maxy
	int roadwidth{ bottomrightoptimal.x - bottomleftoptimal.x };
	if ( roadwidth < lanedetectconstants::kminroadwidth ) return;
	if ( roadwidth > lanedetectconstants::kmaxroadwidth ) return;
	
	cv::Point topright{ cv::Point(rightcenter.x - (rightcenter.y - miny)/rightslope,
								  miny) };
	cv::Point topleft{ cv::Point(leftcenter.x - (leftcenter.y - miny)/leftslope,
								 miny) };
		
	//Check validity of shape
	if ((((leftslope < 0.0f) && (rightslope > 0.0f)) ||
		((leftslope > 0.0f) && (rightslope > 0.0f)) ||
		((leftslope < 0.0f) && (rightslope < 0.0f))) &&
		((bottomleftoptimal.x < bottomrightoptimal.x) && (topleft.x < topright.x))){

		//Construct polygon
		if ( useoptimaly ) {
			polygon[0] = bottomleftoptimal;
			polygon[1] = bottomrightoptimal;
		} else {
			polygon[0] = cv::Point(leftcenter.x +
				(maxy - leftcenter.y)/leftslope, maxy);
			polygon[1] = cv::Point(rightcenter.x +
				(maxy - rightcenter.y)/rightslope, maxy);
		}
		polygon[2] = topright;
		polygon[3] = topleft;
	}

	return;
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
	uint16_t excessarea{ 0 };
	uint16_t overlaparea{ 0 };
	for ( int i = 0; i < optimalmat.rows; i++ ) {
		uchar* p { polygonmat.ptr<uchar>(i) };
		for ( int j = 0; j < optimalmat.cols; j++ ) {
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
uint32_t FastSquareRoot( int32_t x )
{
    int32_t a, b;
    b = x;
    a = x = 0x3f;
    x = b / x;
    a = x = (x + a) >> 1;
    x = b / x;
    a = x = (x + a) >> 1;
    x = b / x;
    x = (x + a) >> 1;
    return x;  
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