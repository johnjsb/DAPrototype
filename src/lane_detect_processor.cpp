//standard libraries
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <deque>
#include <algorithm>

//3rd party libraries
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

//project libraries
#include "lane_detect_constants.h"
#include "lane_detect_processor.h"

namespace lanedetectconstants {
	
	uint16_t ksegmentellipseheight{10};
	float ksegmentanglewindow{89.0f};
	float ksegmentlengthwidthratio{2.04f};
	float ksegmentsanglewindow{45.0f};
	uint16_t kellipseheight{21};
	float kanglewindow{78.2f};
	float klengthwidthratio{5.92f};
    double kcommonanglewindow{33.3};
    uint16_t kminroadwidth {303};
    uint16_t kmaxroadwidth {628};
	uint16_t koptimumwidth {400};
	double kellipseratioweight{1.3};
	double kangleweight{-2.2};
	double kcenteredweight{-1.0};
	double kwidthweight{-2.0};
	double klowestpointweight{-1.0};
	double klowestscorelimit{-DBL_MAX};
	
}

//Main function
void ProcessImage ( cv::Mat image,
                    Polygon& polygon )
//void ProcessImage ( cv::Mat image, cv::Mat& cannyimage, Polygon& polygon )
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
    double otsuthreshval = cv::threshold( image, image, 0, 255,
		CV_THRESH_BINARY | CV_THRESH_OTSU );
	//Canny edge detection
    cv::Canny(image, image, otsuthreshval*0.5, otsuthreshval );
	//cv::cvtColor( image, cannyimage, CV_GRAY2BGR );
	std::vector<Contour> detectedcontours;
    std::vector<cv::Vec4i> detectedhierarchy;
    cv::findContours( image, detectedcontours, detectedhierarchy,
		CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
	//Contours removed by position in function

	//ToDo - There's way more I could be doing:
		//Dilate?
		//HoughLineP evaluation?
		//LSDDetector?
		
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
		EvaluateSegment( contour, image.rows, evaluatedchildsegments );
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
	Polygon bestpolygon{ cv::Point(0,0), cv::Point(0,0), cv::Point(0,0), cv::Point(0,0) };
	double maxscore{lanedetectconstants::klowestscorelimit};
	Contour leftcontour;
	Contour rightcontour;
	for ( EvaluatedContour &leftevaluatedontour : leftcontours ) {
		for ( EvaluatedContour &rightevaluatedcontour : rightcontours ) {
			Polygon newpolygon{ cv::Point(0,0), cv::Point(0,0), cv::Point(0,0),
				cv::Point(0,0) };
			FindPolygon( newpolygon, leftevaluatedontour.contour,
				rightevaluatedcontour.contour );
			//If invalid polygon created, goto next
			if ( newpolygon[0] == cv::Point(0,0) ) continue;
			//If valid score
			double score{ ScoreContourPair( newpolygon, image.cols, image.rows,
				leftevaluatedontour, rightevaluatedcontour) };
			//If highest score update
			if ( score > maxscore ) {
				leftcontour = leftevaluatedontour.contour;
				rightcontour = rightevaluatedcontour.contour;
				maxscore = score;
				bestpolygon = newpolygon;
			}
		}
	}
	
//-----------------------------------------------------------------------------------------
//Return results
//-----------------------------------------------------------------------------------------	
	if ( bestpolygon[0] == cv::Point(0,0) ) {
		for (int i = 0; i < polygon.size(); i++){
			polygon[i] = cv::Point(0,0);
		}
	} else {
		std::copy(std::begin(bestpolygon), std::end(bestpolygon), std::begin(polygon));
	}
}

/*****************************************************************************************/	
void CreateKeypoints( const std::vector<Contour>& contours,
                      std::vector<cv::KeyPoint>& keypoints )
{
	for ( const Contour &contour : contours ) {
		//Get contour moment
		cv::Moments moment{ cv::moments( contour, false ) };
		//Push into keypoint vector
		keypoints.push_back( cv::KeyPoint{ static_cast<float>(moment.m10/moment.m00),
		static_cast<float>(moment.m01/moment.m00), static_cast<float>
			(cv::contourArea(contour)) } );
	}
	return;
}
	
/*****************************************************************************************/	
void EvaluateSegment( const Contour& contour,
                      const int imageheight,
					  std::vector<EvaluatedContour>& evaluatedsegments )
{
	//Filter by size, only to prevent exception when creating ellipse or fitline
	if ( contour.size() < 5 ) return;
	//Create fitline
	cv::Vec4f fitline;
	cv::fitLine(contour, fitline, CV_DIST_L2, 0, 0.1, 0.1 );
	//Filter by angle
	float angle = atan2(fitline[1], fitline[0]) * (180.0f / CV_PI);
	if(angle < 0 ) angle += 180.0;
	if ( abs(angle - 90.0) > lanedetectconstants::ksegmentanglewindow ) return;
	//if ( abs(ellipse.angle - 90.0) > lanedetectconstants::ksegmentanglewindow )
	//Create ellipse
	cv::RotatedRect ellipse{ fitEllipse(contour) };
	//Filter by screen position
	if ( ellipse.center.y < (imageheight/3)) return;
	//Filter by length (ellipse vs segment?)
	if ( ellipse.size.height < lanedetectconstants::ksegmentellipseheight ) return;
	//if ( arcLength(contour, false) < lanedetectconstants::ksegmentlength ) return;
	//Calculate length to width ratio
	float lengthwidthratio{ ellipse.size.height / ellipse.size.width };
	//Filter by length to width ratio
	if ( lengthwidthratio < lanedetectconstants::ksegmentlengthwidthratio ) return;
	//Create moment
	//cv::Moments moment{ cv::moments( contour, false ) };
	//Find center
	//cv::Point center{ cv::Point(static_cast<float>(moment.m10/moment.m00),
	//	static_cast<float>(moment.m01/moment.m00)) };
	//Add to dataset
	//evaluatedsegments.push_back( EvaluatedContour{contour, ellipse, lengthwidthratio,
	//	moment, center, fitline} );
	evaluatedsegments.push_back( EvaluatedContour{contour, ellipse, lengthwidthratio,
		angle, fitline} );
	return;
}

/*****************************************************************************************/	
void ConstructFromSegments( const  std::vector<EvaluatedContour>& evaluatedsegments,
                            std::vector<Contour>& constructedcontours )
{
    for ( const EvaluatedContour &segcontour1 : evaluatedsegments ) {
		for ( const EvaluatedContour &segcontour2 : evaluatedsegments ) {
			float createdangle = (180.0 / CV_PI) * atan2(segcontour1.ellipse.center.y -
				segcontour2.ellipse.center.y, segcontour1.ellipse.center.x -
				segcontour2.ellipse.center.x);
			float angledifference1 = abs(segcontour1.angle -
				segcontour2.angle);
			float angledifference2 = abs(createdangle -	segcontour1.angle);
			float angledifference3 = abs(createdangle -	segcontour2.angle);
			if ((angledifference1 < lanedetectconstants::ksegmentsanglewindow) &&
				(angledifference2 < lanedetectconstants::ksegmentsanglewindow) &&
				(angledifference3 < lanedetectconstants::ksegmentsanglewindow)) {
				Contour newcontour{ segcontour1.contour };
				newcontour.insert( newcontour.end(), segcontour2.contour.begin(),
					segcontour2.contour.end() );
				constructedcontours.push_back( newcontour );
			}
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
		//if ( evaluatedcontour.contour.arcLength(contour, false) <
		//	lanedetectconstants::klength ) continue;
		//Filter by angle
		if ( abs(evaluatedcontour.angle - 90.0) >
			lanedetectconstants::kanglewindow ) continue;
		//Filter by length to width ratio
		if ( evaluatedcontour.lengthwidthratio < lanedetectconstants::klengthwidthratio )
			continue;
		//Push into either left or right evaluated contour set
		if ( evaluatedcontour.ellipse.center.x < (imagewidth/2) ) {
		//if ( evaluatedcontour.center.x < (imagewidth/2) ) {			//moment center
			leftcontours.push_back( evaluatedcontour );
		} else {
			rightcontours.push_back( evaluatedcontour );
		}
	}
	return;
}

/*****************************************************************************************/
void FindPolygon( Polygon& polygon,
                  const Contour& leftcontour,
				  const Contour& rightcontour )
{
	//Check for valid contours to prevent exception
	if ( leftcontour.empty() || rightcontour.empty() ) {
        return;
    }
	
	//Get point extremes
	auto minmaxyleft = std::minmax_element(leftcontour.begin(), leftcontour.end(),
		[]( const cv::Point& lhs, const cv::Point& rhs ) { return lhs.y < rhs.y; });
	auto minmaxyright = std::minmax_element(rightcontour.begin(), rightcontour.end(),
		[]( const cv::Point& lhs, const cv::Point& rhs ) { return lhs.y < rhs.y; });
	int leftmaxx{minmaxyleft.second->x}, leftminx{minmaxyleft.first->x},
		leftmaxy{minmaxyleft.second->y}, leftminy{minmaxyleft.first->y};
	int rightmaxx{minmaxyright.second->x}, rightminx{minmaxyright.first->x},
		rightmaxy{minmaxyright.second->y}, rightminy{minmaxyright.first->y};
    int maxy{std::max(minmaxyleft.second->y, minmaxyright.second->y)};
	int miny{std::max(minmaxyleft.first->y, minmaxyright.first->y)};
	
	//Define slopes
	double leftslope{ static_cast<double>(leftmaxy-leftminy)/static_cast<double>(
		leftmaxx - leftminx) };
    double rightslope{ static_cast<double>(rightmaxy-rightminy)/static_cast<double>(
		rightmaxx - rightminx) };
    cv::Point leftcenter = cv::Point((leftmaxx + leftminx)/2.0,(leftmaxy + leftminy)/2.0);
    cv::Point rightcenter = cv::Point((rightmaxx + rightminx)/2.0,(rightmaxy + rightminy)/2.0);

	//If valid slopes found, calculate 4 vertices of the polygon
    if ( (std::fpclassify(leftslope) == 1024) && (std::fpclassify(rightslope) == 1024) ){
        //Calculate points
        cv::Point bottomleft = cv::Point(leftcenter.x +
			(maxy - leftcenter.y)/leftslope, maxy);
        cv::Point bottomright = cv::Point(rightcenter.x +
			(maxy - rightcenter.y)/rightslope, maxy);
        cv::Point topright = cv::Point(rightcenter.x -
			(rightcenter.y - miny)/rightslope, miny);
        cv::Point topleft = cv::Point(leftcenter.x -
			(leftcenter.y - miny)/leftslope, miny);
        //Check validity of points
        if ((((leftslope < 0.0) && (rightslope > 0.0)) ||
            ((leftslope > 0.0) && (rightslope > 0.0)) ||
            ((leftslope < 0.0) && (rightslope < 0.0))) &&
            ((bottomleft.x < bottomright.x) && (topleft.x < topright.x))){

            //Construct polygon
			polygon[0] = bottomleft;
			polygon[1] = bottomright;
			polygon[2] = topright;
			polygon[3] = topleft;
        }
    }
	return;
}

/*****************************************************************************************/
double ScoreContourPair( const Polygon& polygon,
                         const int imagewidth,
						 const int imageheight,
						 const EvaluatedContour& leftcontour,
						 const EvaluatedContour& rightcontour )
{
	//Filter by common angle
	double deviationangle{ 180.0 - leftcontour.angle -
		rightcontour.angle };
	if ( abs(deviationangle) > lanedetectconstants::kcommonanglewindow ) return (-DBL_MAX);
	//Filter by road width
	int roadwidth{ polygon[1].x - polygon[0].x };
	if ( roadwidth < lanedetectconstants::kminroadwidth ) return (-DBL_MAX);
	if ( roadwidth > lanedetectconstants::kmaxroadwidth ) return (-DBL_MAX);
	//Calculate score
	double weightedscore(0.0);
	weightedscore += lanedetectconstants::kellipseratioweight * (
		leftcontour.lengthwidthratio + rightcontour.lengthwidthratio);
	weightedscore += lanedetectconstants::kangleweight * abs(deviationangle);
	weightedscore += lanedetectconstants::kcenteredweight * (
		abs(imagewidth - polygon[0].x - polygon[1].x));
	weightedscore += lanedetectconstants::kwidthweight * (
		abs(lanedetectconstants::koptimumwidth -(polygon[1].x - polygon[0].x)));
	weightedscore += lanedetectconstants::klowestpointweight * (
		imageheight - polygon[0].y);
	return weightedscore;
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
	Polygon averagepolygon { cv::Point(0,0), cv::Point(0,0), cv::Point(0,0),
		cv::Point(0,0) };
	int nonzerocount{0};
	for ( Polygon &ipolygon : pastpolygons ) {
		if ( ipolygon[0] == cv::Point(0,0) ) continue;
		nonzerocount++;
		for (int i = 0; i < ipolygon.size(); i++) {
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
		std::copy(std::begin(averagepolygon), std::end(averagepolygon),
			std::begin(polygon));
		return;
	}
	//Find differences
	std::vector<PolygonDifferences> polygondifferences;
	for ( Polygon &ipolygon : pastpolygons ) {
		float differencefromaverage{0.0f};
		for (int i = 0; i < ipolygon.size(); i++) {
			differencefromaverage += abs(averagepolygon[i].x - ipolygon[i].x);
			differencefromaverage += abs(averagepolygon[i].y - ipolygon[i].y);
		}
		polygondifferences.push_back( PolygonDifferences { ipolygon,
			differencefromaverage } );
	}
	//Sort
	sort(polygondifferences.begin(), polygondifferences.end(), [](
		const PolygonDifferences& a, const PolygonDifferences& b ) {
		return a.differencefromaverage < b.differencefromaverage; });
	//Sum closest values
	averagepolygon = { cv::Point(0,0), cv::Point(0,0), cv::Point(0,0),
		cv::Point(0,0) };
	for (int i = 0; i < samplestoaverage; i++) {
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
	std::copy(std::begin(averagepolygon), std::end(averagepolygon),
		std::begin(polygon));
	return;
}