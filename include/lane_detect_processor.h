#ifndef LANE_DETECT_PROCESSOR_HPP_INCLUDED
#define LANE_DETECT_PROCESSOR_HPP_INCLUDED

#include <deque>
#include <array>
#include "opencv2/opencv.hpp"

typedef std::array<cv::Point, 4> Polygon;
typedef std::vector<cv::Point> Contour;

struct EvaluatedContour {
    Contour contour;
    cv::RotatedRect ellipse;
    float lengthwidthratio;
	float angle;
	//cv::Moments moment;
	//cv::Point center;
    cv::Vec4f fitline;
};

struct PolygonDifferences {
	Polygon polygon;
	float differencefromaverage;
};

void CreateKeypoints( const std::vector<Contour>& contours,
	                  std::vector<cv::KeyPoint>& keypoints );
void EvaluateSegment( const Contour& contour,
					  const int imageheight,
	                  std::vector<EvaluatedContour>&	evaluatedsegments );
void ConstructFromBlobs( const std::vector<cv::KeyPoint>& keypoints,
	                     std::vector<Contour>& constructedcontours );
void ConstructFromSegmentAndBlob( const std::vector<EvaluatedContour>& evaluatedsegments,
	                              const std::vector<cv::KeyPoint>& keypoints,
								  std::vector<Contour>&	constructedcontours );
void ConstructFromSegments( const std::vector<EvaluatedContour>& evaluatedsegments,
                            std::vector<Contour>& constructedcontours );
void SortContours( const std::vector<EvaluatedContour>& evaluatedsegments,
				   const int imagewidth,
				   std::vector<EvaluatedContour>& leftcontours,
				   std::vector<EvaluatedContour>& rightcontours );
void FindPolygon( Polygon& polygon,
				  const Contour& leftcontour,
				  const Contour& rightcontour );
double ScoreContourPair( const Polygon& polygon,
						 const int imagewidth,
						 const EvaluatedContour& leftcontour,
						 const EvaluatedContour& rightcontour );
void AveragePolygon ( Polygon& polygon,
					  std::deque<Polygon>& pastpolygons,
					  int samplestoaverage,
					  int samplestokeep );
void ProcessImage ( cv::Mat image,
					Polygon& polygon );
//void ProcessImage ( cv::Mat image, cv::Mat& cannyimage, Polygon& polygon );

#endif // LANE_DETECT_PROCESSOR_HPP_INCLUDED
