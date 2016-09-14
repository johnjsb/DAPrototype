#ifndef SIMPLEBLOBPARAMS_HPP_INCLUDED
#define SIMPLEBLOBPARAMS_HPP_INCLUDED

namespace lanedetectconstants {

	//Processing methods to perform
	bool enableblobcontour{false};
	bool enablesegmentblobcontour{false};
	
	//Blob detection parameters
	const cv::SimpleBlobDetector::Params klanedetectblobparams() {
        cv::SimpleBlobDetector::Params params;
        params.thresholdStep = 5;
        params.minThreshold = 60;
        params.maxThreshold = 255;
        params.minRepeatability = 10;
        params.minDistBetweenBlobs = 10;
        params.filterByColor = true;
        params.blobColor = 255;
        params.filterByArea = true;
        params.minArea = 2.0f;
        params.maxArea = 200.0f;
        params.filterByCircularity = false;
        params.minCircularity = 0.9f;
        params.maxCircularity = 1.0f;
        params.filterByInertia = false;
        params.minInertiaRatio = 0.1f;
        params.maxInertiaRatio = 1.0f;
        params.filterByConvexity = true;
        params.minConvexity = 0.7f;
        params.maxConvexity = 1.0f;
        return params;
	}
	
	//Segment filters
	//const uint16_t ksegmentlength{10};
	const uint16_t ksegmentellipseheight{5};
	const float ksegmentanglewindow{80.0f};
	const float ksegmentlengthwidthratio{1.5f};
	
	//Construct from blob filters
	const float kblobslopewindow{0.10f};
	
	//Construct from segment and blob filters
	const float ksegmentblobanglewindow{20.0f};
	
	//Construct from segments filters
	const float ksegmentsanglewindow{45.0f};
	
	//Final contour filters
	//const uint16_t klength{100};
	const uint16_t kellipseheight{60};
	const float kanglewindow{80.0f};
	const float klengthwidthratio{7.0f};
	
	//Scoring variables
    const double kcommonanglewindow = 45.0;
    const uint16_t kminroadwidth = 200.0;
    const uint16_t kmaxroadwidth = 700.0;
	const uint16_t koptimumwidth = 400;
	//weighting for best grade
	const double klengthweight = 5.0;
	const double kangleweight = -5.0;
	const double kcenteredweight = -2.5;
	const double kwidthweight = 0.0;		//-1.0;
	const double klowestpointweight = 0.0;	//-0.25;	//Should be higher but I have bad test videos
	const double klowestscorelimit = -DBL_MAX;
	
}

#endif // SIMPLEBLOBPARAMS_HPP_INCLUDED
