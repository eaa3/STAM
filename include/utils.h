#ifndef _VOODOOMETRY_UTILS_
#define _VOODOOMETRY_UTILS_

#include <opencv2/core/core.hpp>
#include <types.h>

namespace visual_odometry {

namespace utils {

// Matcher goes here
class GenericMatcher {

};


// Generic Feature Extractor
class GenericExtractor {

public:

    void extract(const cv::Mat& frame,  visual_odometry::Memory& mem_out);


};


// Generic Pose Calculator
//class PoseCalculator {
//
//public:
//    void calculate_projection(const visual_odometry::Memory& mem_in, cv::Mat& proj);
//};


}


}

#endif // _VOODOOMETRY_UTILS_
