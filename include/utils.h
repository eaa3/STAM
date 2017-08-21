/** @file utils.h
 * (modified for Visual Odometry and Graph Slam by Saif Sidhik (sxs1412@student.bham.ac.uk))
 * 
 * @author  Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author  Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 * @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
 * @version 1.0
 *
 */


#ifndef _VOODOOMETRY_UTILS_
#define _VOODOOMETRY_UTILS_



#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>


namespace visual_odometry {

namespace utils {


void convertToStdVector(cv::Mat descriptors, std::vector<cv::Mat>& descs_vector_out);
cv::Mat convertToMat(const std::vector<cv::Mat>& descs_vector);

std::vector<cv::Point3f> find3DCheckerboardCorners(const std::string& p3D_filename, const int points_per_row, const int points_per_col, bool invert_flag = false);
std::vector<cv::Point3f> find3DCheckerboardCorners(const int points_per_row, const int points_per_col, bool invert_flag = false);
std::vector<cv::Point3f> find3DCheckerboardCorners(std::vector<cv::Point3f> marker_points, const int points_per_row, const int points_per_col, bool invert_flag);

const float checkerboard_scale_ = 30.0;

// Matcher goes here
class GenericMatcher {

public:

    typedef std::vector<cv::DMatch> MatchSeq;

    // for S01:  ratio_(0.45f), refineF_(true),
    // confidence_(0.99), distance_(2.0)
    GenericMatcher() : ratio_(0.65f), refineF_(true),
        confidence_(0.99), distance_(1.0) {
        // SURF is the default feature
        detector_ = new cv::StarFeatureDetector();//new cv::SurfFeatureDetector();
        // detector_ = new cv::StarFeatureDetector(32, 10, 18, 18, 20);//new cv::SurfFeatureDetector();
        extractor_ = new cv::SURF(1000, 4, 2, true, false);//new cv::SurfDescriptorExtractor();
    }

    GenericMatcher(float ratio, double confidence, double distance, bool refineF = true) : ratio_(ratio), refineF_(refineF),
        confidence_(confidence), distance_(distance) {
        // SURF is the default feature
        detector_ = new cv::StarFeatureDetector(32, 10, 18, 18, 20);//new cv::SurfFeatureDetector();
        extractor_ = new cv::SURF(1000, 4, 2, true, false);//new cv::SurfDescriptorExtractor();
    }


    // Set the feature detector
    void setFeatureDetector(
            cv::Ptr<cv::FeatureDetector>& detect) {
        detector_ = detect;
    }

    // Set the descriptor extractor
    void setDescriptorExtractor(
            cv::Ptr<cv::DescriptorExtractor>& desc) {
        extractor_ = desc;
    }



    //void match(DataSpot3D::DataSpot3DPtr spot_src, DataSpot3D::DataSpot3DPtr spot_target, std::vector<cv::DMatch>& matches);




    // Match feature points using symmetry test and RANSAC
    // returns fundemental matrix
    cv::Mat match2(cv::Mat& image1,
                  cv::Mat& image2, // input images
                  // output matches and keypoints
                  std::vector<cv::DMatch>& matches,
                  std::vector<cv::KeyPoint>& keypoints1,
                  std::vector<cv::KeyPoint>& keypoints2);

    cv::Mat match(const cv::Mat& descriptors1,
                  const cv::Mat& descriptors2, // input descriptors
                    // (output) matches and (input) keypoints
                  std::vector<cv::DMatch>& matches,
                  std::vector<cv::KeyPoint>& keypoints1,
                  std::vector<cv::KeyPoint>& keypoints2);



    // Clear matches for which NN ratio is > than threshold
    // return the number of removed points
    // (corresponding entries being cleared,
    // i.e. size will be 0)
    int ratioTest(std::vector<std::vector< cv::DMatch> >
                  &matches);

    // Insert symmetrical matches in symMatches vector
    void symmetryTest(
            const std::vector<std::vector< cv::DMatch> >& matches1,
            const std::vector<std::vector< cv::DMatch> >& matches2,
            std::vector<cv::DMatch>& symMatches);

    // Identify good matches using RANSAC
    // Return fundemental matrix
    cv::Mat ransacTest(
            const std::vector<cv::DMatch>& matches,
            const std::vector<cv::KeyPoint>& keypoints1,
            const std::vector<cv::KeyPoint>& keypoints2,
            std::vector<cv::DMatch>& outMatches);



    // pointer to the feature point detector object
    cv::Ptr<cv::FeatureDetector> detector_;
    // pointer to the feature descriptor extractor object
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    float ratio_; // max ratio between 1st and 2nd NN
    bool refineF_; // if true will refine the F matrix
    double distance_; // min distance to epipolar
    double confidence_; // confidence level (probability)

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
