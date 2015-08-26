#ifndef  _VOODOOMETRY_TYPES_
#define _VOODOOMETRY_TYPES_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <map>
#include <memory>

namespace visual_odometry {

typedef unsigned int Identifier;

class Feature {

private:
    static int next_id_s_;

public:
    typedef std::shared_ptr<Feature> Ptr;
    typedef std::map<int, Feature::Ptr> FeatMap;
    typedef std::vector<Feature::Ptr> FeatPtrSeq;
    typedef std::vector<Feature> FeatSeq;


    // TYPE ENUM
    enum {
        TRIANGLE, // Newly detected feature
        STAR, // Re-observed feature but non-triangulated yet
        SQUARE, // HAS 2D and 3D POINTS SET
        //CIRCLE, // Ready to use in the current frame
        UNKOWN
    };


    Feature() : id_(next_id_s_++), frame_id_(-1), type_(UNKOWN) {}
    Feature(const Feature& other) {
        id_ = other.id_;
        frame_id_ = other.frame_id_;
        kp_ = other.kp_;
        p3D_ = other.p3D_;
        desc_ = other.desc_;
        type_ = other.type_;
    }




    operator cv::Mat(){
        return desc_;
    }

    operator cv::KeyPoint(){
        return kp_;
    }

    operator cv::Point3f(){
        return p3D_;
    }


    int id_, frame_id_;
    cv::KeyPoint kp_;
    cv::Point3f p3D_;
    cv::Mat desc_;

    int type_;

};

// Has only square features
class TrackSet{
public:
    std::vector<cv::Point3f> points3D_;
    std::vector<cv::Point2f> points2D_;
    std::vector<cv::KeyPoint> keypoints2D_;

    cv::Mat image;
    cv::Mat descriptors;

};




class Memory {

public:


    std::vector<cv::Point3d> map_;
    std::vector<cv::KeyPoint> keypoints2D_;

    std::vector<cv::Mat> Rs_;
    std::vector<cv::Mat> Ts_;
    std::vector< std::vector< cv::Point2d > > points2D_;
    std::vector<cv::Mat> cam_matrix_list, dist_coeff_list;

    cv::Mat camera_matrix, dist_coeffs;




};


}

#endif // _VOODOOMETRY_TYPES_
