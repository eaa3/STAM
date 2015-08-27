#ifndef  _VOODOOMETRY_TYPES_
#define _VOODOOMETRY_TYPES_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <map>
#include <memory>
#include <cstdio>
#include <iostream>

#define LOG(...) printf(__VA_ARGS__)


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
    std::vector<cv::Point2f> points2D_;
    std::vector<int> ids_; // Points3D ids corresponding to each Point2D_
    cv::Mat image_;

};


class KeyFrame {
public:
    static int next_id_s_;
    int id_;

    KeyFrame(cv::Mat image) : image_(image), id_(next_id_s_++) {}

    cv::Mat projMatrix_;

    // Pose
    cv::Mat tvec_, rvec_;

    cv::Mat image_;

    std::vector<cv::KeyPoint> keypoints_; // list of keypoints
    cv::Mat descriptors_; // list of descripts, one to each keypoint in the list of keypoints
    std::vector<int> ids_; // Points3D ids corresponding to each keypoint/descriptor;


};

class Frame {



public:

    typedef std::shared_ptr<Frame> Ptr;

    static int next_id_s_;

    Frame(int n) {
        squareFeatures.resize(n);
        triangleFeatures.resize(n);
        projMatrix.create(cv::Size(4, 3), CV_64FC1);
    }

    Frame(const Frame& other) {
        squareFeatures = other.squareFeatures;
        triangleFeatures = other.triangleFeatures;
        projMatrix = other.projMatrix.clone();
        intrinsic = other.intrinsic;
        distortion = other.distortion;
        image = other.image;
        keypoints = other.keypoints;
        descriptors = other.descriptors.clone();
    }

    std::vector<Feature> squareFeatures;
    std::vector<Feature> triangleFeatures;
    cv::Mat projMatrix;
    cv::Mat t, r;

    // Intrinsics params
    cv::Mat intrinsic;   // intrinsic parameters
    cv::Mat distortion;  // lens distortion coefficients

    cv::Mat image;

    std::vector<cv::KeyPoint> keypoints; // list of keypoints
    cv::Mat descriptors; // list of descripts, one to each keypoint in the list of keypoints
    std::vector<int> ids; // Points3D ids corresponding to each keypoint/descriptor;

    int id_; // ID of this frame


    void calcProjMatrix(cv::Mat guess_r = cv::Mat(), cv::Mat guess_t = cv::Mat());

    void detectAndDescribe();


    void loadIntrinsicsFromFile(const std::string& filename);

    void loadKpFromFile(const std::string& filename);

    void load3DPointsFromFile(const std::string& filename);

    void printProjMatrix();

    void readNextFrame(const std::string& NEXT_FRAME_FMT);

    void updateUsingKLT(Frame& previousFrame);



    void projectAndShow();

};




class Memory {

public:


    std::map<int, cv::Point3d> map_; // point3D id -> point3D
    std::map<int, cv::Point2d> points2D_; // point2D id -> point2D

    std::map<int, cv::Mat> Rs_; // keyframe id -> Rotation Matrix
    std::map<int, cv::Mat> Ts_; // keyframe id -> Translation Vector

    // keyframe id -> [ pair(point3D id, point2D id) ]
    // => Projection (point2D id) of point3D on Keyframe with given keyframe id
    std::multimap<int, std::pair< int, int > > projections_;

    // keyframe id -> [ pair(point3D id, True|False) ]
    // => True: point3D id is visible from keyframe id
    // => False: point3D id is not visible from keyframe id
    std::multimap<int, std::pair< int, bool > > visibility_;

    // keyframe id -> intrinsic matrix, distortion coefficients
    std::map<int, cv::Mat> cam_matrix_list, dist_coeff_list;

    cv::Mat camera_matrix, dist_coeffs;







};


}

#endif // _VOODOOMETRY_TYPES_
