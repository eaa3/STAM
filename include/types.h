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

#include "utils.h"

#define LOG(...) printf(__VA_ARGS__)


namespace visual_odometry {

typedef unsigned int Identifier;

// Has only square features
class TrackSet{
public:
    std::vector<cv::Point2f> points2D_;
    std::vector<int> ids_; // Points3D ids corresponding to each Point2D_
    cv::Mat projMatrix;
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

    Frame(cv::Mat image) : image(image) {}

    Frame(const Frame& other) {

        projMatrix = other.projMatrix.clone();
        image = other.image;
        keypoints = other.keypoints;
        descriptors = other.descriptors.clone();
    }

    cv::Mat projMatrix;
    cv::Mat t, r;

    cv::Mat image;

    std::vector<cv::KeyPoint> keypoints; // list of keypoints
    cv::Mat descriptors; // list of descripts, one to each keypoint in the list of keypoints
    std::vector<int> ids_; // Points3D ids corresponding to each keypoint/descriptor;

    int id_; // ID of this frame

    void detectAndDescribe();
};




class Memory {

public:

    static int next_p3d_id_s_;
    static int next_p2d_id_s_;


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


    // Add new keyframe pose
    void addKeyFrame(int key_frame_id, cv::Mat rvec, cv::Mat tvec);

    // Add correspondence to memory belonging to a specific keyframe identified by its keyframe_id
    // A correspondence is a 3D point and its corresponding projection on the image plane
    // return: (p3d_id, p2d_id)
    std::pair<int,int> addCorrespondence(int key_frame_id, cv::Point2d p2D, cv::Point3d p3D);
    std::pair<int,int> addCorrespondence(int key_frame_id, cv::Point2f p2D, cv::Point3f p3D);

    // Add correspondence to memory belonging to a specific keyframe identified by its keyframe_id
    // A correspondence is a 3D point id and its corresponding projection on the image plane
    // return: (p3d_id, p2d_id)
    std::pair<int,int> addCorrespondence(int key_frame_id, cv::Point2d p2D, int point3d_id);
    std::pair<int,int> addCorrespondence(int key_frame_id, cv::Point2f p2D, int point3d_id);


    // return: (p3d_id, p2d_id)
    std::pair<int,int> addCorrespondence(int key_frame_id, int point2d_id, int point3d_id);




};


}

#endif // _VOODOOMETRY_TYPES_
