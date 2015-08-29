#include "types.h"
#include <stdio.h>

namespace visual_odometry {


/********** MEMORY ************/

int Memory::next_p3d_id_s_ = 0;
int Memory::next_p2d_id_s_ = 0;

void Memory::addKeyFrame(int key_frame_id, cv::Mat rvec, cv::Mat tvec) {

    Rs_.insert( std::make_pair(key_frame_id, rvec) );
    Ts_.insert( std::make_pair(key_frame_id, tvec) );

}

std::pair<int,int> Memory::addCorrespondence(int key_frame_id, cv::Point2d p2D, cv::Point3d p3D){

    int p3d_id = next_p3d_id_s_++;
    int p2d_id = next_p2d_id_s_++;

    map_.insert( std::make_pair(p3d_id, p3D));
    points2D_.insert( std::make_pair(p2d_id, p2D) );

    projections_.insert( std::make_pair(key_frame_id, std::make_pair(p3d_id, p2d_id)) );
    visibility_.insert( std::make_pair(key_frame_id, std::make_pair(p3d_id, true)) );


    return std::make_pair(p3d_id, p2d_id);


}

std::pair<int,int> Memory::addCorrespondence(int key_frame_id, cv::Point2f p2D, cv::Point3f p3D){

    return addCorrespondence(key_frame_id, cv::Point2d(p2D.x, p2D.y), cv::Point3d(p3D.x, p3D.y, p3D.z));


}


std::pair<int,int> Memory::addCorrespondence(int key_frame_id, cv::Point2d p2D, int point3d_id) {

    int p3d_id = point3d_id;
    int p2d_id = next_p2d_id_s_++;

    points2D_.insert( std::make_pair(p2d_id, p2D) );

    projections_.insert( std::make_pair(key_frame_id, std::make_pair(p3d_id, p2d_id)) );
    visibility_.insert( std::make_pair(key_frame_id, std::make_pair(p3d_id, true)) );


    return std::make_pair(p3d_id, p2d_id);

}

std::pair<int,int> Memory::addCorrespondence(int key_frame_id, cv::Point2f p2D, int point3d_id) {

    return addCorrespondence(key_frame_id, cv::Point2d(p2D.x,p2D.y), point3d_id);

}

std::pair<int,int> Memory::addCorrespondence(int key_frame_id, int point2d_id, int point3d_id){

    int p3d_id = point3d_id;
    int p2d_id = point2d_id;

    projections_.insert( std::make_pair(key_frame_id, std::make_pair(p3d_id, p2d_id)) );
    visibility_.insert( std::make_pair(key_frame_id, std::make_pair(p3d_id, true)) );


    return std::make_pair(p3d_id, p2d_id);
}


/********** FRAME ************/

int Frame::next_id_s_ = 0;

void Frame::detectAndDescribe() {
    char algorithm[] = "SIFT";
    cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(algorithm);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create(algorithm);

    std::vector<cv::KeyPoint> kpts;
    detector->detect(image, kpts);
    for(auto it = kpts.begin(); it != kpts.end(); it++)
        keypoints.push_back(*it);

    descriptor->compute(image, keypoints, descriptors);

    ids_.resize(keypoints.size(), -1);
}


}
