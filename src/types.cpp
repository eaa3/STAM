/** @file types.cpp
 *
 * @author	Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author	Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 *
 * @version 1.0
 *
 */


#include "types.h"
#include <stdio.h>
#include <cvsba/cvsba.h>
#include <fstream>

namespace visual_odometry {


/********** MEMORY ************/

int Memory::next_p3d_id_s_ = 0;
int Memory::next_p2d_id_s_ = 0;

void Memory::addKeyFrame(int key_frame_id, cv::Mat rvec, cv::Mat tvec, cv::Mat cam_matrix, cv::Mat dist_coeff) {

    Rs_.insert( std::make_pair(key_frame_id, rvec) );
    Ts_.insert( std::make_pair(key_frame_id, tvec) );

    cam_matrix_list.insert(std::make_pair(key_frame_id, cam_matrix));
    dist_coeff_list.insert(std::make_pair(key_frame_id, dist_coeff));

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

void Memory::optimise() {
    printf(" Preparing bundle adjustment data!\n");
    cvsba::Sba sba;
    std::vector< cv::Point3d > points3D;
    std::vector< std::vector< cv::Point2d > > pointsImg;
    std::vector< std::vector< int > > visibility;
    std::vector< cv::Mat > cameraMatrix, distCoeffs, R, T;

    points3D.resize(map_.size());
    printf(" number of points %d \n", points3D.size());
    int i = 0;
    for(auto p3d_it = map_.begin(); p3d_it != map_.end(); p3d_it++){
        points3D[i] = p3d_it->second;
        i++;
    }

    pointsImg.resize(Rs_.size());
    visibility.resize(Rs_.size());

    printf(" number of keyframes %d \n", Rs_.size());

    int kf_index = 0;
    int kf = 0;
    std::map<int,int> kf_map;
    for(auto it = projections_.begin(); it != projections_.end(); it++){

        auto kf_it = kf_map.find(it->first);


        if( kf_it == kf_map.end() ){
            kf = kf_index++;
            kf_map.insert(std::make_pair(it->first,kf));

        }
        else
        {
            kf = kf_it->second;
        }

        //printf("kf_id %d kf_mem_id %d\n", kf, it->first);


        int p3d_id = it->second.first;
        int p2d_id = it->second.second;

        if( !pointsImg[kf].size() ) {
            pointsImg[kf].resize( map_.size() );
            visibility[kf].resize( map_.size(), 0);
        }

        visibility[kf][p3d_id] = 1;
        pointsImg[kf][p3d_id] = points2D_[p2d_id];
    }
    printf("projSize %d\n", pointsImg.size());

    printf(" number of keyframes %d \n", kf_map.size());
    cameraMatrix.resize( kf_map.size() );
    distCoeffs.resize( kf_map.size() );
    R.resize(kf_map.size() );
    T.resize(kf_map.size() );
    for(auto it =  kf_map.begin(); it != kf_map.end(); it++){

        //printf("accessing RT pos %d\n", it->second);

        R[it->second] = Rs_[it->first];
        T[it->second] = Ts_[it->first];


        distCoeffs[it->second] = dist_coeff_list[it->first];
        cameraMatrix[it->second] = cam_matrix_list[it->first];
    }
    printf(" input size R T %d %d\n", R.size(), T.size());

//    double Sba::run (  std::vector<cv::Point3d>& points,
//                       const std::vector<std::vector<cv::Point2d> >& imagePoints,
//                       const std::vector<std::vector<int> >& visibility,
//                       std::vector<cv::Mat>& cameraMatrix,
//                       std::vector<cv::Mat>& R,
//                       std::vector<cv::Mat>& T,
//                       std::vector<cv::Mat>& distCoeffs );

//    points: vector of estimated 3d points (size N).
//    imagePoints: (input/[output]) vector of vectors of estimated image projections of 3d points (size MxN). Element imagePoints[i][j] refers to j 3d point projection over camera i.
//    visibility: [input] same structure and size than imagePoints (size MxN). Element  visibility[i][j] is 1 if points[j] is visible on camera i. Otherwise it is 0. No-visible projections are ignored in imagePoints structure.
//    cameraMatrix: (input/[output]) vector of camera intrinsic matrixes (size N). Each matrix consists in 3x3 camera projection matrix.
//    R: (input/[output]) vector of estimated camera rotations (size N). Each rotation is stored in Rodrigues format (size 3).
//    T: (input/[output]) vector of estimated camera traslations (size N).
//    distCoeffs: (input/[output]) vector of camera distortion coefficients (size N). Each element is composed by 5 distortion coefficients.
//    Return value:  the projection error obtained after the optimization.

    cvsba::Sba::Params params = sba.getParams();
    params.type = cvsba::Sba::MOTION;
    params.fixedDistortion=5;
    params.fixedIntrinsics=5;
    params.verbose = false;
    sba.setParams(params);
    printf("Running Bundle Adjustment! \n");
    sba.run(points3D,  pointsImg,  visibility,  cameraMatrix,  R,  T, distCoeffs);
    printf("Bundle Adjustment FINISHED! \n");
    std::cout<<"Initial error="<<sba.getInitialReprjError()<<". Final error="<<sba.getFinalReprjError()<<std::endl;

    for(auto it =  kf_map.begin(); it != kf_map.end(); it++){

        //printf("accessing RT pos %d\n", it->second);

        Rs_[it->first] = R[it->second];
        Ts_[it->first] = T[it->second];
    }

    // TODO: Update map using points3D


}

void Memory::updateKF(Frame::Ptr keyframe){

    auto it_r = Rs_.find(keyframe->id_);
    auto it_t = Ts_.find(keyframe->id_);
    auto it_cam = cam_matrix_list.find(keyframe->id_);

    cv::Mat rvec = it_r->second;
    cv::Mat tvec = it_t->second;

    cv::Mat intrinsics = it_cam->second;

    cv::Mat R(3, 3, CV_64FC1);
    cv::Rodrigues(rvec, R);

    cv::Mat Pose(3,4, R.type());
    R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
    cv::Mat projMatrix = Pose;

    keyframe->r = rvec;
    keyframe->t = tvec;

    keyframe->projMatrix = projMatrix;

}

void Memory::dumpMapAndKFExtrinsics() {


    std::ofstream kf_ext_out("kf_extrinsics_dump.txt");
    std::ofstream kf_projmatrix_out("kf_projmatrix_dump.txt");
    FILE* map_out = fopen("map_dump.txt","w");

    auto it_r = Rs_.begin();
    auto it_t = Ts_.begin();
    auto it_cam = cam_matrix_list.begin();
    for(; it_r != Rs_.end() && it_t != Ts_.end() && it_cam != cam_matrix_list.end(); it_r++, it_t++, it_cam++){

        cv::Mat rvec = it_r->second;
        cv::Mat tvec = it_t->second;
        kf_ext_out << rvec << tvec << std::endl;

        cv::Mat intrinsics = it_cam->second;

        cv::Mat R;
        cv::Rodrigues(rvec, R);

        cv::Mat Pose(3,4, R.type());
        R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
        cv::Mat projMatrix = intrinsics*Pose;

        kf_projmatrix_out << projMatrix << std::endl;


    }

    for(auto p3d_it = map_.begin(); p3d_it != map_.end(); p3d_it++){

        fprintf(map_out, "%lf %lf %lf\n",p3d_it->second.x, p3d_it->second.y, p3d_it->second.z);
    }

    fclose(map_out);
    kf_ext_out.close();

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
