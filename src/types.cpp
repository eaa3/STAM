#include "types.h"
#include <stdio.h>

namespace visual_odometry {





/********** FEATURE ************/

// Static member
int Feature::next_id_s_ = 0;


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

void Frame::calcProjMatrix(cv::Mat guess_r, cv::Mat guess_t) {

    std::vector<cv::Point3f> points3d;
    std::vector<cv::Point2f> points2d;

    for (int i = std::max<int>(0,(squareFeatures.size()-40)); i < squareFeatures.size(); i++) {
        points3d.push_back(squareFeatures[i].p3D_);
        points2d.push_back(squareFeatures[i].kp_.pt);
    }


    //        bool cv::solvePnPRansac	(	InputArray 	objectPoints,
    //        InputArray 	imagePoints,
    //        InputArray 	cameraMatrix,
    //        InputArray 	distCoeffs,
    //        OutputArray 	rvec,
    //        OutputArray 	tvec,
    //        bool 	useExtrinsicGuess = false,
    //        int 	iterationsCount = 100,
    //        float 	reprojectionError = 8.0,
    //        double 	confidence = 0.99,
    //        OutputArray 	inliers = noArray(),
    //        int 	flags = SOLVEPNP_ITERATIVE
    //        )


    LOG("Number of Points %d\n", points3d.size());

    cv::Mat tvec = guess_t, rvec = guess_r;
    cv::solvePnPRansac(points3d, points2d, intrinsic, distortion, rvec, tvec, false, 100, 5.0, 0.99);
    //r = rvec; t = tvec;
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::Mat Pose(3,4, R.type());
    R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
    projMatrix = intrinsic*Pose;
    //CalcProjectionMatrix(points3d, points2d, projMatrix);

    printProjMatrix();
}

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


void Frame::loadIntrinsicsFromFile(const std::string& filename){


    cv::FileStorage cvfs(filename.c_str(),CV_STORAGE_READ);
    if( cvfs.isOpened() ){
        cvfs["mat_intrinsicMat"] >> intrinsic;
        cvfs["mat_distortionMat"] >> distortion;
    }
}

void Frame::loadKpFromFile(const std::string& filename){

    FILE *f = fopen(filename.c_str(), "r");
    float x, y;
    int index = 0;

    while (fscanf(f, "%f,%f", &x, &y) == 2){
        squareFeatures[index].kp_ = cv::KeyPoint(x, y, 1);
        squareFeatures[index].type_ = Feature::SQUARE;
        index++;
    }

    fclose(f);

}

void Frame::load3DPointsFromFile(const std::string& filename){

    FILE *f = fopen(filename.c_str(), "r");
    cv::Point3f p;
    int index = 0;

    while (fscanf(f, "%f,%f,%f", &p.x, &p.y, &p.z) == 3){
        squareFeatures[index].p3D_ = p;
        index++;
    }

    fclose(f);

}

void Frame::printProjMatrix() {
    static int projIndex = 0;
    LOG("Proj Matrix #%d:", projIndex++);
    for (int i = 0; i < 12; i++) {
        LOG("%s%lf\t", (i % 4 == 0) ? "\n" : "", projMatrix.at<double>(i / 4, i % 4));
    }
    LOG("\n");

    projectAndShow();
}

void Frame::readNextFrame(const std::string& NEXT_FRAME_FMT) {


    static int findex = 0;
    char buf[256];
    sprintf(buf, NEXT_FRAME_FMT.c_str(), findex++);
    id_ = next_id_s_++;
    image = cv::imread(buf);
    if (image.empty()) {
        exit(0);
    }
}

void Frame::updateUsingKLT(Frame& previousFrame) {

    squareFeatures.clear();

    std::vector<cv::Point2f> previousPoints;
    std::vector<cv::Point2f> currentPoints;
    std::vector<uchar> status;
    cv::Mat errors;
    for (int i = 0; i < previousFrame.squareFeatures.size(); i++) {
        previousPoints.push_back(previousFrame.squareFeatures[i].kp_.pt);
    }

    //        void calcOpticalFlowPyrLK(InputArray prevImg, InputArray nextImg, InputArray prevPts,
    //                                  InputOutputArray nextPts, OutputArray status,
    //                                  OutputArray err,
    //                                  Size winSize=Size(21,21),
    //                                  int maxLevel=3,
    //                                  TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
    //                                  int flags=0, double minEigThreshold=1e-4 )

    cv::calcOpticalFlowPyrLK(previousFrame.image, image, previousPoints, currentPoints, status, errors); //cv::Size(21,21), 3, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01) , cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4 ); //1e-4

    for (int i = 0; i < previousPoints.size(); i++) {
        if (status[i] != 0 && errors.at<float>(i) < 12.0f) { // klt ok!
            Feature f;

            f.kp_ = cv::KeyPoint(currentPoints[i].x, currentPoints[i].y, 1);
            f.p3D_ = previousFrame.squareFeatures[i].p3D_;
            squareFeatures.push_back(f);

            // Meaning: f is visible in this frame
        }
    }
}

void Frame::projectAndShow() {

    cv::Mat point3D;
    point3D.create(cv::Size(1, 4), CV_64FC1);
    point3D.at<double>(0) = 140;
    point3D.at<double>(1) = -264;
    point3D.at<double>(2) = 300;
    point3D.at<double>(3) = 1;

    cv::Mat result;
    result.create(cv::Size(1, 3), CV_64FC1);
    cv::gemm(projMatrix, point3D, 1, 0, 0, result);

    //printf("%lf %lf %lf\n", result.at<double>(0), result.at<double>(1), result.at<double>(2));
    //printf("%lf %lf\n", result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2));

    cv::Mat imcopy = image.clone();
    cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)),4,cv::Scalar(0,0,255),-1);
    cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)), 3, cv::Scalar(255, 255, 255), -1);
    cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)), 2, cv::Scalar(0, 0, 255), -1);


    float b = 100/(id_+1);
    int bi = int(b*255);
    //std::max<int>(0,(squareFeatures.size()-40))
    for(int i = 0; i < squareFeatures.size(); i++){

        point3D.at<double>(0) = squareFeatures[i].p3D_.x;
        point3D.at<double>(1) = squareFeatures[i].p3D_.y;
        point3D.at<double>(2) = squareFeatures[i].p3D_.z;
        point3D.at<double>(3) = 1;

        cv::Mat p3D;
        p3D.create(cv::Size(1, 4), CV_64FC1);
        p3D.at<double>(0) = squareFeatures[i].p3D_.x;
        p3D.at<double>(1) = squareFeatures[i].p3D_.y;
        p3D.at<double>(2) = squareFeatures[i].p3D_.z;
        p3D.at<double>(2) = 1;
        cv::Mat p3DCam = (intrinsic.inv()*projMatrix)*p3D;

        float r = 40.0/(abs(p3D.at<double>(1))+1);
        int ri = 255*r;
        cv::gemm(projMatrix,  point3D , 1, 0, 0, result);
        cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)),4,cv::Scalar(ri,0,255),-1);
        cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)), 3, cv::Scalar(ri, 255, 255), -1);
        cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)), 2, cv::Scalar(ri, 0, 255), -1);



    }

    for(int i = 0; i < triangleFeatures.size(); i++){
        cv::circle(imcopy, cv::Point(triangleFeatures[i].kp_.pt.x,triangleFeatures[i].kp_.pt.y), 4,cv::Scalar(0,125,255),-1);
        cv::circle(imcopy, cv::Point(triangleFeatures[i].kp_.pt.x,triangleFeatures[i].kp_.pt.y), 3, cv::Scalar(125, 255, 255), -1);
        cv::circle(imcopy, cv::Point(triangleFeatures[i].kp_.pt.x,triangleFeatures[i].kp_.pt.y) , 2, cv::Scalar(125, 125, 255), -1);
    }

    cv::imshow("frame", imcopy);
    if (cv::waitKey(1) == 27) exit(0);
}



}
