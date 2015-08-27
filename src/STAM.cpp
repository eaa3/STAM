#include "STAM.h"

namespace vo_utils = visual_odometry::utils;

int SCENE = 1;


double baseline[] = { 175, 50, 25 };
std::string points2d_init_file[] = { "S01_2Ddata_dst_init.csv", "S02_2Ddata_dst_init.csv", "S03_2Ddata_dst_init.csv" };
std::string points3d_init_file[] = { "S01_3Ddata_dst_init.csv", "S02_3Ddata_dst_init.csv", "S03_3Ddata_dst_init.csv" };
std::string intrinsics_file[] = { "intrinsicsS01.xml", "intrinsicsS02.xml", "intrinsicsS03.xml" };
std::string next_frame_fmt[] = { "S01L03_VGA/S01L03_VGA_%04d.png", "S02L03_VGA/S02L03_VGA_%04d.png", "S03L03_VGA/S03L03_VGA_%04d.png"};

const double THR_BASELINE = baseline[SCENE-1];
const std::string POINTS_2D_INIT_FILE = points2d_init_file[SCENE-1];
const std::string POINTS_3D_INIT_FILE = points3d_init_file[SCENE-1];
const std::string INTRINSICS_FILE = intrinsics_file[SCENE-1];
const std::string NEXT_FRAME_FMT = next_frame_fmt[SCENE-1];

namespace visual_odometry {

int STAM::next_id_s_ = 0;
int STAM::next_kf_id_s_ = 0;

bool STAM::init(){

    load3DPointsFromFile(POINTS_2D_INIT_FILE);
    load3DPointsFromFile(POINTS_3D_INIT_FILE);
    loadIntrinsicsFromFile(INTRINSICS_FILE);

    STAM::next_kf_id_s_++;

    previousFrame->loadKpFromFile(POINTS_2D_INIT_FILE);
    previousFrame->load3DPointsFromFile(POINTS_3D_INIT_FILE);
    previousFrame->loadIntrinsicsFromFile(INTRINSICS_FILE);
    currentFrame->loadIntrinsicsFromFile(INTRINSICS_FILE);

    previousFrame->readNextFrame(NEXT_FRAME_FMT);

    previousFrame->calcProjMatrix();

    previousFrame->detectAndDescribe();

    keyFrame = previousFrame;

    return true;

}


void STAM::process(cv::Mat frame){



    currentFrame = Frame::Ptr(new Frame(*previousFrame));

    currentFrame->readNextFrame(NEXT_FRAME_FMT);

    currentFrame->updateUsingKLT(*previousFrame);


    currentFrame->calcProjMatrix();

    // for S01: baseline= 175
    // for S02: baseline= 50
    // for S03: baseline= 25 or 40


    bool enough_baseline = has_enough_baseline(keyFrame->intrinsic.inv()*keyFrame->projMatrix, currentFrame->intrinsic.inv()*currentFrame->projMatrix, THR_BASELINE);
    LOG("BEFORE: %d\t", currentFrame->squareFeatures.size());
    if (enough_baseline) { // keyframe interval // change here from 5 to any other number
        currentFrame->detectAndDescribe();

        LOG("BEFORE: %d\t", currentFrame->squareFeatures.size());
        matchAndTriangulate(*keyFrame, *currentFrame, currentFrame->intrinsic, currentFrame->distortion);
        LOG("AFTER: %d\n", currentFrame->squareFeatures.size());

        keyFrame = currentFrame;
        // This is extremely important (otherwise all Frames will have a common projMatrix in the memory)
        keyFrame->projMatrix = currentFrame->projMatrix.clone();
        keyFrame->descriptors = currentFrame->descriptors.clone();
        keyFrame->r = currentFrame->r.clone();
        keyFrame->t = currentFrame->t.clone();
    }


    previousFrame = currentFrame;

}

bool STAM::has_enough_baseline(cv::Mat pose1, cv::Mat pose2, double thr_baseline){

    cv::Mat R1 = pose1(cv::Range::all(), cv::Range(0, 3));
    cv::Mat T1 = pose1(cv::Range::all(), cv::Range(3, 4));

    cv::Mat R2 = pose2(cv::Range::all(), cv::Range(0, 3));
    cv::Mat T2 = pose2(cv::Range::all(), cv::Range(3, 4));

    cv::Mat pos1, pos2;
    pos1 = -R1.inv() * T1;
    pos2 = -R2.inv() * T2;

    double baseline = cv::norm(pos1 - pos2);

    LOG("Baseline %f\n", baseline);
    return baseline >= thr_baseline;
}

void STAM::loadIntrinsicsFromFile(const std::string& filename){


    cv::FileStorage cvfs(filename.c_str(),CV_STORAGE_READ);
    if( cvfs.isOpened() ){
        cvfs["mat_intrinsicMat"] >> intrinsic;
        cvfs["mat_distortionMat"] >> distortion;
    }
}

void STAM::loadKpFromFile(const std::string& filename){

    FILE *f = fopen(filename.c_str(), "r");
    float x, y;
    int index = 0;

    while (fscanf(f, "%f,%f", &x, &y) == 2){
        trackset_.points2D_.push_back( cv::Point2f(x, y) );

        memory_.points2D_.insert( std::make_pair(index, cv::Point2d(x, y)) );
        memory_.projections_.insert(std::make_pair(STAM::next_kf_id_s_, std::make_pair(index,index)));
        memory_.visibility_.insert(std::make_pair(STAM::next_kf_id_s_, std::make_pair(index, true)) );
        index++;
    }

    fclose(f);

}

void STAM::load3DPointsFromFile(const std::string& filename){

    FILE *f = fopen(filename.c_str(), "r");
    cv::Point3f p;
    int index = 0;

    while (fscanf(f, "%f,%f,%f", &p.x, &p.y, &p.z) == 3){
        trackset_.ids_.push_back(index);

        memory_.map_.insert( std::make_pair(index, cv::Point3d(p.x,p.y,p.z)) );
        index++;
    }

    STAM::next_id_s_ = index;

    fclose(f);

}

cv::Mat STAM::calcProjMatrix(cv::Mat guess_r, cv::Mat guess_t) {

    std::vector<cv::Point3f> points3d;
    std::vector<cv::Point2f> points2d;

    for (int i = std::max<int>(0,(trackset_.points2D_.size()-40)); i < trackset_.points2D_.size(); i++) {
        points3d.push_back(memory_.map_[trackset_.ids_[i]]);
        points2d.push_back(trackset_.points2D_[i]);
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
    cv::Mat projMatrix = intrinsic*Pose;

//    printProjMatrix();

    return projMatrix;
}

void STAM::updateUsingKLT(cv::Mat curr_image) {


    std::vector<cv::Point2f> currentPoints;
    std::vector<uchar> status;
    cv::Mat errors;


    //        void calcOpticalFlowPyrLK(InputArray prevImg, InputArray nextImg, InputArray prevPts,
    //                                  InputOutputArray nextPts, OutputArray status,
    //                                  OutputArray err,
    //                                  Size winSize=Size(21,21),
    //                                  int maxLevel=3,
    //                                  TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
    //                                  int flags=0, double minEigThreshold=1e-4 )

    cv::calcOpticalFlowPyrLK(trackset_.image_, curr_image, trackset_.points2D_, currentPoints, status, errors); //cv::Size(21,21), 3, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01) , cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4 ); //1e-4

    std::vector<cv::Point2f> filtered_p2D;
    std::vector<int> filtered_ids;
    for (int i = 0; i < trackset_.points2D_.size(); i++) {
        if (status[i] != 0 && errors.at<float>(i) < 12.0f) { // klt ok!

            filtered_p2D.push_back(trackset_.points2D_[i]);
            filtered_ids.push_back(trackset_.ids_[i]);

            // Meaning: f is visible in this frame

        }
    }
}


void STAM::matchAndTriangulate(Frame& previousFrame, Frame& currentFrame, cv::Mat intrinsics, cv::Mat distortion) {

    static vo_utils::GenericMatcher matcher;
    std::vector<cv::DMatch> matches;

    matcher.match(currentFrame.descriptors, previousFrame.descriptors, matches, currentFrame.keypoints, previousFrame.keypoints);

    LOG("NMatches %d\n",matches.size());

    std::vector<cv::Point2f> previousTriangulate, currentTriangulate;
    cv::Mat	outputTriangulate;
    outputTriangulate.create(cv::Size(4, matches.size()), CV_32FC1);


    for (int i = 0; i < matches.size(); i++) {
        cv::Point pt1 = previousFrame.keypoints[matches[i].trainIdx].pt;
        cv::Point pt2 = currentFrame.keypoints[matches[i].queryIdx].pt;
        previousTriangulate.push_back(pt1);
        currentTriangulate.push_back(pt2);
    }


    if( previousTriangulate.size() == 0 || currentTriangulate.size() == 0 ){
        //LOG("Triangulation Points %d %d\n",previousTriangulate.size(),currentTriangulate.size());
        return;
    }




    // undistort
    std::vector<cv::Point2f> previousTriangulateUnd, currentTriangulateUnd;
    cv::undistortPoints(previousTriangulate, previousTriangulateUnd, intrinsics, distortion);
    cv::undistortPoints(currentTriangulate, currentTriangulateUnd, intrinsics, distortion);

    cv::triangulatePoints(intrinsics.inv()*previousFrame.projMatrix, intrinsics.inv()*currentFrame.projMatrix, previousTriangulateUnd, currentTriangulateUnd, outputTriangulate);

    for (int i = 0; i < matches.size(); i++) {
        Feature f;
        f.kp_ = cv::KeyPoint(currentTriangulate[i].x, currentTriangulate[i].y, 1);
        f.p3D_ = cv::Point3f(outputTriangulate.at<float>(0, i) / outputTriangulate.at<float>(3, i),
            outputTriangulate.at<float>(1, i) / outputTriangulate.at<float>(3, i),
            outputTriangulate.at<float>(2, i) / outputTriangulate.at<float>(3, i));
        currentFrame.squareFeatures.push_back(f);

         //memory_.points2D_.insert( std::make_pair(index, currentTriangulate[i]) );
        // memory_.map_.insert( std::make_pair(index, cv::Point3d(p.x,p.y,p.z)) );


        //memory_.projections_.insert(std::make_pair(STAM::next_kf_id_s_, std::make_pair(index,index)));
        //memory_.visibility_.insert(std::make_pair(STAM::next_kf_id_s_, std::make_pair(index, true)) );
    }
}



}

