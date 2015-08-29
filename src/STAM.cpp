#include "STAM.h"



int SCENE = 1;


double baseline[] = { 175, 50, 40 };
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

bool STAM::init(cv::Mat image){

    params.baseline_thr = baseline[SCENE-1];
    params.POINTS_2D_INIT_FILE = points2d_init_file[SCENE-1];
    params.POINTS_3D_INIT_FILE = points3d_init_file[SCENE-1];
    params.INTRINSICS_FILE = intrinsics_file[SCENE-1];
    params.NEXT_FRAME_FMT = next_frame_fmt[SCENE-1];

    loadIntrinsicsFromFile(params.INTRINSICS_FILE);
    initFromFiles(image, params.POINTS_2D_INIT_FILE, params.POINTS_3D_INIT_FILE);


    /*
    previousFrame->loadKpFromFile(POINTS_2D_INIT_FILE);
    previousFrame->load3DPointsFromFile(POINTS_3D_INIT_FILE);
    previousFrame->loadIntrinsicsFromFile(INTRINSICS_FILE);
    currentFrame->loadIntrinsicsFromFile(INTRINSICS_FILE);

    previousFrame->readNextFrame(NEXT_FRAME_FMT);

    previousFrame->calcProjMatrix();

    previousFrame->detectAndDescribe();

    keyFrame = previousFrame;
    */

    return true;

}


void STAM::process(cv::Mat image){


    Frame::Ptr current_frame = Frame::Ptr(new Frame(image));


    updateUsingKLT(image);
    auto it_id = trackset_.ids_.begin();
    for(auto it = trackset_.points2D_.begin(); it != trackset_.points2D_.end() && it_id != trackset_.ids_.end(); it++, it_id++){
        if( *it_id >= 0 ) {
            current_frame->keypoints.push_back( cv::KeyPoint(it->x, it->y, 1) );
            current_frame->ids_.push_back(*it_id);
        }
    }



    current_frame->projMatrix = calcProjMatrix(false, current_frame->r, current_frame->t);

    // for S01: baseline= 175
    // for S02: baseline= 50
    // for S03: baseline= 25 or 40

    Frame::Ptr key_frame = key_frames_.back();

    bool enough_baseline = has_enough_baseline(intrinsics_.inv()*key_frame->projMatrix, intrinsics_.inv()*current_frame->projMatrix, params.baseline_thr);

    if (enough_baseline) { // keyframe interval // change here from 5 to any other number
        current_frame->detectAndDescribe();

        mapping(key_frame, current_frame);
    }

    projectAndShow(current_frame->projMatrix, image);

    //previousFrame = currentFrame;


    /*
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


    previousFrame = currentFrame;*/

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
        cvfs["mat_intrinsicMat"] >> intrinsics_;
        cvfs["mat_distortionMat"] >> distortion_;
    }
}


void STAM::initFromFiles(cv::Mat image, const std::string& p2D_filename, const std::string& p3D_filename){
    FILE *p2D_f = fopen(p2D_filename.c_str(), "r");
    FILE *p3D_f = fopen(p3D_filename.c_str(), "r");

    cv::Point3f p3D;
    cv::Point2f p2D;

    Frame::Ptr frame(new Frame(image));
    trackset_.image_ = image;

    while( fscanf(p3D_f, "%f,%f,%f", &p3D.x, &p3D.y, &p3D.z) == 3 && fscanf(p2D_f, "%f,%f", &p2D.x, &p2D.y) == 2 ){

        // return: (p3d_id, p2d_id)
        auto added_ids = memory_.addCorrespondence(frame->id_, p2D, p3D);

        frame->keypoints.push_back( cv::KeyPoint(p2D.x, p2D.y, 1) );
        // Indicate the 3D point this keypoint refers to
        frame->ids_.push_back( added_ids.first );

        trackset_.points2D_.push_back( p2D );
        trackset_.ids_.push_back( added_ids.first );

    }

    frame->detectAndDescribe();

    frame->projMatrix = calcProjMatrix();
    trackset_.projMatrix = frame->projMatrix;

    key_frames_.push_back(frame);

    previous_frame_ = frame;

    fclose(p2D_f);
    fclose(p3D_f);
}


cv::Mat STAM::calcProjMatrix(bool use_guess, cv::Mat guess_r, cv::Mat guess_t) {

    std::vector<cv::Point3f> points3d;
    std::vector<cv::Point2f> points2d;

    for (int i = std::max<int>(0,(trackset_.points2D_.size()-40)); i < trackset_.points2D_.size(); i++) {

        if( trackset_.ids_[i] >= 0 ){
            points3d.push_back(memory_.map_[trackset_.ids_[i]]);
            points2d.push_back(trackset_.points2D_[i]);
        }
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
    cv::solvePnPRansac(points3d, points2d, intrinsics_, distortion_, rvec, tvec, use_guess, 100, 5.0, 0.99);
    rvec.copyTo(guess_r); tvec.copyTo(guess_t);
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    cv::Mat Pose(3,4, R.type());
    R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
    cv::Mat projMatrix = intrinsics_*Pose;

//    printProjMatrix();

    return projMatrix;
}

void STAM::updateUsingKLT(cv::Mat image) {


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

    cv::calcOpticalFlowPyrLK(trackset_.image_, image, trackset_.points2D_, currentPoints, status, errors); //cv::Size(21,21), 3, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01) , cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4 ); //1e-4

    std::vector<cv::Point2f> filtered_p2D;
    std::vector<int> filtered_ids;
    for (int i = 0; i < trackset_.points2D_.size(); i++) {
        if (status[i] != 0 && errors.at<float>(i) < 12.0f) { // klt ok!

            filtered_p2D.push_back(currentPoints[i]);
            filtered_ids.push_back(trackset_.ids_[i]);

            // Meaning: f is visible in this frame

        }
    }

    trackset_.points2D_ = filtered_p2D;
    trackset_.ids_ = filtered_ids;
    trackset_.image_ = image.clone();
}

void STAM::mapping(Frame::Ptr key_frame, Frame::Ptr current_frame){




    //LOG("BEFORE: %d\t", frame->squareFeatures.size());
    matchAndTriangulate(key_frame, current_frame, intrinsics_, distortion_);
    //LOG("AFTER: %d\n", currentFrame->squareFeatures.size());

    // This frame becomes a keyframe
    key_frames_.push_back(current_frame);


}

void STAM::matchAndTriangulate(Frame::Ptr& key_frame, Frame::Ptr& current_frame, cv::Mat intrinsics, cv::Mat distortion) {

    std::vector<cv::DMatch> matches;

    matcher.match(current_frame->descriptors, key_frame->descriptors, matches, current_frame->keypoints, key_frame->keypoints);

    std::vector<bool> mask(current_frame->keypoints.size(), false);




    LOG("NMatches %d\n",matches.size());

    std::vector<cv::Point2f> previousTriangulate, currentTriangulate;
    cv::Mat	outputTriangulate;
    outputTriangulate.create(cv::Size(4, matches.size()), CV_32FC1);

    // Points for triangulation (i.e. points in current_frame that doesn't have 3D points already)
    std::vector<int> triangulation_indices;
    for (int i = 0; i < matches.size(); i++) {

        if( key_frame->ids_[matches[i].trainIdx] < 0 ){
            cv::Point pt1 = key_frame->keypoints[matches[i].trainIdx].pt;
            cv::Point pt2 = current_frame->keypoints[matches[i].queryIdx].pt;

            previousTriangulate.push_back(pt1);
            currentTriangulate.push_back(pt2);

            triangulation_indices.push_back(matches[i].queryIdx);

        }
        // Otherwise, we need to update a new observation for this point which already has a 3D correspondence
        else {
            cv::Point2f pt2 = current_frame->keypoints[matches[i].queryIdx].pt;
            int p3d_id = key_frame->ids_[matches[i].trainIdx];

            memory_.addCorrespondence(current_frame->id_, pt2, p3d_id);

            // we also add those points to the trackingset_
            trackset_.points2D_.push_back(pt2);
            trackset_.ids_.push_back(p3d_id);

        }
        // marking points which had matches.
        // Later we can use the negation of this mask to extract the points that had no matchings
        mask[matches[i].queryIdx] = true;

    }



    if( previousTriangulate.size() == 0 || currentTriangulate.size() == 0 ){
        //LOG("Triangulation Points %d %d\n",previousTriangulate.size(),currentTriangulate.size());
        return;
    }




    // undistort
    std::vector<cv::Point2f> previousTriangulateUnd, currentTriangulateUnd;
    cv::undistortPoints(previousTriangulate, previousTriangulateUnd, intrinsics, distortion);
    cv::undistortPoints(currentTriangulate, currentTriangulateUnd, intrinsics, distortion);

    cv::triangulatePoints(intrinsics.inv()*key_frame->projMatrix, intrinsics.inv()*current_frame->projMatrix, previousTriangulateUnd, currentTriangulateUnd, outputTriangulate);

    for (int i = 0; i < triangulation_indices.size(); i++) {

        cv::Point2f p2D = currentTriangulate[i];
        cv::Point3f p3D = cv::Point3f(outputTriangulate.at<float>(0, i) / outputTriangulate.at<float>(3, i),
                                      outputTriangulate.at<float>(1, i) / outputTriangulate.at<float>(3, i),
                                      outputTriangulate.at<float>(2, i) / outputTriangulate.at<float>(3, i)
                                      );



        auto added_ids = memory_.addCorrespondence(current_frame->id_, p2D, p3D);
        int p3d_id = added_ids.first;

        // Updating the fact that this 2D point now has a 3D correspondence
        current_frame->ids_[triangulation_indices[i]] = p3d_id;

        // we also add those points to the trackingset_
        trackset_.points2D_.push_back(p2D);
        trackset_.ids_.push_back(p3d_id);

    }
}

void STAM::projectAndShow(cv::Mat projMatrix, cv::Mat image) {

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



    //std::max<int>(0,(squareFeatures.size()-40))

    for(auto it = memory_.map_.begin(); it != memory_.map_.end(); it++){

        point3D.at<double>(0) = it->second.x;
        point3D.at<double>(1) = it->second.y;
        point3D.at<double>(2) = it->second.z;
        point3D.at<double>(3) = 1;

        cv::Mat p3D;
        p3D.create(cv::Size(1, 4), CV_64FC1);
        p3D.at<double>(0) = it->second.x;
        p3D.at<double>(1) = it->second.y;
        p3D.at<double>(2) = it->second.z;
        p3D.at<double>(2) = 1;
        cv::Mat p3DCam = (intrinsics_.inv()*projMatrix)*p3D;

        float r = 40.0/(abs(p3D.at<double>(1))+1);
        int ri = 255*r;
        cv::gemm(projMatrix,  point3D , 1, 0, 0, result);
        cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)),4,cv::Scalar(ri,0,255),-1);
        cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)), 3, cv::Scalar(ri, 255, 255), -1);
        cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)), 2, cv::Scalar(ri, 0, 255), -1);



    }



    cv::imshow("frame", imcopy);
    if (cv::waitKey(1) == 27) exit(0);
}




}

