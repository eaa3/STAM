/** @file STAM.cpp
 * (modified for Visual Odometry and Graph Slam by Saif Sidhik (sxs1412@student.bham.ac.uk))
 * 
 * @author  Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author  Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 * @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
 * @version 1.0
 *
 */

#include <typeinfo>////
#include "STAM.h"

int SCENE = 1;


double baseline[] = { 175, 50, 80 };
std::string points2d_init_file[] = { "S01_INPUT/S01_2Ddata_dst_init.csv", "S02_INPUT/S02_2Ddata_dst_init.csv", "S03_INPUT/S03_2Ddata_dst_init.csv" };
std::string points3d_init_file[] = { "S01_INPUT/S01_3Ddata_dst_init.csv", "S02_INPUT/S02_3Ddata_dst_init.csv", "S03_INPUT/S03_3Ddata_dst_init.csv" };
std::string intrinsics_file[] = { "S01_INPUT/intrinsicsS01.xml", "S02_INPUT/intrinsicsS02.xml", "S03_INPUT/intrinsicsS03.xml" };
std::string next_frame_fmt[] = { "S01_INPUT/S01L03_VGA/S01L03_VGA_%04d.png", "S02_INPUT/S02L03_VGA/S02L03_VGA_%04d.png", "S03_INPUT/S03L03_VGA/S03L03_VGA_%04d.png"};
std::string template_file_fmt[] = {"S01_INPUT/S01L03_patch/S01L03_VGA_patch_%04d.png", "S02_INPUT/S02L03_patch/S02L03_VGA_patch_%04d.png", "S03_INPUT/S03L03_VGA_patch/S03L03_VGA_patch_%04d.png"};

// const double THR_BASELINE = baseline[SCENE-1];
// const std::string POINTS_2D_INIT_FILE = points2d_init_file[SCENE-1];
// const std::string POINTS_3D_INIT_FILE = points3d_init_file[SCENE-1];
// const std::string INTRINSICS_FILE = intrinsics_file[SCENE-1];
// const std::string NEXT_FRAME_FMT = next_frame_fmt[SCENE-1];
// const std::string TEMPL_FILE_FMT = template_file_fmt[SCENE-1];

namespace visual_odometry {

STAM::STAM(bool marker_flag):marker_flag_(marker_flag){}


// ----- use only for stam dataset hardcoded to the locations above
bool STAM::init(cv::Mat image){

    params.baseline_thr = baseline[SCENE-1];
    params.POINTS_2D_INIT_FILE = points2d_init_file[SCENE-1];
    params.POINTS_3D_INIT_FILE = points3d_init_file[SCENE-1];
    params.INTRINSICS_FILE = intrinsics_file[SCENE-1];
    params.NEXT_FRAME_FMT = next_frame_fmt[SCENE-1];
    params.TEMPL_FILE_FMT = template_file_fmt[SCENE-1];

    loadIntrinsicsFromFile(params.INTRINSICS_FILE);
    // initFromFiles(image, params.POINTS_2D_INIT_FILE, params.POINTS_3D_INIT_FILE);
    initFromTemplates(image, params.POINTS_3D_INIT_FILE, params.TEMPL_FILE_FMT);


    return true;

}

// ----- init with templates. finds the positions of the templates in the initial image. requires p3d of the template points. baselline adjustable.
bool STAM::init(cv::Mat img, std::string nxtFrame, std::string intrinsic_file, std::string points3d_file, std::string template_file_format, const double base_line)
{
    params.baseline_thr = base_line;
    params.POINTS_3D_INIT_FILE = points3d_file;
    params.INTRINSICS_FILE = intrinsic_file;
    params.NEXT_FRAME_FMT = nxtFrame;
    params.TEMPL_FILE_FMT = template_file_format;
    std::cout << "BaseLine Threshold: " << params.baseline_thr << std::endl;

    loadIntrinsicsFromFile(params.INTRINSICS_FILE);

    initFromTemplates(img,params.POINTS_3D_INIT_FILE, params.TEMPL_FILE_FMT);
}

// ----- init with checkerboard with known coordiates (of 4 corners)
bool STAM::init(cv::Mat img, std::string nxtFrame, std::string intrinsic_file, std::string points3d_file, int corners_per_row, int corners_per_col, const double base_line)
{
    params.baseline_thr = base_line;
    params.POINTS_3D_INIT_FILE = points3d_file;
    params.INTRINSICS_FILE = intrinsic_file;
    params.NEXT_FRAME_FMT = nxtFrame;
    std::cout << "BaseLine Threshold: " << params.baseline_thr << std::endl;

    loadIntrinsicsFromFile(params.INTRINSICS_FILE);
    initFromCheckerboard(img, corners_per_row, corners_per_col, params.POINTS_3D_INIT_FILE);
}

// ----- init with checkerboard with unknown coordinates
bool STAM::init(cv::Mat img, std::string nxtFrame, std::string intrinsic_file, int corners_per_row, int corners_per_col, const double base_line)
{
    params.baseline_thr = base_line;
    params.INTRINSICS_FILE = intrinsic_file;
    params.NEXT_FRAME_FMT = nxtFrame;
    std::cout << "BaseLine Threshold: " << params.baseline_thr << std::endl;

    loadIntrinsicsFromFile(params.INTRINSICS_FILE);
    initFromCheckerboardDefaultOrigin(img, corners_per_row, corners_per_col);
}

// -----  requires the coordinates of the 4 checkerboard inner corners (top-left, top-right, bottom-left, bottom-right)
void STAM::initFromCheckerboard(cv::Mat image, int corners_per_row, int corners_per_col, const std::string& p3D_filename)
{
    cv::Point3f p3D;  //
    cv::Point2f p2D;  //
    std::vector<cv::Point2f> p2D_vec; // storing detected chessboard corners
    bool invert_checkerboard = false;
    // std::cout << image << std::endl;
    Frame::Ptr frame(new Frame(image));
    trackset_.image_ = image;
    // std::cout << " frame " << image << std::endl;
    cv::cvtColor(image, image, CV_BGR2GRAY);
    cv::Size patternsize(corners_per_row,corners_per_col);
    bool patternfound = cv::findChessboardCorners(image, patternsize, p2D_vec, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE+ cv::CALIB_CB_FAST_CHECK);

    if(patternfound)
    {   std::cout << "found checkerboard" << std::endl;
        cv::cornerSubPix(image, p2D_vec, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.1));
    }
    else 
    {
        std::cout << "ERROR! NO CHECKERBOARD FOUND TO INITIALISE STAM! VERIFY CORNERS PER ROW AND COLUMN.\nABORTING VISUAL ODOMETRY " << std::endl;
        exit(1);
    }
    // Reading the checkerboard marker points from csv file

    if ((p2D_vec[0].x- p2D_vec[p2D_vec.size()-1].x > 0.0) && (p2D_vec[0].y - p2D_vec[p2D_vec.size()-1].y > 0 ))
        invert_checkerboard = true;

    std::vector<cv::Point3f> p3D_vec = utils::find3DCheckerboardCorners(p3D_filename, corners_per_row, corners_per_col, invert_checkerboard);

    assert(p2D_vec.size() == corners_per_row*corners_per_col && p2D_vec.size() == p3D_vec.size());
    for (int i = 0; i < p2D_vec.size(); ++i)
    {

        // ======= DEBUG =================
        // std::cout << p2D_vec[i] << std::endl;

        //// ----- Visualize chessboard corners ----------------
        // cv::circle(image, p2D_vec[i],10,cv::Scalar(255,0,255),-1);
        // cv::imshow("window",image);
        // cv::waitKey(50);
        //// ============================

        auto added_ids = memory_.addCorrespondence(frame->id_, p2D_vec[i], p3D_vec[i]);
        frame->keypoints.push_back( cv::KeyPoint(p2D_vec[i].x, p2D_vec[i].y, 1) );

        frame->ids_.push_back( added_ids.first );

        trackset_.points2D_.push_back( p2D_vec[i] );
        trackset_.ids_.push_back( added_ids.first );
    }
    // cv::waitKey(0);
    // std::cout << p2D_vec << std::endl;
    // std::cout << p3D_vec << std::endl;
    frame->detectAndDescribe();

    frame->projMatrix = calcProjMatrix(false, frame->r, frame->t);
    trackset_.projMatrix = frame->projMatrix;

    key_frames_.push_back(frame);

    // Not used
    previous_frame_ = frame;

    memory_.addKeyFrame(frame->id_, frame->r, frame->t, intrinsics_, distortion_(cv::Range(0,1),cv::Range(0,5)));

}

// ----- does not require the 3d positions of the checkerboard corners. Uses the bottom left inner-corner of the checkerboard as the default origin
void STAM::initFromCheckerboardDefaultOrigin(cv::Mat image, int corners_per_row, int corners_per_col)
{
    cv::Point3f p3D;  //
    cv::Point2f p2D;  //
    std::vector<cv::Point2f> p2D_vec; // storing detected chessboard corners
    bool invert_checkerboard = false;
    // std::cout << image << std::endl;
    Frame::Ptr frame(new Frame(image));
    trackset_.image_ = image;
    // std::cout << " frame " << image << std::endl;
    cv::cvtColor(image, image, CV_BGR2GRAY);
    cv::Size patternsize(corners_per_row,corners_per_col);
    bool patternfound = cv::findChessboardCorners(image, patternsize, p2D_vec, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE+ cv::CALIB_CB_FAST_CHECK);

    if(patternfound)
    {   std::cout << "found checkerboard" << std::endl;
        cv::cornerSubPix(image, p2D_vec, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.1));
    }
    else 
    {
        std::cout << "ERROR! NO CHECKERBOARD FOUND TO INITIALISE STAM! VERIFY CORNERS PER ROW AND COLUMN.\nABORTING VISUAL ODOMETRY " << std::endl;
        exit(1);
    }
    // Reading the checkerboard marker points from csv file

    if ((p2D_vec[0].x- p2D_vec[p2D_vec.size()-1].x > 0.0) && (p2D_vec[0].y - p2D_vec[p2D_vec.size()-1].y > 0 ))
        invert_checkerboard = true;

    std::vector<cv::Point3f> p3D_vec = utils::find3DCheckerboardCorners(corners_per_row, corners_per_col, invert_checkerboard);

    assert(p2D_vec.size() == corners_per_row*corners_per_col && p2D_vec.size() == p3D_vec.size());
    for (int i = 0; i < p2D_vec.size(); ++i)
    {

        // ======= DEBUG =================
        // std::cout << p2D_vec[i] << std::endl;

        //// ----- Visualize chessboard corners ----------------
        // cv::circle(image, p2D_vec[i],10,cv::Scalar(255,0,255),-1);
        // cv::imshow("window",image);
        // cv::waitKey(50);
        //// ============================
        auto added_ids = memory_.addCorrespondence(frame->id_, p2D_vec[i], p3D_vec[i]);
        frame->keypoints.push_back( cv::KeyPoint(p2D_vec[i].x, p2D_vec[i].y, 1) );

        frame->ids_.push_back( added_ids.first );

        trackset_.points2D_.push_back( p2D_vec[i] );
        trackset_.ids_.push_back( added_ids.first );
    }
    // cv::waitKey(0);
    // std::cout << p2D_vec << std::endl;
    // std::cout << p3D_vec << std::endl;
    frame->detectAndDescribe();

    frame->projMatrix = calcProjMatrix(false, frame->r, frame->t);
    trackset_.projMatrix = frame->projMatrix;

    key_frames_.push_back(frame);

    // Not used
    previous_frame_ = frame;

    memory_.addKeyFrame(frame->id_, frame->r, frame->t, intrinsics_, distortion_(cv::Range(0,1),cv::Range(0,5)));

}

// only returns value if new world points are obtained (i.e. newly triangulated points)
std::vector<cv::Point3f> STAM::getNew3dPoints()
{
    std::vector<cv::Point3f> ret_val;
    static int keypoint_counter = memory_.map_.begin()->first;
    
    for(auto it = memory_.map_.begin(); it != memory_.map_.end(); it++)
    {
        if (it->first == keypoint_counter)
        {
            ret_val.push_back(it->second);
            keypoint_counter++;
            // std::cout << it -> second << std::endl;
        }
    }
    return ret_val;

}

Frame::Ptr STAM::process(cv::Mat image, bool visualize_flag){
    if( image.empty() )
        return Frame::Ptr();

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

    // // ++++++++++++++++++++++++++

    // cv::Mat out;
    // std::vector<cv::KeyPoint> key_points;
    // cv::KeyPoint::convert(curr2dPts_, key_points);
    // cv::drawKeypoints(image, key_points,out);
    // cv::imshow("stam",out);
    // cv::waitKey(1);

    // // ==========================

    cv::Mat R1 = current_frame->projMatrix(cv::Range::all(), cv::Range(0, 3));
    cv::Mat T1 = current_frame->projMatrix(cv::Range::all(), cv::Range(3, 4));

    cv::Mat Pose(3, 4, CV_64FC1);
    cv::Mat pos, R;
    R = R1.inv();
    pos = (-R) * T1;

    R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
    pos.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
    current_frame->pose = Pose;
    // std::cout << " R " << R << " pos " << pos << std::endl;
    // std:: cout << Pose << std::endl;

    // for S01: baseline= 175
    // for S02: baseline= 50
    // for S03: baseline= 25 or 40

    // last keyframe
    Frame::Ptr key_frame = key_frames_.back();

    bool enough_baseline = has_enough_baseline(key_frame->projMatrix, current_frame->projMatrix, params.baseline_thr);

    if (enough_baseline) { // keyframe interval // change here from 5 to any other number
        current_frame->detectAndDescribe();

        mapping(key_frame, current_frame);
    }
    if (visualize_flag)
        projectAndShow(current_frame->projMatrix, image);

    //previousFrame = currentFrame;

    return current_frame;

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

    //LOG("Baseline %f\n", baseline);
    return baseline >= thr_baseline;
}

void STAM::loadIntrinsicsFromFile(const std::string& filename){


    cv::FileStorage cvfs(filename.c_str(),CV_STORAGE_READ);
    if( cvfs.isOpened() ){
        cvfs["mat_intrinsicMat"] >> intrinsics_;
        cvfs["mat_distortionMat"] >> distortion_;

        printf("Loaded distortion %d %d\n", distortion_.rows, distortion_.cols);
        //cv::waitKey(0);
    }
}

void STAM::initFromTemplates(cv::Mat image, const std::string& p3D_filename, const std::string& template_format)
{   // use template matching to find the 2D image coordinates of the given patches and match the corresponding 3D coordinates
    cv::Point3f p3D;
    cv::Point2f p2D;
    FILE *p3D_f = fopen(p3D_filename.c_str(), "r");
    int findex = 0;
    cv::Mat templ;  
    // std::cout << image << std::endl;
    Frame::Ptr frame(new Frame(image));
    trackset_.image_ = image;
    cv::cvtColor(image, image, CV_BGR2GRAY);
    // std::cout << " frame " << frame->id_ << std::endl;

    while (fscanf(p3D_f, "%f,%f,%f", &p3D.x, &p3D.y, &p3D.z) == 3)
    {
        char buf[256];
        sprintf(buf, template_format.c_str(), findex++);
        templ = cv::imread(buf,0);

        // ------Uncomment to reduce template size. may speed up the process

        // resize(templ,templ,Size(templ.cols/1.5,templ.rows/1.5));

        // -------------

        cv::Mat result(image.rows - templ.rows + 1,image.cols - templ.cols + 1,CV_32FC1);

        cv::matchTemplate(image,templ,result,CV_TM_CCOEFF_NORMED);

        double maxv; cv::Point maxp;
        cv::minMaxLoc(result,0,&maxv,0,&maxp);
        p2D.x = ((2*maxp.x)+templ.cols)/2;
        p2D.y = ((2*maxp.y)+templ.rows)/2;
        // p3D *= 1000.00;
        // std::cout << "2D : " << p2D.x << " " << p2D.y << std::endl;
        // std::cout << "3D : " << p3D.x << " " << p3D.y << " " <<p3D.z << std::endl;

        auto added_ids = memory_.addCorrespondence(frame->id_, p2D, p3D);
        frame->keypoints.push_back( cv::KeyPoint(p2D.x, p2D.y, 1) );

        frame->ids_.push_back( added_ids.first );

        trackset_.points2D_.push_back( p2D );
        trackset_.ids_.push_back( added_ids.first );
    }

    frame->detectAndDescribe();

    frame->projMatrix = calcProjMatrix(false, frame->r, frame->t);
    trackset_.projMatrix = frame->projMatrix;

    key_frames_.push_back(frame);

    // Not used
    previous_frame_ = frame;

    memory_.addKeyFrame(frame->id_, frame->r, frame->t, intrinsics_, distortion_(cv::Range(0,1),cv::Range(0,5)));

    fclose(p3D_f);
}

void STAM::initFromTemplatesRandom(cv::Mat image, const std::string& p3D_filename, const std::string& template_format)
{   // use template matching to find the 2D image coordinates of the given patches and match the corresponding 3D coordinates
    cv::Point3f p3D;
    cv::Point2f p2D;
    FILE *p3D_f = fopen(p3D_filename.c_str(), "r");
    int findex = 0;
    cv::Mat templ;  
    // std::cout << image << std::endl;
    Frame::Ptr frame(new Frame(image));
    trackset_.image_ = image;
    cv::cvtColor(image, image, CV_BGR2GRAY);

    while (fscanf(p3D_f, "%f,%f,%f", &p3D.x, &p3D.y, &p3D.z) == 3)
    {
        char buf[256];
        sprintf(buf, template_format.c_str(), findex++);
        templ = cv::imread(buf,0);

        // ------Uncomment to reduce template size. may speed up the process

        // resize(templ,templ,Size(templ.cols/1.5,templ.rows/1.5));

        // -------------

        cv::Mat result(image.rows - templ.rows + 1,image.cols - templ.cols + 1,CV_32FC1);

        cv::matchTemplate(image,templ,result,CV_TM_CCOEFF_NORMED);

        double maxv; cv::Point maxp;
        cv::minMaxLoc(result,0,&maxv,0,&maxp);
        p2D.x = ((2*maxp.x)+templ.cols)/2;
        p2D.y = ((2*maxp.y)+templ.rows)/2;
        p3D.x = p2D.x;
        p3D.y = p2D.y;
        p3D.z = 1000;

        auto added_ids = memory_.addCorrespondence(frame->id_, p2D, p3D);
        frame->keypoints.push_back( cv::KeyPoint(p2D.x, p2D.y, 1) );

        frame->ids_.push_back( added_ids.first );

        trackset_.points2D_.push_back( p2D );
        trackset_.ids_.push_back( added_ids.first );
    }

    frame->detectAndDescribe();

    frame->projMatrix = calcProjMatrix(false, frame->r, frame->t);
    trackset_.projMatrix = frame->projMatrix;

    key_frames_.push_back(frame);

    // Not used
    previous_frame_ = frame;

    memory_.addKeyFrame(frame->id_, frame->r, frame->t, intrinsics_, distortion_(cv::Range(0,1),cv::Range(0,5)));

    fclose(p3D_f);
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

    frame->projMatrix = calcProjMatrix(false, frame->r, frame->t);
    trackset_.projMatrix = frame->projMatrix;

    key_frames_.push_back(frame);

    // Not used
    previous_frame_ = frame;

    memory_.addKeyFrame(frame->id_, frame->r, frame->t, intrinsics_, distortion_(cv::Range(0,1),cv::Range(0,5)));

    fclose(p2D_f);
    fclose(p3D_f);
}


cv::Mat STAM::calcProjMatrix(bool use_guess, cv::Mat& guess_r, cv::Mat& guess_t) {

    std::vector<cv::Point3f> points3d;
    std::vector<cv::Point2f> points2d;

    for (int i = std::max<int>(0,(trackset_.points2D_.size()-100)); i < trackset_.points2D_.size(); i++) {

        if( trackset_.ids_[i] >= 0 ){
            points3d.push_back(memory_.map_[trackset_.ids_[i]]);
            points2d.push_back(trackset_.points2D_[i]);
        }
    }
    curr3dPts_.clear();
    curr2dPts_.clear();
    for (int i = 0; i < trackset_.points2D_.size(); i++) {

        if( trackset_.ids_[i] >= 0 ){
            curr3dPts_.push_back(memory_.map_[trackset_.ids_[i]]);
            curr2dPts_.push_back(trackset_.points2D_[i]);
        }
    }

    std::cout << "STAM Keypoints in current frame: " << curr3dPts_.size() << " " << curr2dPts_.size() << std::endl;

    // std::cout << " vectors 2d : \n" << points3d << std::endl; 

    //        bool cv::solvePnPRansac   (   InputArray  objectPoints,
    //        InputArray    imagePoints,
    //        InputArray    cameraMatrix,
    //        InputArray    distCoeffs,
    //        OutputArray   rvec,
    //        OutputArray   tvec,
    //        bool  useExtrinsicGuess = false,
    //        int   iterationsCount = 100,
    //        float     reprojectionError = 8.0,
    //        double    confidence = 0.99,
    //        OutputArray   inliers = noArray(),
    //        int   flags = SOLVEPNP_ITERATIVE
    //        )

    // curr3dPts_ = points3d;
    // curr2dPts_ = points2d;
    //LOG("Number of Points %d\n", points3d.size());

    cv::solvePnPRansac(points3d, points2d, intrinsics_, distortion_, guess_r, guess_t, use_guess, 100, 5.0, 0.99);
    cv::Mat rvec = guess_r, tvec = guess_t;
    cv::Mat R(3, 3, CV_64FC1);
    cv::Rodrigues(rvec, R);

    cv::Mat Pose(3,4, R.type());
    R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
    cv::Mat projMatrix = Pose;


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
    bool success = matchAndTriangulate(key_frame, current_frame, intrinsics_, distortion_);
    //LOG("AFTER: %d\n", currentFrame->squareFeatures.size());

    if( success ) {
        // This frame becomes a keyframe
        key_frames_.push_back(current_frame);

        memory_.addKeyFrame(current_frame->id_, current_frame->r, current_frame->t, intrinsics_, distortion_(cv::Range(0,1),cv::Range(0,5)));

    }



}

bool STAM::matchAndTriangulate(Frame::Ptr& key_frame, Frame::Ptr& current_frame, cv::Mat intrinsics, cv::Mat distortion) {

    std::vector<cv::DMatch> matches;

    matcher.match(current_frame->descriptors, key_frame->descriptors, matches, current_frame->keypoints, key_frame->keypoints);

    std::vector<bool> mask(current_frame->keypoints.size(), false);

    // LOG("NMatches %lu\n",matches.size());

    if( !matches.size() )
        return false;

    std::vector<cv::Point2f> previousTriangulate, currentTriangulate;
    cv::Mat outputTriangulate;
    outputTriangulate.create(cv::Size(4, matches.size()), CV_32FC1);

    // Points for triangulation (i.e. points in current_frame that doesn't have 3D points already)
    std::vector<int> triangulation_indices;
    std::vector<cv::Point3f> points3D;
    std::vector<cv::Point2f> points2D;
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

            points2D.push_back(pt2);
            points3D.push_back(memory_.map_[p3d_id]);

        }
        // marking points which had matches.
        // Later we can use the negation of this mask to extract the points that had no matchings
        mask[matches[i].queryIdx] = true;

    }

    if( previousTriangulate.size() == 0 || currentTriangulate.size() == 0 ){
        //LOG("Triangulation Points %d %d\n",previousTriangulate.size(),currentTriangulate.size());
        return false;
    }

    // undistort
    std::vector<cv::Point2f> previousTriangulateUnd, currentTriangulateUnd;
    cv::undistortPoints(previousTriangulate, previousTriangulateUnd, intrinsics, distortion);
    cv::undistortPoints(currentTriangulate, currentTriangulateUnd, intrinsics, distortion);

    cv::triangulatePoints(key_frame->projMatrix, current_frame->projMatrix, previousTriangulateUnd, currentTriangulateUnd, outputTriangulate);

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

        points2D.push_back(p2D);
        points3D.push_back(memory_.map_[p3d_id]);


    }


    // Compute pose again
//    if( points3D.size() >= 20 ){
//        cv::Mat guess_r = current_frame->r, guess_t = current_frame->t;
//        cv::solvePnPRansac(points3D, points2D, intrinsics_, distortion_, guess_r, guess_t, false, 100, 8.0, 0.99);
//        current_frame->r = guess_r, current_frame->t = guess_t;
//        cv::Mat R;
//        cv::Rodrigues(current_frame->r, R);

//        cv::Mat Pose(3,4, R.type());
//        R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
//        current_frame->t.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
//        current_frame->projMatrix = intrinsics_*Pose;
//    }



    return true;
}

void STAM::projectAndShow(cv::Mat projMatrix, cv::Mat image) 
{

    cv::Mat point3D;
    point3D.create(cv::Size(1, 4), CV_64FC1);

    cv::Mat result;

    bool marker_flag = marker_flag_; // if true, the keypoints will be visualised in the frames

    cv::Mat imcopy = image.clone();
    if (marker_flag)
    {

        for(auto it = memory_.map_.begin(); it != memory_.map_.end(); it++)
        {

            point3D.at<double>(0) = it->second.x;
            point3D.at<double>(1) = it->second.y;
            point3D.at<double>(2) = it->second.z;
            point3D.at<double>(3) = 1;
            // std::cout << point3D << std::endl;

            cv::Mat p3D;
            p3D.create(cv::Size(1, 4), CV_64FC1);
            p3D.at<double>(0) = it->second.x;
            p3D.at<double>(1) = it->second.y;
            p3D.at<double>(2) = it->second.z;
            p3D.at<double>(2) = 1;
            cv::Mat p3DCam = (projMatrix)*p3D;

            float r = 40.0/(abs(p3D.at<double>(1))+1);
            int ri = 255*r;
            cv::gemm(intrinsics_*projMatrix,  point3D , 1, 0, 0, result);
            cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)),4,cv::Scalar(ri,0,255),-1);
            cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)), 3, cv::Scalar(ri, 255, 255), -1);
            cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)), 2, cv::Scalar(ri, 0, 255), -1);

        }

    }

    cv::imshow("frame", imcopy);
    if (cv::waitKey(1) == 27) 
    {
        exit(0);
    }

}


void STAM::optimise() {
    memory_.optimise();
    updateOptimisedKF();
}

void STAM::dump() {
    memory_.dumpMapAndKFExtrinsics();
}

void STAM::updateOptimisedKF(){
    auto it = key_frames_.begin();
    for(;it!=key_frames_.end(); it++)
        memory_.updateKF(*it);
}


}

