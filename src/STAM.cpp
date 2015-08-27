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

bool STAM::init(){

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
        vo_utils::matchAndTriangulate(*keyFrame, *currentFrame, currentFrame->intrinsic, currentFrame->distortion);
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
        //squareFeatures[index].kp_ = cv::KeyPoint(x, y, 1);
        //squareFeatures[index].type_ = Feature::SQUARE;
        index++;
    }

    fclose(f);

}

void STAM::load3DPointsFromFile(const std::string& filename){

    FILE *f = fopen(filename.c_str(), "r");
    cv::Point3f p;
    int index = 0;

    while (fscanf(f, "%f,%f,%f", &p.x, &p.y, &p.z) == 3){
        //squareFeatures[index].p3D_ = p;
        index++;
    }

    fclose(f);

}


}

