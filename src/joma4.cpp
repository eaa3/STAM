#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/video/video.hpp"

#include <utils.h>

#include <vector>

#include "types.h"

#include <algorithm>

#include <cvsba/cvsba.h>


int SCENE = 1;


double baseline[] = { 175, 50, 25 };
std::string points2d_init_file[] = { "S01_2Ddata_dst_init.csv", "S02_2Ddata_dst_init.csv", "S03_2Ddata_dst_init.csv" };
std::string points3d_init_file[] = { "S01_3Ddata_dst_init.csv", "S02_3Ddata_dst_init.csv", "S03_3Ddata_dst_init.csv" };
std::string intrinsics_file[] = { "intrinsicsS01.xml", "intrinsicsS02.xml", "intrinsicsS03.xml" };
std::string next_frame_fmt[] = { "S01L03_VGA/S01L03_VGA_%04d.png", "S02L03_VGA/S02L03_VGA_%04d.png", "S03L03_VGA/S03L03_VGA_%04d.png"};

#define THR_BASELINE baseline[SCENE-1]
#define POINTS_2D_INIT_FILE points2d_init_file[SCENE-1]
#define POINTS_3D_INIT_FILE points3d_init_file[SCENE-1]
#define INTRINSICS_FILE intrinsics_file[SCENE-1]
#define NEXT_FRAME_FMT next_frame_fmt[SCENE-1].c_str()


using namespace std;
using namespace visual_odometry;
using namespace visual_odometry::utils;

#define LOG(...) printf(__VA_ARGS__)

//void CalcProjectionMatrix(const vector<cv::Point3f>& pt_3d, const vector<cv::Point2f>& pt_2d, cv::Mat pM);

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

	vector<Feature> squareFeatures;
	vector<Feature> triangleFeatures;
	cv::Mat projMatrix;
    cv::Mat t, r;
	
    // Intrinsics params
    cv::Mat intrinsic;   // intrinsic parameters
    cv::Mat distortion;  // lens distortion coefficients

	cv::Mat image;
	
	vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;

    int id_;


    void calcProjMatrix(cv::Mat guess_r = cv::Mat(), cv::Mat guess_t = cv::Mat()) {

		vector<cv::Point3f> points3d;
		vector<cv::Point2f> points2d;

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

    void detectAndDescribe() {
		char algorithm[] = "SIFT";
		cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create(algorithm);
		cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create(algorithm);

		keypoints.clear();

		detector->detect(image, keypoints);
		descriptor->compute(image, keypoints, descriptors);
	}


    void loadIntrinsicsFromFile(const std::string& filename){


        cv::FileStorage cvfs(filename.c_str(),CV_STORAGE_READ);
        if( cvfs.isOpened() ){
           cvfs["mat_intrinsicMat"] >> intrinsic;
           cvfs["mat_distortionMat"] >> distortion;
        }
    }

    void loadKpFromFile(const std::string& filename){

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

    void load3DPointsFromFile(const std::string& filename){

		FILE *f = fopen(filename.c_str(), "r");
		cv::Point3f p;
		int index = 0;

		while (fscanf(f, "%f,%f,%f", &p.x, &p.y, &p.z) == 3){
			squareFeatures[index].p3D_ = p;
			index++;
		}

		fclose(f);

	}

    void printProjMatrix() {
		static int projIndex = 0;
        LOG("Proj Matrix #%d:", projIndex++);
		for (int i = 0; i < 12; i++) {
            LOG("%s%lf\t", (i % 4 == 0) ? "\n" : "", projMatrix.at<double>(i / 4, i % 4));
		}
        LOG("\n");

		projectAndShow();
	}

    void readNextFrame() {

		static int findex = 0;
		char buf[256];
        sprintf(buf, NEXT_FRAME_FMT, findex++);
        id_ = next_id_s_++;
		image = cv::imread(buf);
		if (image.empty()) {
			exit(0);
		}
	}

    void updateUsingKLT(Frame& previousFrame) {
		
		squareFeatures.clear();
		
		vector<cv::Point2f> previousPoints;
		vector<cv::Point2f> currentPoints;
		vector<uchar> status;
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

    void projectAndShow() {

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

};

int Frame::next_id_s_ = 0;

void matchAndTriangulate(Frame& previousFrame, Frame& currentFrame, cv::Mat intrinsics, cv::Mat distortion) {
	
	static GenericMatcher matcher;
	vector<cv::DMatch> matches;

	matcher.match(currentFrame.descriptors, previousFrame.descriptors, matches, currentFrame.keypoints, previousFrame.keypoints);

    LOG("NMatches %d\n",matches.size());

	vector<cv::Point2f> previousTriangulate, currentTriangulate;
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
	}
}


bool has_enough_baseline(cv::Mat pose1, cv::Mat pose2, double thr_baseline){

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



int main(int argc, char **argv) {

    if( argc < 2 ){
        printf("Usage: \n\n./demo_level3 <scene_number>\n attention: scene_number = 1|2|3\n\n");
        exit(1);
    }else{
        SCENE = atoi(argv[1]);
        if( SCENE < 1 || SCENE > 3 ){
            printf("Usage: \n\n./demo_level3 <scene_number>\n attention: scene_number = 1|2|3\n\n");
            exit(1);
        }
    }
    cv::namedWindow("frame", CV_WINDOW_NORMAL);

	
    Frame::Ptr previousFrame(new Frame(40));
    Frame::Ptr currentFrame(new Frame(40));
    Frame::Ptr keyFrame(new Frame(40));

	cv::initModule_nonfree();
	
    previousFrame->loadKpFromFile(POINTS_2D_INIT_FILE);
    previousFrame->load3DPointsFromFile(POINTS_3D_INIT_FILE);
    previousFrame->loadIntrinsicsFromFile(INTRINSICS_FILE);
    currentFrame->loadIntrinsicsFromFile(INTRINSICS_FILE);

    previousFrame->readNextFrame();

    previousFrame->calcProjMatrix();
	
    previousFrame->detectAndDescribe();

	keyFrame = previousFrame;

    //std::vector<Frame> frames;


	int counter = 1;

	while (true) {
        currentFrame = Frame::Ptr(new Frame(*previousFrame));

        currentFrame->readNextFrame();

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
		counter++;
	}

}
