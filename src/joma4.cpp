#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/video/video.hpp"

#include <utils.h>

#include <vector>

#include "types.h"

using namespace std;
using namespace visual_odometry;
using namespace visual_odometry::utils;

#define LOG(...) printf(__VA_ARGS__)

//void CalcProjectionMatrix(const vector<cv::Point3f>& pt_3d, const vector<cv::Point2f>& pt_2d, cv::Mat pM);

class Frame {

public:

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
	
    // Intrinsics params
    cv::Mat intrinsic;   // intrinsic parameters
    cv::Mat distortion;  // lens distortion coefficients

	cv::Mat image;
	
	vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;

    void calcProjMatrix() {

		vector<cv::Point3f> points3d;
		vector<cv::Point2f> points2d;

		for (int i = 0; i < squareFeatures.size(); i++) {
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

        cv::Mat tvec, rvec;
        cv::solvePnPRansac(points3d, points2d, intrinsic, distortion, rvec, tvec, false, 100, 8.0, 0.99);

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
		sprintf(buf, "S01L03_VGA/S01L03_VGA_%04d.png", findex++);

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

		cv::calcOpticalFlowPyrLK(previousFrame.image, image, previousPoints, currentPoints, status, errors);

		for (int i = 0; i < previousPoints.size(); i++) {
			if (status[i] != 0 /*&& errors.at<float>(i) < 4*/) { // klt ok!
				Feature f;
				f.kp_ = cv::KeyPoint(currentPoints[i].x, currentPoints[i].y, 1);
				f.p3D_ = previousFrame.squareFeatures[i].p3D_;
				squareFeatures.push_back(f);
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




        for(int i = 0; i < squareFeatures.size(); i++){

            point3D.at<double>(0) = squareFeatures[i].p3D_.x;
            point3D.at<double>(1) = squareFeatures[i].p3D_.y;
            point3D.at<double>(2) = squareFeatures[i].p3D_.z;
            point3D.at<double>(3) = 1;

            cv::gemm(projMatrix,  point3D , 1, 0, 0, result);
            cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)),4,cv::Scalar(0,0,255),-1);
            cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)), 3, cv::Scalar(255, 255, 255), -1);
            cv::circle(imcopy, cv::Point(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2)), 2, cv::Scalar(0, 0, 255), -1);



        }

        for(int i = 0; i < triangleFeatures.size(); i++){
            cv::circle(imcopy, cv::Point(triangleFeatures[i].kp_.pt.x,triangleFeatures[i].kp_.pt.y), 4,cv::Scalar(0,125,255),-1);
            cv::circle(imcopy, cv::Point(triangleFeatures[i].kp_.pt.x,triangleFeatures[i].kp_.pt.y), 3, cv::Scalar(125, 255, 255), -1);
            cv::circle(imcopy, cv::Point(triangleFeatures[i].kp_.pt.x,triangleFeatures[i].kp_.pt.y) , 2, cv::Scalar(125, 125, 255), -1);
        }

        cv::imshow("frame", imcopy);
		if (cv::waitKey(0) == 27) exit(0);
	}

};

void matchAndTriangulate(Frame& previousFrame, Frame& currentFrame, cv::Mat intrinsics, cv::Mat distortion) {
	
	static GenericMatcher matcher;
	vector<cv::DMatch> matches;

	matcher.match(currentFrame.descriptors, previousFrame.descriptors, matches, currentFrame.keypoints, previousFrame.keypoints);

	vector<cv::Point2f> previousTriangulate, currentTriangulate;
	cv::Mat	outputTriangulate;
	outputTriangulate.create(cv::Size(4, matches.size()), CV_32FC1);
	

	for (int i = 0; i < matches.size(); i++) {
        cv::Point pt1 = previousFrame.keypoints[matches[i].trainIdx].pt;
        cv::Point pt2 = currentFrame.keypoints[matches[i].queryIdx].pt;
		previousTriangulate.push_back(pt1);
		currentTriangulate.push_back(pt2);
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


int main() {

    //cvNamedWindow("frame", CV_WINDOW_NORMAL);
    //cvSetWindowProperty("frame", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	
    Frame previousFrame(40);
    Frame currentFrame(40);
    Frame keyFrame(40);

	cv::initModule_nonfree();
	
	previousFrame.loadKpFromFile("S01_2Ddata_dst_init.csv");
	previousFrame.load3DPointsFromFile("S01_3Ddata_dst_init.csv");
    previousFrame.loadIntrinsicsFromFile("intrinsics.xml");
    currentFrame.loadIntrinsicsFromFile("intrinsics.xml");

	previousFrame.readNextFrame();

	previousFrame.calcProjMatrix();
	
    previousFrame.detectAndDescribe();

	keyFrame = previousFrame;

	int counter = 1;

	while (true) {

		currentFrame.readNextFrame();

		currentFrame.updateUsingKLT(previousFrame);

		currentFrame.calcProjMatrix();
		

        bool enough_baseline = has_enough_baseline(keyFrame.intrinsic.inv()*keyFrame.projMatrix, currentFrame.intrinsic.inv()*currentFrame.projMatrix, 150);

        if (enough_baseline /*counter % 20 == 0*/) { // keyframe interval // change here from 5 to any other number
			currentFrame.detectAndDescribe();

            LOG("BEFORE: %d\t", currentFrame.squareFeatures.size());
            matchAndTriangulate(keyFrame, currentFrame, currentFrame.intrinsic, currentFrame.distortion);
            LOG("AFTER: %d\n", currentFrame.squareFeatures.size());

			keyFrame = currentFrame;
            // This is extremely important (otherwise all Frames will have a common projMatrix in the memory)
            keyFrame.projMatrix = currentFrame.projMatrix.clone();
		}
		
		previousFrame = currentFrame;
		counter++;
	}

}
