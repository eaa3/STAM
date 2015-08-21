#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "lbp.hpp"

#include <utils.h>

#define M 482

using namespace cv;
using namespace std;
using namespace visual_odometry::utils;

string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

int main() {

	cvNamedWindow("frame", CV_WINDOW_NORMAL);
	cvSetWindowProperty("frame", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	/*cvNamedWindow("lbp", CV_WINDOW_NORMAL);
	cvSetWindowProperty("lbp", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);*/

	initModule_nonfree();

	char algorithm[] = "SIFT";
	Ptr<FeatureDetector> detector = FeatureDetector::create(algorithm);
	Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(algorithm);
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");

	GenericMatcher matcher2;
		
	vector<KeyPoint> keypoints1;
	vector<KeyPoint> keypoints2;
	Mat descriptors1;
	Mat descriptors2;
	vector<DMatch> matches;
	//Mat grayscale;
	//Mat lbp;

	char buf[256];
	sprintf(buf, "S01L03_VGA/S01L03_VGA_%04d.png", 0);
	Mat search_img = imread(buf);
	//GaussianBlur(search_img, search_img, Size(3, 3), 0, 0);
	//cvtColor(search_img, grayscale, CV_BGR2GRAY);
	//lbp::OLBP(grayscale, lbp);
	//lbp::ELBP(grayscale, lbp, 1, 8);
	//lbp::VARLBP(grayscale, lbp, 1, 8);

	detector->detect(search_img, keypoints2);

	for (int i = 0; i < keypoints2.size(); i++) {
		printf("%f %f\n", keypoints2[i].pt.x, keypoints2[i].pt.y);
		break;
	}

	KeyPoint key0;
	key0.pt = keypoints2[0].pt;

	vector<KeyPoint> manual_keypoints;
	manual_keypoints.push_back(key0);

	Mat manual_descriptor;
	manual_descriptor.create(Size(128, 1), CV_32FC1);

	descriptor->compute(search_img, manual_keypoints, manual_descriptor);
	printf("%s\n", type2str(manual_descriptor.type()).c_str());

	descriptor->compute(search_img, keypoints2, descriptors2);

	for (int i = 0; i < manual_descriptor.cols; i++) {
		manual_descriptor.at<float>(0,i);
		descriptors2.at<float>(0, i);
		//printf("%f %f\n", manual_descriptor.at<float>(i), descriptors2.at<float>(0, i));
	}
	return 0;

	for (int k = 1; k < M; k++)
	{
		// --------------------------------------
		// Read "S01L02_VGA_****.png" as a reference image.
		sprintf(buf, "S01L03_VGA/S01L03_VGA_%04d.png", k);
		//sprintf(buf, "S01L03_VGA/S01L03_VGA_%04d.png", k);
		search_img = imread(buf);
		//GaussianBlur(search_img, search_img, Size(3, 3), 0, 0);
		//cvtColor(search_img, grayscale, CV_BGR2GRAY);
		//lbp::OLBP(grayscale, lbp);
		//lbp::ELBP(grayscale, lbp, 3, 24);
		//lbp::VARLBP(grayscale, lbp, 1, 8);

		// When the image is not available, quite this program.
		if (search_img.empty())
		{

			return -1;
		}

		keypoints1 = keypoints2;
		keypoints2.clear();
		detector->detect(search_img, keypoints2);
		/*for (int i = 0; i < keypoints2.size(); i++) {
			circle(search_img, keypoints[i].pt, 2, Scalar(255, 255, 255), -1);
		}*/

		descriptors1 = descriptors2;
		descriptor->compute(search_img, keypoints2, descriptors2);

		//matcher->match(descriptors2, descriptors1, matches);
		//printf("%d\n", matches.size());
		
		matches.clear();
		matcher2.match(descriptors2, descriptors1, matches, keypoints2, keypoints1);

		/*for (int i = 0; i < keypoints2.size(); i++) {
			circle(search_img, keypoints2[i].pt, 2, Scalar(0, 0, 255), -1);
		}*/

		//int counter = 0;
		
		for (int i = 0; i < matches.size(); i++) {
			//printf("%f\n", matches[i].distance);
			CvPoint pt2 = keypoints2[matches[i].queryIdx].pt;
			CvPoint pt1 = keypoints1[matches[i].trainIdx].pt;
			
			float distance = sqrtf((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
			if (distance < 20) {
				circle(search_img, keypoints2[matches[i].queryIdx].pt, 2, Scalar(0, 255, 0), -1);
				//counter++;
				line(search_img, pt2, pt1, Scalar(0, 0, 0));
			}
			//else {
				//line(search_img, pt2, pt1, Scalar(0, 0, 255));
			//}
			
		}
		//std::cout << '\r';
		//printf("%3d %3.2f\n", k, 100 * (float)counter/matches.size());

		// --------------------------------------

		//for (int i = 0; i < search_img.rows; i++) {
		//	for (int j = 0; j < search_img.cols; j++) {
		//		Vec3b pixel = search_img.at<Vec3b>(i, j);
		//		if (pixel.val[1] > pixel.val[2] && pixel.val[1] > pixel.val[0]) {
		//			// qualidade bom			
		//		}
		//		else {
		//			pixel.val[0] = 255;
		//			pixel.val[1] = 255;
		//			pixel.val[2] = 255;
		//		}
		//		search_img.at<Vec3b>(i, j) = pixel;
		//	}
		//}



		/*for (int i = 0; i < search_img.rows; i++) {
		for (int j = 0; j < search_img.cols; j++) {
		Vec3b pixel = search_img.at<Vec3b>(i, j);
		pixel.val[0] = pixel.val[0] & 0x0080;
		pixel.val[1] = pixel.val[1] & 0x0080;
		pixel.val[2] = pixel.val[2] & 0x0080;
		search_img.at<Vec3b>(i, j) = pixel;
		}
		}*/

		//for (int i = 0; i < search_img.rows; i++) {
		//	for (int j = 0; j < search_img.cols; j++) {
		//		Vec3b pixel = search_img.at<Vec3b>(i, j);
		//		if (pixel.val[1] == pixel.val[2] && pixel.val[1] == pixel.val[0]) {
		//			pixel.val[0] = 255;
		//			pixel.val[1] = 255;
		//			pixel.val[2] = 255;
		//		}
		//		/*else {
		//			
		//		}*/
		//		search_img.at<Vec3b>(i, j) = pixel;
		//	}
		//}


		imshow("frame", search_img);
		//imshow("lbp", lbp);
		int c = cvWaitKey(0);
		//printf("%d\n", c);

		if (c == 2424832) k -= 2;

		//Mat pM(3, 4, CV_64FC1);

		//user_function(search_img, pM);

		// Output the estimated projection matrix at the last frame.
		//ofstream ofs("S0102_Matrix.csv", ios::trunc);
		//for (int i = 0; i < 3; i++)
		//	ofs << pM.at<double>(i, 0) << "," << pM.at<double>(i, 1) << "," << pM.at<double>(i, 2) << "," << pM.at<double>(i, 3) << endl;
	}

}
