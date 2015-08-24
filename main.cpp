#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <utils.h>

#define M 482

using namespace cv;
using namespace std;
using namespace visual_odometry::utils;

namespace vo = visual_odometry;


void loadKpFromFile(const std::string& filename, vo::FrameData& frame_out){

    FILE *f = fopen(filename.c_str(), "r");
    float x, y;

    while( fscanf(f,"%f,%f",&x, &y) == 2 ){

        frame_out.points2D_.push_back(cv::Point2f(x,y));
        frame_out.kps_.push_back(cv::KeyPoint(x,y,1, -1, 5000));



    }

    fclose(f);

}

void load3DPointsFromFile(const std::string& filename, vo::FrameData& frame_out){

    FILE *f = fopen(filename.c_str(), "r");
    cv::Point3f p;

    while( fscanf(f,"%f,%f,%f",&p.x, &p.y, &p.z) == 3 ){
        frame_out.points3D_.push_back(p);


    }

    fclose(f);

}


int main() {
    initModule_nonfree();
    char buf[256];
    Mat search_img;

    char algorithm[] = "SIFT";
    Ptr<FeatureDetector> detector = FeatureDetector::create(algorithm);
    Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(algorithm);

    GenericMatcher matcher2;

    vector<KeyPoint> detected_keypoints;

    vo::FrameData::Ptr curr_frame_data;


    vector<KeyPoint> keypoints1;
    vector<KeyPoint> keypoints2;
    Mat descriptors1;
    Mat descriptors2;
    vector<DMatch> matches;

    //  Lê frame atual (frame 0)

    sprintf(buf, "S01L03_VGA/S01L03_VGA_%04d.png", 0);
    search_img = imread(buf);
    curr_frame_data = vo::FrameData::Ptr(new vo::FrameData(search_img));




    //     *
    //     *      Extrai manualmente os keypoints e descritores do centróide dos patches dados (já se tem a posição 2D do centro de cada um deles) -> Features do tipo quadrado
    //     *


    loadKpFromFile("S01_2Ddata_dst_init.csv", *curr_frame_data);
    load3DPointsFromFile("S01_3Ddata_dst_init.csv", *curr_frame_data);

    curr_frame_data->describe(descriptor);

    printf("%d x %d \n", curr_frame_data->desc_.rows, curr_frame_data->desc_.cols);



    //            extrair as features que serão usadas para calcular a pose (usar apenas as features quadrado que estão visíveis)

    //            calcular a pose do frame atual
    printf("Computing PMatrix %d %d\n", curr_frame_data->points3D_.size(), curr_frame_data->points2D_.size());
    CalcProjectionMatrix(curr_frame_data->points3D_, curr_frame_data->points2D_, curr_frame_data->cam_matrix_);
    printf("Computing PMatrix: SUCCESS\n");

    //     *       Calcula os keypoints e descritores automáticos -> features do tipo triangulo
    //     *
     detected_keypoints.clear();
     detector->detect(curr_frame_data->img_, detected_keypoints);
	 for (int i = 0; i < detected_keypoints.size(); i++) {
		 printf("%f\n", detected_keypoints[i].size);
	 }
	 	 
	 printf("Detected keypoints\n");
     curr_frame_data->addKeyPoints(detected_keypoints);
     curr_frame_data->describe(descriptor); // Updating description


     keypoints2 = curr_frame_data->kps_;
     descriptors2 = curr_frame_data->desc_.clone();

    //     *       coloca as features no frame atual
    //     *
    //     *       coloca as features do frame atual para o conjunto cumulativo
    //     *





    printf("Initialised!\n");
    for (int k = 1; k < M; k++)
    {
        char buf[256];
        // --------------------------------------
        // Read "S01L02_VGA_****.png" as a reference image.
        sprintf(buf, "S01L03_VGA/S01L03_VGA_%04d.png", k);
        printf("reading image\n");
        //sprintf(buf, "S01L03_VGA/S01L03_VGA_%04d.png", k);
        search_img = imread(buf);
        printf(" image read\n");

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
        matcher2.match(descriptors1, descriptors2, matches, keypoints1, keypoints2);

        /*for (int i = 0; i < keypoints2.size(); i++) {
            circle(search_img, keypoints2[i].pt, 2, Scalar(0, 0, 255), -1);
        }*/

        //int counter = 0;

        for (int i = 0; i < matches.size(); i++) {
            //printf("%f\n", matches[i].distance);
            CvPoint pt2 = keypoints2[matches[i].trainIdx].pt;
            CvPoint pt1 = keypoints1[matches[i].queryIdx].pt;

            float distance = sqrtf((pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
            if (distance < 20) {
                circle(search_img, keypoints2[matches[i].queryIdx].pt, 2, Scalar(0, 255, 0), -1);
                //counter++;
                line(search_img, pt2, pt1, Scalar(0, 0, 0));
            }


        }

        printf("Show image\n");
        imshow("frame", search_img);
        cv::waitKey(0);

    }

}
