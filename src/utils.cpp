/** @file utils.cpp
 * (modified for Visual Odometry and Graph Slam by Saif Sidhik (sxs1412@student.bham.ac.uk))
 * 
 * @author  Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author  Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 * @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
 * @version 1.0
 *
 */


#include "utils.h"
#include <iostream>

namespace visual_odometry {

namespace utils {

void convertToStdVector(cv::Mat descriptors, std::vector<cv::Mat>& descs_vector_out){

    for(int i = 0; i < descriptors.rows; i++){
        cv::Mat desc = descriptors(cv::Range(i,i+1), cv::Range(0,descriptors.cols)).clone();
        descs_vector_out.push_back(desc);
    }

}

cv::Mat convertToMat(const std::vector<cv::Mat>& descs_vector){

    if( descs_vector.size() == 0 ){
        printf("WARNING!!!\n");
    }

    int cols = descs_vector[0].cols;
    cv::Mat descriptors_out;
    descriptors_out.create(descs_vector.size(), cols, descs_vector[0].type() );

    for(int i = 0; i < descs_vector.size(); i++){
        descs_vector[i].copyTo(descriptors_out(cv::Range(i,i+1), cv::Range(0,cols)));
    }

    printf("rows %d cols %d\n", descriptors_out.rows,descriptors_out.cols);

    return descriptors_out;

}

std::vector<cv::Point3f> find3DCheckerboardCorners(const std::string& p3D_filename, const int points_per_row, const int points_per_col, bool invert_flag)
{

    // Input: -- csv file containing world coordinates of 4 points: the first inner Chessboard corner (top-left) (p1), the last inner corners along the row and coloumn (p2,p3), bottom right inner corner (p4)
    //        -- number of points along the row,
    //        -- number of points along the column.
    //
    // Output:-- Vector of 3D points in the required format for STAM initialisation (checkerboard initialisation).

    // Get marker points from file
    FILE *checkerboar_marker_file = fopen(p3D_filename.c_str(), "r");
    cv::Point3f p3D;
    std::vector<cv::Point3f> marker_points;
    while (fscanf(checkerboar_marker_file, "%f,%f,%f", &p3D.x, &p3D.y, &p3D.z) == 3)
    {
        marker_points.push_back(p3D);
    }

    std::vector<cv::Point3f> ret_val = find3DCheckerboardCorners(marker_points, points_per_row, points_per_col, invert_flag);

    return ret_val;

}

std::vector<cv::Point3f> find3DCheckerboardCorners(const int points_per_row, const int points_per_col, bool invert_flag)
{
    // Input: 
    //        -- number of points along the row,
    //        -- number of points along the column.
    //
    // Output:-- Vector of 3D points in the required format for STAM initialisation (checkerboard initialisation) with the bottom right corner as origin
    // checkerboard_scale_ defines the scale of the new coordinate system. Value defines the length of one side of a square in the checkerboard.
    cv::Point3f p1, p2, p3, p4;
    std::vector<cv::Point3f> marker_points;
    p1.x = 0.0; p1.y = checkerboard_scale_*points_per_col; p1.z = 0.0; marker_points.push_back(p1);
    p2.x = checkerboard_scale_*points_per_row; p2.y = checkerboard_scale_*points_per_col; p2.z = 0.0; marker_points.push_back(p2);
    p3.x = 0.0; p3.y = 0.0; p3.z =0.0; marker_points.push_back(p3);
    p4.x = checkerboard_scale_*points_per_row; p4.y = 0.0; p4.z = 0.0; marker_points.push_back(p4);


    std::vector<cv::Point3f> ret_val = find3DCheckerboardCorners(marker_points, points_per_row, points_per_col, invert_flag);

    return ret_val;

}

std::vector<cv::Point3f> find3DCheckerboardCorners(std::vector<cv::Point3f> marker_points, const int points_per_row, const int points_per_col, bool invert_flag)
{

    assert(marker_points.size() == 4);
    cv::Point3f p1, p2, p3,p4, p1_temp, p2_temp, p3_temp, p4_temp;
    p1_temp = marker_points[3]; p2_temp = marker_points[2]; p3_temp = marker_points[1]; p4_temp = marker_points[0];

    // -------- Checking if the corners were detected from top-left to bottom-right or bottom-right to top-left
    if (invert_flag)
    {
        std::cout << "Detected corners from bottom-right to top-left." << std::endl;
        p1 = p1_temp; p2 = p2_temp; p3 = p3_temp; p4 = p4_temp;
    }
    else
    {
        std::cout << "Detected corners from top-left to bottom-right." << std::endl;
        p1 = p4_temp; p2 = p3_temp; p3 = p2_temp; p4 = p1_temp; 
    }


    // create a vector of 3d points of all inner corners row by row, left to right
    std::vector<cv::Point3f> ret_val;
    float dx_row, dy_row, dz_row, dx_col, dy_col, dz_col;
    dx_row = (p2.x - p1.x)/(points_per_row-1);
    dy_row = (p2.y - p1.y)/(points_per_row-1);
    dz_row = (p2.z - p1.z)/(points_per_row-1);
    dx_col = (p3.x - p1.x)/(points_per_col-1);
    dy_col = (p3.y - p1.y)/(points_per_col-1);
    dz_col = (p3.z - p1.z)/(points_per_col-1);
    cv::Point3f corner_point;
    for (int col_point = 0; col_point < points_per_col; ++col_point)
    {
        for (int row_point = 0; row_point < points_per_row; ++row_point)
        {
            corner_point.x = p1.x + (col_point)*dx_col + (row_point)*dx_row;
            corner_point.y = p1.y + (col_point)*dy_col + (row_point)*dy_row;
            corner_point.z = p1.z + (col_point)*dz_col + (row_point)*dz_row;

            ret_val.push_back(corner_point);
        }
    }
    std::cout << "Checkerboard 3D corner estimation error: " << float(cv::norm(ret_val[ret_val.size()-1]-p4)) << std::endl;
    assert (cv::norm(ret_val[ret_val.size()-1]-p4) < 40.00);

    return ret_val;

}

// Generic Matcher


// Match feature points using symmetry test and RANSAC
// returns fundemental matrix
cv::Mat GenericMatcher::match(const cv::Mat& descriptors1,
                              const cv::Mat& descriptors2, // input descriptors
                              // output matches and keypoints
                              std::vector<cv::DMatch>& matches,
                              std::vector<cv::KeyPoint>& keypoints1,
                              std::vector<cv::KeyPoint>& keypoints2) {

    if(descriptors1.type()!=CV_32F) {
        descriptors1.convertTo(descriptors1, CV_32F);
    }

    if(descriptors2.type()!=CV_32F) {
        descriptors2.convertTo(descriptors2, CV_32F);
    }


    //cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased"); // alternative: "BruteForce" FlannBased
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce"); // alternative: "BruteForce" FlannBased

    // from image 1 to image 2
    // based on k nearest neighbours (with k=2)
    std::vector< std::vector<cv::DMatch> > matches1;

    matcher->knnMatch(descriptors1,descriptors2,
                      matches1, // vector of matches (up to 2 per entry)
                      2);
    // return 2 nearest neighbours
    // from image 2 to image 1
    // based on k nearest neighbours (with k=2)
    std::vector< std::vector<cv::DMatch> > matches2;
    matcher->knnMatch(descriptors2,descriptors1,
                      matches2, // vector of matches (up to 2 per entry)
                      2);
    // return 2 nearest neighbours
    // 3. Remove matches for which NN ratio is
    // > than threshold
    // clean image 1 -> image 2 matches
    int removed= ratioTest(matches1);
    // clean image 2 -> image 1 matches
    removed= ratioTest(matches2);
    // 4. Remove non-symmetrical matches
    std::vector<cv::DMatch> symMatches;
    symmetryTest(matches1,matches2,symMatches);
    //symmetryTest(matches1, matches2, matches);
    // 5. Validate matches using RANSAC
    cv::Mat fundemental = ransacTest(symMatches,
                                      keypoints1, keypoints2, matches);
    // return the found fundemental matrix
    return fundemental;

}


// Match feature points using symmetry test and RANSAC
// returns fundemental matrix
cv::Mat GenericMatcher::match2(cv::Mat& image1,
                              cv::Mat& image2, // input images
                              // output matches and keypoints
                              std::vector<cv::DMatch>& matches,
                              std::vector<cv::KeyPoint>& keypoints1,
                              std::vector<cv::KeyPoint>& keypoints2) {
    // 1a. Detection of the SURF features
    if( !keypoints1.size() ) detector_->detect(image1,keypoints1);
    if( !keypoints2.size() ) detector_->detect(image2,keypoints2);

    if( keypoints1.size() <= 10 || keypoints2.size() <= 10)
        return cv::Mat();

    // 1b. Extraction of the SURF descriptors
    cv::Mat descriptors1, descriptors2;
    extractor_->compute(image1,keypoints1,descriptors1);
    extractor_->compute(image2,keypoints2,descriptors2);
    // 2. Match the two image descriptors
    // Construction of the matcher


    return match(descriptors1, descriptors2, matches, keypoints1, keypoints2);

}


// Clear matches for which NN ratio is > than threshold
// return the number of removed points
// (corresponding entries being cleared,
// i.e. size will be 0)
int GenericMatcher::ratioTest(std::vector<std::vector< cv::DMatch> >& matches) {
    int removed=0;
    // for all matches
    for (std::vector<std::vector< cv::DMatch> >::iterator
         matchIterator= matches.begin();
         matchIterator!= matches.end(); ++matchIterator) {
        // if 2 NN has been identified
        if (matchIterator->size() > 1) {
            // check distance ratio
            if ((*matchIterator)[0].distance/
                    (*matchIterator)[1].distance > ratio_) {
                matchIterator->clear(); // remove match
                removed++;
            }
        } else { // does not have 2 neighbours
            matchIterator->clear(); // remove match
            removed++;
        }
    }
    return removed;
}

// Insert symmetrical matches in symMatches vector
void GenericMatcher::symmetryTest(
        const std::vector<std::vector< cv::DMatch> >& matches1,
        const std::vector<std::vector< cv::DMatch> >& matches2,
        std::vector<cv::DMatch>& symMatches) {
    // for all matches image 1 -> image 2
    for (std::vector<std::vector< cv::DMatch> >::
         const_iterator matchIterator1= matches1.begin();
         matchIterator1!= matches1.end(); ++matchIterator1) {
        // ignore deleted matches
        if (matchIterator1->size() < 2)
            continue;
        // for all matches image 2 -> image 1
        for (std::vector<std::vector< cv::DMatch> >::
             const_iterator matchIterator2= matches2.begin();
             matchIterator2!= matches2.end();
             ++matchIterator2) {
            // ignore deleted matches
            if (matchIterator2->size() < 2)
                continue;
            // Match symmetry test
            if ((*matchIterator1)[0].queryIdx ==
                    (*matchIterator2)[0].trainIdx &&
                    (*matchIterator2)[0].queryIdx ==
                    (*matchIterator1)[0].trainIdx) {
                // add symmetrical match
                symMatches.push_back(
                            cv::DMatch((*matchIterator1)[0].queryIdx,
                            (*matchIterator1)[0].trainIdx,
                        (*matchIterator1)[0].distance));
                break; // next match in image 1 -> image 2
            }
        }
    }
}

// Identify good matches using RANSAC
// Return fundemental matrix
cv::Mat GenericMatcher::ransacTest(
        const std::vector<cv::DMatch>& matches,
        const std::vector<cv::KeyPoint>& keypoints1,
        const std::vector<cv::KeyPoint>& keypoints2,
        std::vector<cv::DMatch>& outMatches) {
    // Convert keypoints into Point2f
    std::vector<cv::Point2f> points1, points2;
    for (std::vector<cv::DMatch>::
         const_iterator it= matches.begin();
         it!= matches.end(); ++it) {
        // Get the position of left keypoints
        float x= keypoints1[it->queryIdx].pt.x;
        float y= keypoints1[it->queryIdx].pt.y;
        points1.push_back(cv::Point2f(x,y));
        // Get the position of right keypoints
        x= keypoints2[it->trainIdx].pt.x;
        y= keypoints2[it->trainIdx].pt.y;
        points2.push_back(cv::Point2f(x,y));
    }



    if( points1.size() != points2.size() || points1.size() < 8 || points2.size() < 8 ){
        std::cout << " Point list size is different or < 8, aborting fundamental matrix computation. " << std::endl;

        return cv::Mat();
    }

    //std::cout << " Computing F with NPoints: " << std::min(points1.size(), points2.size()) <<  std::endl;
    // Compute F matrix using RANSAC
    std::vector<uchar> inliers(points1.size(),0);
    cv::Mat fundemental= cv::findFundamentalMat(
                cv::Mat(points1),cv::Mat(points2), // matching points
                inliers,
                // match status (inlier or outlier)
                CV_FM_RANSAC, // RANSAC method
                distance_,
                // distance to epipolar line
                confidence_); // confidence probability
    // extract the surviving (inliers) matches
    std::vector<uchar>::const_iterator
            itIn= inliers.begin();
    std::vector<cv::DMatch>::const_iterator
            itM= matches.begin();
    // for all matches
    for ( ;itIn!= inliers.end(); ++itIn, ++itM) {
        if (*itIn) { // it is a valid match
            outMatches.push_back(*itM);
        }
    }
    if (refineF_) {
        // The F matrix will be recomputed with
        // all accepted matches
        // Convert keypoints into Point2f
        // for final F computation
        points1.clear();
        points2.clear();
        for (std::vector<cv::DMatch>::
             const_iterator it= outMatches.begin();
             it!= outMatches.end(); ++it) {
            // Get the position of left keypoints

            float x= keypoints1[it->queryIdx].pt.x;
            float y= keypoints1[it->queryIdx].pt.y;
            points1.push_back(cv::Point2f(x,y));
            // Get the position of right keypoints
            x= keypoints2[it->trainIdx].pt.x;
            y= keypoints2[it->trainIdx].pt.y;
            points2.push_back(cv::Point2f(x,y));
        }

        if( points1.size() != points2.size() || points1.size() < 8 || points2.size() < 8 ){
            std::cout << " Point list size is different or < 8, abording fundamental matrix Re-computation. " << std::endl;
            return cv::Mat();
        }

        //std::cout << " Re-computing F " << std::endl;
        // Compute 8-point F from all accepted matches
        fundemental= cv::findFundamentalMat(
                    cv::Mat(points1),cv::Mat(points2), // matches
                    CV_FM_8POINT); // 8-point method
    }
    return fundemental;
}


} // utils namespace
}
