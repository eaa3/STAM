/** @file utils.cpp
 *
 * @author	Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author	Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 *
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
    //if( descs_vector.size() == 0 )
    //    return;

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
} // visual odometry namespace
