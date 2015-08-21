#include "types.h"


namespace visual_odometry {


/********** FEATURE ************/

// Static member
int Feature::next_id_s_ = 0;
Identifier FrameData::next_id_s_ = 0;


/********** MEMORY ************/


// Receives other memory, makes data association
//and returns matched features ready to use for pose estimation (memmap_out)
//void Memory::add(const std::vector<cv::KeyPoint>& current_kps, const std::vector<cv::Mat>& current_descs, vector<Feature::Ptr>& features_out){

    // Match features of *this and other => (output) Matches


    //Convert current memory to vector of descs




    // Update type of matched features in *this and add the new ones
    // Filter and returns Circle features (memmap_out)




//}

void Memory::update(const cv::Mat& new_proj){

    // Append new_proj to ProjMap
    // Find 3D points to all stars (if there's enough movement)
    // Update *this shiffting stars to squares


}



}
