#ifndef _VOODOOMETRY_
#define _VOODOOMETRY_

#include <map>
#include <opencv2/core/core.hpp>

#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/video/video.hpp"

#include <utils.h>

#include <vector>

#include "types.h"
#include "utils.h"

#include <algorithm>

#include <cvsba/cvsba.h>

namespace vo_utils = visual_odometry::utils;

extern int SCENE;

namespace visual_odometry {

class STAM {



public:

    class Params {

    public:

        double baseline_thr;
        std::string POINTS_2D_INIT_FILE;
        std::string POINTS_3D_INIT_FILE;
        std::string INTRINSICS_FILE;
        std::string NEXT_FRAME_FMT;

    };


    STAM()  {}

    bool init(cv::Mat image);

    void process(cv::Mat image);


private:


    bool has_enough_baseline(cv::Mat pose1, cv::Mat pose2, double thr_baseline);
    cv::Mat calcProjMatrix(bool use_guess = false, cv::Mat guess_r = cv::Mat(), cv::Mat guess_t = cv::Mat());

    void loadIntrinsicsFromFile(const std::string& filename);

    void initFromFiles(cv::Mat image, const std::string& p2D_filename, const std::string& p3D_filename);

    void updateUsingKLT(cv::Mat image);

    void mapping(Frame::Ptr key_frame, Frame::Ptr current_frame);

    void matchAndTriangulate(Frame::Ptr& key_frame, Frame::Ptr& current_frame, cv::Mat intrinsics, cv::Mat distortion);

    void projectAndShow(cv::Mat projMatrix, cv::Mat image);


    Frame::Ptr previous_frame_;
    std::list<Frame::Ptr> key_frames_;

    TrackSet trackset_;
    Memory memory_;

    vo_utils::GenericMatcher matcher;

    cv::Mat intrinsics_, distortion_;

    Params params;



};

}



#endif // _VOODOOMETRY_
