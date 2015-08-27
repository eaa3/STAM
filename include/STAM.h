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

namespace visual_odometry {

class STAM {



public:

    STAM() : previousFrame(new Frame(40)),
             currentFrame(new Frame(40)),
             keyFrame(new Frame(40)) {}

    bool init();

    void process(cv::Mat frame);


private:

    bool has_enough_baseline(cv::Mat pose1, cv::Mat pose2, double thr_baseline);
    void calcProjMatrix(cv::Mat guess_r = cv::Mat(), cv::Mat guess_t = cv::Mat());

    void loadIntrinsicsFromFile(const std::string& filename);

    void loadKpFromFile(const std::string& filename);

    void load3DPointsFromFile(const std::string& filename);

    void updateUsingKLT(Frame& previousFrame);





    Frame::Ptr previousFrame;
    Frame::Ptr currentFrame;
    Frame::Ptr keyFrame;

    TrackSet trackset;
    Memory memory;

    cv::Mat intrinsic, distortion;



};

}



#endif // _VOODOOMETRY_
