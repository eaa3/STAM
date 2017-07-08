/** @file STAM.h
 *
 * @author	Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author	Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 *
 * @version 1.0
 *
 */

#ifndef _VOODOOMETRY_
#define _VOODOOMETRY_

#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <vector>
#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/video/video.hpp"

#include <utils.h>

#include <list>

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
        std::string TEMPL_FILE_FMT;

    };


    STAM()  {}

    bool init(cv::Mat image);

    Frame::Ptr process(cv::Mat image, bool visualize_flag);

    void optimise();
    void dump();
    std::list<Frame::Ptr> key_frames_;

    cv::Mat intrinsics_;
private:


    bool has_enough_baseline(cv::Mat pose1, cv::Mat pose2, double thr_baseline);
    cv::Mat calcProjMatrix(bool use_guess, cv::Mat& guess_r, cv::Mat& guess_t);

    void loadIntrinsicsFromFile(const std::string& filename);

    void initFromFiles(cv::Mat image, const std::string& p2D_filename, const std::string& p3D_filename);

    void initFromTemplates(cv::Mat image, const std::string& p3D_filename, const std::string& template_format);

    void updateUsingKLT(cv::Mat image);

    void mapping(Frame::Ptr key_frame, Frame::Ptr current_frame);

    bool matchAndTriangulate(Frame::Ptr& key_frame, Frame::Ptr& current_frame, cv::Mat intrinsics, cv::Mat distortion);

    void projectAndShow(cv::Mat projMatrix, cv::Mat image);

    void updateOptimisedKF();

    Frame::Ptr previous_frame_;
    //std::list<Frame::Ptr> key_frames_;

    TrackSet trackset_;
    Memory memory_;

    vo_utils::GenericMatcher matcher;

    cv::Mat distortion_;

    Params params;



};

} // namespace visual_odometry



#endif // _VOODOOMETRY_
