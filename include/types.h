#ifndef  _VOODOOMETRY_TYPES_
#define _VOODOOMETRY_TYPES_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <map>

namespace visual_odometry {


class Feature {

private:
    static int next_id_s_;

public:


    // TYPE ENUM
    enum {
        TRIANGLE, // Newly detected feature
        STAR, // Re-observed feature but non-triangulated yet
        SQUARE, // HAS 2D and 3D POINTS SET
        //CIRCLE, // Ready to use in the current frame
        UNKOWN
    };


    Feature() : id_(next_id_s_++), frame_id_(-1), type_(UNKOWN) {}
    Feature(const Feature& other) {
        id_ = other.id_;
        frame_id_ = other.frame_id_;
        kp_ = other.kp_;
        p3D_ = other.p3D_;
        desc_ = other.desc_;
        type_ = other.type_;
    }






    int id_, frame_id_;
    cv::KeyPoint kp_;
    cv::Point3f p3D_;
    cv::Mat desc_;

    int type_;

};




class Memory {

public:

    typedef std::map<int, Feature> FeatMap;
    typedef std::map<int, cv::Mat> ProjMap;

    FeatMap feature_memory_;
    ProjMap pose_memory_;

    // Receives other memory, makes data association
    //and returns matched features ready to use for pose estimation (memmap_out)
	//TODO mudar Memory other pra FeatMap
    void add(const Memory& other, FeatMap& featmap_out);

    // This function will triangulate
    void update(const cv::Mat& new_proj);



};


}

#endif // _VOODOOMETRY_TYPES_
