#ifndef  _VOODOOMETRY_TYPES_
#define _VOODOOMETRY_TYPES_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <map>

namespace visual_odometry {

typedef unsigned int Identifier;

class Feature {

private:
    static int next_id_s_;

public:
    typedef std::shared_ptr<Feature> Ptr;
    typedef std::map<int, Feature::Ptr> FeatMap;
    typedef std::vector<Feature::Ptr> FeatPtrSeq;
    typedef std::vector<Feature> FeatSeq;


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




    operator cv::Mat(){
        return desc_;
    }

    operator cv::KeyPoint(){
        return kp_;
    }

    operator cv::Point3f(){
        return p3D_;
    }


    int id_, frame_id_;
    cv::KeyPoint kp_;
    cv::Point3f p3D_;
    cv::Mat desc_;

    int type_;

};

class FrameData {

private:
    static Identifier next_id_s_;
    static Identifier next_kp_id_s;

public:

    typedef std::shared_ptr<FrameData> Ptr;
    typedef std::map<Identifier, FrameData::Ptr> FrameDataMap;

    FrameData() : id_(next_id_s_++), cam_matrix_(cv::Mat(3, 4, CV_64FC1)) {

    }
    FrameData(const FrameData& other)  {
        id_ = other.id_;
        img_ = other.img_;
        desc_ = other.desc_;
        kps_ = other.kps_;
        points3D_ = other.points3D_;
        points2D_ = other.points2D_;
        cam_matrix_ = other.cam_matrix_;


    }

    FrameData(cv::Mat img, cv::Mat desc,
              const std::vector< cv::KeyPoint >& kps,
              const std::vector< cv::Point2f >& points2D,
              const std::vector< cv::Point3f >&  points3D) :
        id_(next_id_s_++), img_(img), desc_(desc), kps_(kps), cam_matrix_(cv::Mat(3, 4, CV_64FC1)),
        points2D_(points2D), points3D_(points3D)
    {


    }

    FrameData(cv::Mat img) :  id_(next_id_s_++), img_(img), cam_matrix_(cv::Mat(3, 4, CV_64FC1))
    {

    }


    void addKeyPoints(const std::vector< cv::KeyPoint >& kps_in){
        size_t old_size = kps_.size();
        size_t new_size = old_size + kps_in.size();

        kps_.resize(new_size);

        for(int i = old_size; i < new_size; i++){
            kps_[i] = kps_in[i - old_size];
        }
    }

    void describe(cv::Ptr<cv::DescriptorExtractor> descriptor_in){
        descriptor_in->compute(img_, kps_, desc_);
    }

    void describe(cv::Ptr<cv::DescriptorExtractor> descriptor_in, cv::Mat img_in){
        descriptor_in->compute(img_in, kps_, desc_);
    }


    Identifier id_;
    cv::Mat img_;
    cv::Mat desc_;
    std::vector<Identifier> ids;
    std::vector< cv::KeyPoint > kps_;
    std::vector< cv::Point2f > points2D_;
    std::vector< cv::Point3f > points3D_;

    cv::Mat cam_matrix_; // K*X where X is the camera extrinsic params and K is the camera intrinsics (pinhole model)



};




class Memory {

public:


    typedef std::map<int, cv::Mat> ProjMap;

    Feature::FeatMap feature_memory_;
    ProjMap proj_memory_;

    // Receives other memory, makes data association
    // and returns matched features ready to use for pose estimation (memmap_out)
    // TODO: mudar Memory other pra FeatMap
   // void add(const std::vector< cv::KeyPoint >& current_kps, const std::vector< cv::Mat >& current_descs, vector< Feature::Ptr >& features_out);


    // This function will triangulate
    void update(const cv::Mat& new_proj);



};


}

#endif // _VOODOOMETRY_TYPES_
