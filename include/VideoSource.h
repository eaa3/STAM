#include <opencv2/core/core.hpp>


class VideoSource {

public:


    cv::Mat readNextFrame(const std::string& NEXT_FRAME_FMT);


};
