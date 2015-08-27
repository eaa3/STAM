#include <VideoSource.h>
#include <opencv2/highgui/highgui.hpp>

cv::Mat VideoSource::readNextFrame(const std::string& NEXT_FRAME_FMT) {


    static int findex = 0;
    cv::Mat image;
    char buf[256];
    sprintf(buf, NEXT_FRAME_FMT.c_str(), findex++);
    image = cv::imread(buf);
    if (image.empty()) {
        exit(0);
    }

    return image;
}
