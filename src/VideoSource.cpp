/** @file VideoSource.cpp
 *
 * @author	Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author	Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 *
 * @version 1.0
 *
 */


#include <VideoSource.h>
#include <opencv2/highgui/highgui.hpp>

cv::Mat VideoSource::readNextFrame(const std::string& NEXT_FRAME_FMT) {


    static int findex = 0;
    cv::Mat image;
    char buf[256];
    sprintf(buf, NEXT_FRAME_FMT.c_str(), findex++);
    image = cv::imread(buf,-1);

    //if( findex >= 100 ) return cv::Mat();

    return image;
}
