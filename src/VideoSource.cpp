/** @file VideoSource.cpp
 * (modified for Visual Odometry and Graph Slam by Saif Sidhik (sxs1412@student.bham.ac.uk))
 * 
 * @author  Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author  Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 * @author  Saif Sidhik (sxs1412@student.bham.ac.uk)
 * @version 1.0
 *
 */


#include <VideoSource.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

cv::Mat VideoSource::readNextFrame(const std::string& NEXT_FRAME_FMT) {


    static int findex = 0;
    cv::Mat image;
    char buf[256];
    sprintf(buf, NEXT_FRAME_FMT.c_str(), findex);
    image = cv::imread(buf,-1);
    findex+=1;
    // std::cout << "index " << findex-1 << std::endl;
    // std::cout << buf << std::endl;
    //if( findex >= 100 ) return cv::Mat();

    return image;
}
