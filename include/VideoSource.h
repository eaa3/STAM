/** @file VideoSource.h
 *
 * @author	Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author	Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 *
 * @version 1.0
 *
 */

#include <opencv2/core/core.hpp>


class VideoSource {

public:


    cv::Mat readNextFrame(const std::string& NEXT_FRAME_FMT);


};
