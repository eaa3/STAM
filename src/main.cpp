#include "VideoSource.h"
#include "STAM.h"

namespace vo = visual_odometry;

int main(int argc, char** argv){


    VideoSource video_source;
    cv::Mat frame;
    vo::STAM STAM;



    std::string next_frame_format[] = { "S01L03_VGA/S01L03_VGA_%04d.png", "S02L03_VGA/S02L03_VGA_%04d.png", "S03L03_VGA/S03L03_VGA_%04d.png"};

    STAM.init(video_source.readNextFrame(next_frame_format[0]));
    while( !(frame = video_source.readNextFrame(next_frame_format[0])).empty() ){

        STAM.process(frame);

    }

    printf("BYEBYE\n");

    return 0;
}
