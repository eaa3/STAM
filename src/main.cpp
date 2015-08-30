/** @file main.cpp
 *
 * @author	Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author	Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 *
 * @version 1.0
 *
 */


#include "VideoSource.h"
#include "STAM.h"

namespace vo = visual_odometry;

int main(int argc, char** argv){

    if( argc < 2 ){
        printf(" usage: ./stam <scene_number>\n where <scene_number> = 1|2|3\n\n");
        exit(1);
    }
    else{
        SCENE = atoi(argv[1]);

        if( SCENE > 3 || SCENE < 1 )
        {
             printf(" usage: ./stam <scene_number>\n where <scene_number> = 1|2|3\n\n");
             exit(1);
        }


    }


    VideoSource video_source;
    cv::Mat frame;
    vo::STAM STAM;



    std::string next_frame_format[] = { "S01L03_VGA/S01L03_VGA_%04d.png", "S02L03_VGA/S02L03_VGA_%04d.png", "S03L03_VGA/S03L03_VGA_%04d.png"};
    int i = 0;
    STAM.init(video_source.readNextFrame(next_frame_format[SCENE-1]));
    while( !(frame = video_source.readNextFrame(next_frame_format[SCENE-1])).empty() ){

        STAM.process(frame);


        if( SCENE > 1 && i%300 == 0 )
            STAM.optimise();

        i++;



    }

    STAM.optimise();
    STAM.dump();

    printf("BYEBYE\n");

    return 0;
}
