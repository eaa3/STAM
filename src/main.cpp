/** @file main.cpp
 *
 * @author  Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author  Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 *
 * @version 1.0
 *
 */


#include "VideoSource.h"
#include "STAM.h"
#include <fstream>

namespace vo = visual_odometry;
bool visualize_flag;

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
        if ( argc == 3 )
        {
            int flag = atoi(argv[2]);
            visualize_flag = (flag == 1); 
        }
        else visualize_flag = false;


    }


    VideoSource video_source;
    cv::Mat frame;
    vo::STAM vOdom;
    std::stringstream traj_name;
    traj_name << "trajectory_scene" << argv[1] << ".txt";
    std::ofstream traj_out(traj_name.str());

    std::string path_prefix[] = { "S01_INPUT" , "S02_INPUT", "S03_INPUT"};
    std::string next_frame_format[] = { "S01_INPUT/S01L03_VGA/S01L03_VGA_%04d.png", "S02_INPUT/S02L03_VGA/S02L03_VGA_%04d.png", "S03_INPUT/S03L03_VGA/S03L03_VGA_%04d.png"};
    int i = 0;
    vOdom.init(video_source.readNextFrame(next_frame_format[SCENE-1]));

    visual_odometry::Frame::Ptr current_frame;
    while( !(frame = video_source.readNextFrame(next_frame_format[SCENE-1])).empty() ){

        current_frame = vOdom.process(frame,visualize_flag);


        if( SCENE > 1 && i%300 == 0 )
            vOdom.optimise();

        i++;
        cv::Mat p;

        cv::Mat pM = vOdom.intrinsics_*current_frame->projMatrix;//.mul(1.0/274759.971);

        for (int j = 0; j < 3; j++)
            traj_out << pM.at<double>(j, 0) << "," << pM.at<double>(j, 1) << "," << pM.at<double>(j, 2) << "," << pM.at<double>(j, 3) << std::endl;

        double q1,q2,q3,q4;
        std::cout << current_frame->pose << std::endl;
        std::cout << current_frame->pose.colRange(0,3) << std::endl << std::endl << " this  \n";
        std::cout << current_frame->pose.at<double>(0,3) << '\n' << current_frame->pose.at<double>(2,3) << std::endl;
        current_frame->getQuaternion(q1,q2,q3,q4);
        std::cout << q1 << " " << q2 << " " << q3 << " " << q4 << std::endl;
        // std::cout << current_frame->r << std::endl;
        // std::cout << current_frame->pose.at<double>(0,3) << std::endl;

    }

    vOdom.optimise();
    vOdom.dump();


    traj_out.close();

    printf("EXITING\n");

    return 0;
}
