// ==============================================================================
// ISMAR 2015 Tracking Competition 
// 
// main2.cpp 
// Input: A reference image "S01L02_VGA_****.png"
// Output: A Projection Matrix "S01L02_Matrix.csv"
// 
// <Operating environment>
// The compilation is checked on the following environments:
// - Windows8.1 + OpenCV 2.4.10, OpenCV 3.0.0
//
// If you have further question, feel free to email tracking_chairs@ismar15.org.
// ==============================================================================

#include "ismar.h"

int main(int argc, char *argv[])
{
	cout << "--------------------------------" << endl;
	cout << "ISMAR 2015  Tracking Competition" << endl;
	cout << "  Scenario 01  Level 02" << endl;
	cout << "--------------------------------" << endl;

	for (int k = 0; k < M; k++)
	{
		cout << "frame number = " << setfill('0') << setw(4) << right << k << endl;

		// --------------------------------------
		// Read "S01L02_VGA_****.png" as a reference image.
		char buf[256];
        sprintf(buf, "S03L03_VGA/S03L03_VGA_%04d.png", k);
		//sprintf(buf, "S01L03_VGA/S01L03_VGA_%04d.png", k);
		Mat search_img = imread(buf);

		// When the image is not available, quite this program.
		if (search_img.empty())
		{
			cout << "Error: Can't find the search image file." << endl;
			return -1;
		}
		// --------------------------------------

		Mat pM(3, 4, CV_64FC1);

		user_function(search_img, pM);

		// Output the estimated projection matrix at the last frame.
        ofstream ofs("S0303_Matrix.csv", ios::trunc);
		for (int i = 0; i < 3; i++)
			ofs << pM.at<double>(i, 0) << "," << pM.at<double>(i, 1) << "," << pM.at<double>(i, 2) << "," << pM.at<double>(i, 3) << endl;
	

		cv::waitKey(0);
	}
	return 0;
}
