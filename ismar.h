// ==============================================================================
// If you have further question, feel free to email tracking_chairs@ismar15.org.
// ==============================================================================
//
// In thie “ismar.h”,
// please set the number of templates N.
// please set the number of frames L of the video sequence.
//
// ==============================================================================

// ---------------------------------------------------------------
// include OpenCV libs
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// for the MSVC
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#else
#define CV_EXT_STR ".lib"
#endif

#pragma comment (lib, "C:\\Program Files\\opencv\\build\\x86\\vc12\\lib\\opencv_core" CV_VERSION_STR CV_EXT_STR)
#pragma comment (lib, "C:\\Program Files\\opencv\\build\\x86\\vc12\\lib\\opencv_highgui" CV_VERSION_STR CV_EXT_STR)
#pragma comment (lib, "C:\\Program Files\\opencv\\build\\x86\\vc12\\lib\\opencv_imgproc" CV_VERSION_STR CV_EXT_STR)

// for using OpenCV3.0, this code is necessary.
//#pragma comment (lib, "C:\\Program Files\\opencv\\build\\x86\\vc12\\lib\\opencv_world" CV_VERSION_STR CV_EXT_STR)
// ---------------------------------------------------------------

#include <vector>
#include <string>
#include <iomanip>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

// remove warnings
#pragma warning (disable : 4018)
#pragma warning (disable : 4244)

// Set the number of templates N (default 40).
#define N 40

// Set the number of frames L of the video sequence (default 201).
#define M 201

// Competitors edit this function. 
void user_function(Mat search_img, Mat pM);
