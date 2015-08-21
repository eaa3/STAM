// ==============================================================================
// If you have further question, feel free to email tracking_chairs@ismar15.org.
// ==============================================================================
// 
// The user_function() implements template matching.
// Patch images stored in "tmp" folder are named as "S01L**_VGA_patch_****.png".
// Input: 3D coordinates of evaluation points "S01L**_3Ddata.csv"
// Output: 2D coordinate of observation points "S01L**_2Ddata_dst.csv"
//         //3D coordinates of evaluation points "S01L**_3Ddata_dst.csv"
// 
// ==============================================================================

#include "ismar.h"
#include "utils.h"

// ---------------------------------------------------------------
// Read .csv file as a numerical value.
void Read3DData(ifstream &ifs, int pt[][3])
{
	int p;
	string str;
	vector<string> str_vector;

	// Read a line of the input file one by one.
	// Separate the line at every “,”.
	// Store the separated characters to an array.
	for (int i = 0; i < N; i++)
	{
		while (getline(ifs, str))
		{
			while ((p = str.find(",")) != str.npos)
			{
				str_vector.push_back(str.substr(0, p));
				str = str.substr(p + 1);
			}
			str_vector.push_back(str);
		}
	}

	// Convert the stored character to the numerical value.
	vector<float> f_vector(str_vector.size());
	transform(str_vector.begin(), str_vector.end(), f_vector.begin(), [](const string& val){return stod(val); });

	// Store 3D coordinate values.
	int k = 0;
	for (int j = 0; j < N * 3; j += 3)
	{
		pt[k][0] = f_vector[j];
		pt[k][1] = f_vector[j + 1];
		pt[k][2] = f_vector[j + 2];
		k++;
	}
}
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// Estimate a projection matrix from 2D-3D correspondences using DLT algorithm.
void CalcProjectionMatrix(const vector<Point3f>& pt_3d, const vector<Point2f>& pt_2d, Mat pM)
{
	// When can't find 3D-data or 2D-data, quite this program.
	if (pt_3d.empty() || pt_2d.empty())
	{
		cout << " Error: Can't find the coordinate data." << endl;
		exit(1);
	}

	else
	{
		Mat A;
		A.create(Size(12, pt_3d.size() * 2), CV_64FC1);

		// Generate a matrix from 2D-3D correspondences.
		for (int i = 0, j = 0; i < pt_3d.size() * 2; i += 2, ++j)
		{

			A.at<double>(i, 0) = 0.0;
			A.at<double>(i, 1) = 0.0;
			A.at<double>(i, 2) = 0.0;
			A.at<double>(i, 3) = 0.0;

			A.at<double>(i, 4) = -pt_3d[j].x;
			A.at<double>(i, 5) = -pt_3d[j].y;
			A.at<double>(i, 6) = -pt_3d[j].z;
			A.at<double>(i, 7) = -1.0;

			A.at<double>(i, 8) = pt_2d[j].y * pt_3d[j].x;
			A.at<double>(i, 9) = pt_2d[j].y * pt_3d[j].y;
			A.at<double>(i, 10) = pt_2d[j].y * pt_3d[j].z;
			A.at<double>(i, 11) = pt_2d[j].y;

			A.at<double>(i + 1, 0) = pt_3d[j].x;
			A.at<double>(i + 1, 1) = pt_3d[j].y;
			A.at<double>(i + 1, 2) = pt_3d[j].z;
			A.at<double>(i + 1, 3) = 1.0;

			A.at<double>(i + 1, 4) = 0.0;
			A.at<double>(i + 1, 5) = 0.0;
			A.at<double>(i + 1, 6) = 0.0;
			A.at<double>(i + 1, 7) = 0.0;

			A.at<double>(i + 1, 8) = -pt_2d[j].x * pt_3d[j].x;
			A.at<double>(i + 1, 9) = -pt_2d[j].x * pt_3d[j].y;
			A.at<double>(i + 1, 10) = -pt_2d[j].x * pt_3d[j].z;
			A.at<double>(i + 1, 11) = -pt_2d[j].x;
		}

		Mat pvect;

		// Estimate the singular value and vector by singular value decomposition (SVD).
		SVD::solveZ(A, pvect);

		// Generate the projection matrix.
		for (int i = 0; i < 12; i++)
			pM.at<double>(i / 4, i % 4) = pvect.at<double>(i);
	}
}
// ---------------------------------------------------------------

// ===============================================================
void user_function(Mat search_img, Mat pM)
{
	// An array of map of comparison results.
	Mat result_img;

	// An array to store 3D coordinates.
	vector<Point3f> object_pt;

	// Store the 2D coordinates estimated by user_function.
	vector<Point2f> image_pt;

	// --------------------------------------
	ifstream ifs("S01_3Ddata.csv");

	// When no 3D-data is available, quite this program.
	if (!ifs)
	{
		cout << "Error: Can't find the coordinate file." << endl;
		exit(1);
	}

    int pt[N][3];
    Read3DData(ifs, pt);
	// --------------------------------------
	
	// Output the 2D and 3D coordinate at the last frame.
	ofstream pt_3d("S01_3Ddata_dst.csv", ios::trunc);
	ofstream pt_2d("S01_2Ddata_dst.csv", ios::trunc);

	for (int i = 0; i < N; i++)
	{
		// --------------------------------------
		// Read template images.
        char buf[256];
        sprintf(buf, "S01L02_patch/S01L02_VGA_patch_%04d.png", i);
		//sprintf(buf, "S01L03_patch/S01L03_VGA_patch_%04d.png", i);
        Mat tmp_img = imread(buf);

		// When no template image is available, quite this program.
		if (tmp_img.empty())
		{
			cout << "Error: Can't find patch image file." << endl;
			exit(1);
		}
		// --------------------------------------

		// Template matching against "search image".
		matchTemplate(search_img, tmp_img, result_img, CV_TM_CCOEFF_NORMED);

		// Variables for "the maximum values of matching score" and "the point locations".
		double maxVal;
		Point maxLoc;

		// Find the global maximum in "result image".
		minMaxLoc(result_img, NULL, &maxVal, NULL, &maxLoc);


		// Store and Output the coordinate of the "best match region".
        if (maxVal > 0.9)
		{
			object_pt.push_back(Point3f(pt[i][0], pt[i][1], pt[i][2]));
			pt_3d << object_pt.back().x << "," << object_pt.back().y << "," << object_pt.back().z << endl;

			image_pt.push_back(Point2f((maxLoc.x + (tmp_img.cols) / 2), (maxLoc.y + (tmp_img.rows) / 2)));
			pt_2d << image_pt.back().x << "," << image_pt.back().y << endl;

            circle(search_img, image_pt.back(), 2, Scalar(64,255,255));

            //putText(search_img, string(buf).substr(14,29), image_pt.back(), FONT_HERSHEY_DUPLEX, 0.25, Scalar(64,255,255));


            //imshow("search img", search_img);
            //imshow("result img", result_img);
            //waitKey(0);

		}
	}

    imshow("search img", search_img);
    //imshow("result img", result_img);
    waitKey(1);

	CalcProjectionMatrix(object_pt, image_pt, pM);
}
// ===============================================================
