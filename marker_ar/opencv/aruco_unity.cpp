#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <filesystem>

#include <opencv2/highgui.hpp>
#include "Windows.h"
#include "Wincon.h"

using namespace std;
using namespace cv;

extern "C"
{
	Ptr<aruco::Dictionary> dict;

	int img_width, img_height;
	float marker_size;

	Mat* camera_matrix;
	Mat* dist_coeffs;

	vector<int>* ids;
	vector<vector<Point2f>>* corners;
	vector<float>* corners_flat;

	//Rotation and translation vectors from pose estimation
	vector<Vec3d>* tvecs;
	vector<double>* tvecs_flat;
	vector<Vec3d>* rvecs;
	vector<double>* rvecs_flat;

	static bool readCameraParameters(string filename, Mat& camMatrix, Mat& distCoeffs) {
		FileStorage fs(filename, FileStorage::READ);
		if (!fs.isOpened())
			return false;
		fs["camera_matrix"] >> camMatrix;
		fs["distortion_coefficients"] >> distCoeffs;
		fs["image_width"] >> img_width;
		fs["image_height"] >> img_height;
		return true;
	}

	__declspec(dllexport) int init(int dict_id, float marker_size_ = 0.0384848) {
		marker_size = marker_size_;
		string filename = "calib.txt";

		dict = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dict_id));

		ids = new vector<int>();
		corners = new vector<vector<Point2f>>();
		corners_flat = new vector<float>();

		tvecs = new vector<Vec3d>();
		rvecs = new vector<Vec3d>();
		tvecs_flat = new vector<double>();
		rvecs_flat = new vector<double>();

		camera_matrix = new Mat();
		dist_coeffs = new Mat();

		bool readOk = readCameraParameters(filename, *camera_matrix, *dist_coeffs);
		if (!readOk) {
			cerr << "Invalid camera file" << endl;
			return 0;
		}

		// Uncomment to create a console for stdout which is useful for debugging when called from unity
		//AllocConsole();
		//freopen("CONOUT$", "w", stdout);

		return 1;
	}

	__declspec(dllexport) int detect_markers(unsigned char* _unity_img, int* _out_ids_len, int** _out_ids, float** _out_corners, double** _out_rvecs, double** _out_tvecs) {
		Mat gray;

		// for calling using unity. Unity sends flipped RGBA image
		Mat img = Mat(img_height, img_width, CV_8UC4, _unity_img, img_width * 4);
		cvtColor(img, gray, COLOR_RGBA2GRAY);
		flip(gray, gray, 0);

		// if directly calling using main uncomment below block
		//Mat img = Mat(img_height, img_width, CV_8UC3, _unity_img);
		//cvtColor(img, gray, COLOR_BGR2GRAY);
		
		//cout << "Gray size: " << gray.size() << endl;

		imwrite("rgb_aruco.png", img);
		imwrite("gray_aruco.png", gray);

		aruco::detectMarkers(gray, dict, *corners, *ids);

		//write address of elem 0 in ids vector to out_ids (*out_ids = address, since out_ids is just our copy of the c# variable address), write array length to *out_ids_len.
		//Also, ids has to be allocated non-local, otherwise it goes out of scope after this function ends..
		int marker_count = ids->size();
		//cout << " Marker count: " << marker_count << endl;
		*_out_ids_len = marker_count;
		*_out_ids = NULL;
		*_out_corners = NULL;
		if (*_out_ids_len > 0) {
			aruco::estimatePoseSingleMarkers(*corners, marker_size, *camera_matrix, *dist_coeffs, *rvecs, *tvecs);

			corners_flat->resize(marker_count * 8); //For each marker, we have 4 corner points, each of which are 2 floats. So we need markers * 8 floats overall;
			rvecs_flat->resize(marker_count * 3); // 1 rvec per marker, 3 doubles per Vec3d
			tvecs_flat->resize(marker_count * 3); // Same as for rvecs
			for (int i = 0; i < marker_count; i++) {
				//corners has an array of 4 `Point2f`s for each marker. Since they're continuous, and each Point2f is just 2 floats, we can copy 4 points into 8 floats in our flat array
				memcpy(corners_flat->data() + (i * 8), (*corners)[i].data(), 4 * sizeof(Point2f));

				//Copy over rvec and tvec doubles into corresponding flat arrays
				for (int j = 0; j < 3; j++) {
					(*rvecs_flat)[i * 3 + j] = (*rvecs)[i][j];
					(*tvecs_flat)[i * 3 + j] = (*tvecs)[i][j];
				}

			}

			*_out_ids = ids->data();
			*_out_corners = corners_flat->data();
			*_out_rvecs = rvecs_flat->data();
			*_out_tvecs = tvecs_flat->data();
		}

		int result = ids->size();

		return result;
	}
}

int main() {
	cout << init(10) << endl;

	VideoCapture inputVideo;
	int waitTime = 10;
	inputVideo.open(0);

	while (inputVideo.grab()) {
		Mat image, imageCopy;
		inputVideo.retrieve(image);
		image.copyTo(imageCopy);
		unsigned char* _unity_img = image.data;
		int _out_ids_len;
		int  *_out_ids;
		float* _out_corners;
		double* _out_rvecs;
		double* _out_tvecs;
		detect_markers(_unity_img, &_out_ids_len, &_out_ids, &_out_corners, &_out_rvecs, &_out_tvecs);

		if (_out_ids_len > 0) {
			aruco::drawDetectedMarkers(imageCopy, *corners, *ids);

			for (unsigned int i = 0; i < (*ids).size(); i++)
				aruco::drawAxis(imageCopy, *camera_matrix, *dist_coeffs, (*rvecs)[i], (*tvecs)[i], marker_size * 0.5f);
		}

		imshow("out", imageCopy);
		char key = (char)waitKey(waitTime);
		if (key == 27) break;
	}
	return 0;
}