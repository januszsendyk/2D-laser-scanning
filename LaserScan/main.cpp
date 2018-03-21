#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/// Matrices to store images

Mat frame;


/// Global Variables
const int alpha_slider_max = 255;
int alpha_slider = 50;

const int hsv_h_min_max = 255;
int hsv_h_min = 166;
const int hsv_s_min_max = 255;
int hsv_s_min = 37;
const int hsv_v_min_max = 255;
int hsv_v_min = 180;

const int hsv_h_max_max = 255;
int hsv_h_max = 255;
const int hsv_s_max_max = 255;
int hsv_s_max = 255;
const int hsv_v_max_max = 255;
int hsv_v_max = 255;

void find_first(Mat& input, Mat& output)
{
	int rows = input.rows;
	int columns = input.cols;
	Mat model = input;
	int flaga = 0;
	for (int i = 0; i < rows; i++)
	{
		flaga = 0;
		for (int j = 0; j < columns; j++)
		{
			if (model.at<uchar>(i, j) > 0&& flaga==0)
			{
				flaga = 1;
				output.at<uchar>(i, j) = 255;
			}
			else
			{
				output.at<uchar>(i, j) = 0;
			}
		}
	}
}

int main(int, char**)
{
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened()) // check if we succeeded
		return -1;

	namedWindow("Film");


	createTrackbar("hsv_h_min", "Film", &hsv_h_min, hsv_h_min_max);
	createTrackbar("hsv_s_min", "Film", &hsv_s_min, hsv_s_min_max);
	createTrackbar("hsv_v_min", "Film", &hsv_v_min, hsv_v_min_max);
	createTrackbar("hsv_h_max", "Film", &hsv_h_max, hsv_h_max_max);
	createTrackbar("hsv_s_max", "Film", &hsv_s_max, hsv_s_max_max);
	createTrackbar("hsv_v_max", "Film", &hsv_v_max, hsv_v_max_max);

	FileStorage fs2("cam_param.yml", FileStorage::READ);

	Mat cameraMatrix2, distCoeffs2;
	fs2["cameraMatrix"] >> cameraMatrix2;
	fs2["distCoeffs"] >> distCoeffs2;

	fs2.release();


	

	while (1) {
		Mat tmp;
		if (!cap.read(tmp))
		cap >> tmp;
		undistort(tmp, frame, cameraMatrix2, distCoeffs2);
		imshow("Film", frame);
		Mat hsv;
		cvtColor(frame, hsv, COLOR_BGR2HSV);

		Mat mask;

		inRange(hsv, Scalar(hsv_h_min, hsv_s_min, hsv_v_min), Scalar(hsv_h_max, hsv_s_max, hsv_v_max), mask);
		
		Mat frame_dilate;
		int erosion_size = 2;
		Mat element = getStructuringElement(MORPH_RECT, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));
		//cvtColor(mediana, mediana, COLOR_HSV2BGR);
		dilate(mask, mask, element);

		imshow("Film_mask", mask);

		Mat linia;
		mask.copyTo(linia);
		find_first(mask, linia);
		rotate(linia, linia, ROTATE_90_CLOCKWISE);
		imshow("Film_linia", linia);
		cv::waitKey(15);
	}

	waitKey(0);
	return 0;
}
