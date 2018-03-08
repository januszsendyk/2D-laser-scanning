#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/// Matrices to store images

Mat frame;


int main(int, char**)
{
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened()) // check if we succeeded
		return -1;

	namedWindow("Film");

	while (1) {
		if (!cap.read(frame))
		cap >> frame;

		imshow("Film", frame);

		cv::waitKey(15);
	}

	waitKey(0);
	return 0;
}
