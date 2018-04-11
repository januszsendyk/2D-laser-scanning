#include <opencv2/opencv.hpp>
#include <Eigen/Core>
using namespace cv;
using namespace std;

/// Matrices to store images

Mat frame;


/// Global Variables
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vectorVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vectorVector3d;
typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > vectorVector4d;

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
		Mat frame;
		if (!cap.read(frame))
		cap >> frame;
		//undistort(tmp, frame, cameraMatrix2, distCoeffs2);
		imshow("Film", frame);


		Size patternsize(7, 6);
		vector<Point2f> corners;

		bool patternfound = findChessboardCorners(frame, patternsize, corners,
			CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
			+ CALIB_CB_FAST_CHECK);
		if (patternfound)
		{
			Mat subpixel_BW;
			cvtColor(frame, subpixel_BW, CV_BGR2GRAY);
			cornerSubPix(subpixel_BW, corners, Size(11, 11), Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			//if (i == 0)
			//cout << corners;
			//wek.push_back(corners);
			//dobra_wek.push_back(dobra);

			//namedWindow("Chessboard");
			drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
			//imshow("Chessboard", frame);
		
			
			vector<Point3f> axis;
			axis.push_back(cv::Point3d(0, 0, 0));
			axis.push_back(cv::Point3d(1, 0, 0));
			axis.push_back(cv::Point3d(0, 1, 0));
			axis.push_back(cv::Point3d(0, 0, -1));

			vector<Point3f> objp;
			for (int y = patternsize.height -1; y >= 0 ; y--)
			{
				for (int x = patternsize.width-1; x >= 0; x--)
				{
					objp.push_back(cv::Point3d(x, y, 0));
				}
			}
			Mat rvec(3, 1, DataType<double>::type);
			Mat tvec(3, 1, DataType<double>::type);
			//vector<Point3f> rvec;
			//vector<Point3f> tvec;
			vector<Point2f> imgpts;
			//# Find the rotation and translation vectors.
			solvePnP(objp, corners, cameraMatrix2, distCoeffs2, rvec, tvec);

			Mat rotMat;
			Rodrigues(rvec, rotMat);

			Mat Transf(4,4,CV_64F);
			Transf.at<double>(0, 0) = rotMat.at<double>(0, 0);
			Transf.at<double>(1, 0) = rotMat.at<double>(1, 0);
			Transf.at<double>(2, 0) = rotMat.at<double>(2, 0);
			Transf.at<double>(0, 1) = rotMat.at<double>(0, 1);
			Transf.at<double>(1, 1) = rotMat.at<double>(1, 1);
			Transf.at<double>(2, 1) = rotMat.at<double>(2, 1);
			Transf.at<double>(0, 2) = rotMat.at<double>(0, 2);
			Transf.at<double>(1, 2) = rotMat.at<double>(1, 2);
			Transf.at<double>(2, 2) = rotMat.at<double>(2, 2);

			Transf.at<double>(0, 3) = tvec.at<double>(0);
			Transf.at<double>(1, 3) = tvec.at<double>(1);
			Transf.at<double>(2, 3) = tvec.at<double>(2);

			Transf.at<double>(3, 0) = 0;
			Transf.at<double>(3, 1) = 0;
			Transf.at<double>(3, 2) = 0;
			Transf.at<double>(3, 3) = 1;

			cout<<Transf<<endl;


			Eigen::Vector4d PlaneAtCamera;
			PlaneAtCamera << 0, 0, 1, 0;

			//ret, rvecs, tvecs, inliers = cv2.solvePnP(objp, corners2, mtx, dist)
			//# project 3D points to image plane
			projectPoints(axis, rvec, tvec, cameraMatrix2, distCoeffs2, imgpts);
			//imgpts, jac = cv2.projectPoints(axis, rvec, tvecs, mtx, dist)
			//img = draw(img, corners2, imgpts)

			int myradius = 5;
			//for (int i = 0; i<imgpts.size(); i++)

				circle(frame, cvPoint(imgpts[0].x, imgpts[0].y), myradius, CV_RGB(255, 0, 255), -1, 8, 0);
				circle(frame, cvPoint(imgpts[1].x, imgpts[1].y), myradius, CV_RGB(255, 0, 0), -1, 8, 0);
				line(frame, cvPoint(imgpts[0].x, imgpts[0].y), cvPoint(imgpts[1].x, imgpts[1].y), CV_RGB(255, 0, 0),4);
				circle(frame, cvPoint(imgpts[2].x, imgpts[2].y), myradius, CV_RGB(0, 255, 0), -1, 8, 0);
				line(frame, cvPoint(imgpts[0].x, imgpts[0].y), cvPoint(imgpts[2].x, imgpts[2].y), CV_RGB(0, 255, 0),4);
				circle(frame, cvPoint(imgpts[3].x, imgpts[3].y), myradius, CV_RGB(0, 0, 255), -1, 8, 0);
				line(frame, cvPoint(imgpts[0].x, imgpts[0].y), cvPoint(imgpts[3].x, imgpts[3].y), CV_RGB(0, 0, 255),4);
			imshow("Axis", frame);
		}
		

		//k = cv2.waitKey(0) & 0xFF
		//if k == ord('s') :
		//		cv2.imwrite(fname[:6] + '.png', img)


		/*Mat hsv;
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
		imshow("Film_linia", linia);*/
		cv::waitKey(10);
	}

	waitKey(0);
	return 0;
}

bool projectImagePointsOntoPlane(const vectorVector2d &pts,
	vectorVector3d &pts3d,
	const cv::Mat &cameraMatrix,
	const Eigen::Vector4d &planeEq)
{
	static constexpr double eps = 1e-6;

	float fx = cameraMatrix.at<float>(0, 0);
	float fy = cameraMatrix.at<float>(1, 1);
	float cx = cameraMatrix.at<float>(0, 2);
	float cy = cameraMatrix.at<float>(1, 2);

	// camera center in homogeneous cooridinates
	Eigen::Vector4d C;
	C << 0, 0, 0, 1;

	double den = planeEq.transpose() * C;

	// plane through camera center
	if (abs(den) < eps) {
		return false;
	}
	else {
		// pseudoinverse of the camera matrix P
		Eigen::Matrix<double, 4, 3> Ppinv;
		Ppinv << 1 / fx, 0, -cx / fx,
			0, 1 / fy, -cy / fy,
			0, 0, 1,
			0, 0, 0;

		Eigen::Matrix<double, 1, 3> pPpinv = planeEq.transpose() * Ppinv;

		for (const Eigen::Vector2d &pt2d : pts) {
			// point in homogeneous cooridinates
			Eigen::Vector3d pt;
			pt << pt2d(0), pt2d(1), 1;

			double lambda = -(pPpinv * pt)(0) / den;

			Eigen::Vector4d pt3d = Ppinv * pt + lambda * C;

			// adding point in inhomogeneous coordinates
			pts3d.push_back(pt3d.head<3>() / pt3d(3));
		}
		return true;
	}
}