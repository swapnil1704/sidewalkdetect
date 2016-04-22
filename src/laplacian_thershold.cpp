#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
  Mat img;

//-------------------------------------------------------------//
// For Laplacian   
  Mat src_gray, dst;
  int kernel_size = 3;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  char* window_name = "Laplace Demo";
  Mat abs_dst;
  int c;
//-------------------------------------------------------------//
// For Thersholding
/// Global variables

int threshold_value = 10;
int threshold_type = 0;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;
char* window_name1 = "Threshold Demo";
char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* trackbar_value = "Value";

/// Function headers
void Threshold_Demo( int, void* );
//------------------------------------------------------------//





class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  { 
    cv_bridge::CvImagePtr cv_ptr;

   try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
       // Rotate the Image

	Point2f src_center(cv_ptr->image.cols/2.0F, cv_ptr->image.rows/2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, 180, 1.0);
	warpAffine(cv_ptr->image, img, rot_mat, cv_ptr->image.size());

//------------------------------------------------------------//

// Blurring and implemeting discrete analog of the Laplacian operator
  GaussianBlur( img, img, Size(3,3), 0, 0, BORDER_DEFAULT );

  /// Convert the image to grayscale
  cvtColor( img, src_gray, CV_BGR2GRAY );

  /// Create window
  namedWindow( window_name, CV_WINDOW_AUTOSIZE );

  /// Apply Laplace function

  Laplacian( src_gray, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( dst, abs_dst );

  /// Show what you got
  imshow( window_name, abs_dst );

//--------------------------------------------------------------//    

//Thersholding Operation

  /// Create a window to display results
  namedWindow( window_name, CV_WINDOW_AUTOSIZE );

  /// Create Trackbar to choose type of Threshold
  createTrackbar( trackbar_type, window_name, &threshold_type, max_type, Threshold_Demo );
  createTrackbar( trackbar_value, window_name, &threshold_value, max_value, Threshold_Demo );

  /// Call the function to initialize
  Threshold_Demo( 0, 0 );

//------------------------------------------------------------//

    // Update GUI Window
    //    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(3);
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());


  }
};

/**
* @function Threshold_Demo
*/
void Threshold_Demo( int, void* )
{
  /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
   */

  threshold( abs_dst, dst, threshold_value, max_BINARY_value,threshold_type );

  imshow( window_name, dst );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

