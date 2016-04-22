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
Mat img,img1;

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

//-----------------------------------------------------------//

// For Dilation
Mat dilation_dst;

int dilation_elem = 0;
int dilation_size = 3;
int const max_elem = 2;
int const max_kernel_size = 21;

//----------------------------------------------------------//

/// Function headers
void Threshold_Demo( int, void* );
void Dilation( int, void* );

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

       // Rotate the Image

	Point2f src_center(cv_ptr->image.cols/2.0F, cv_ptr->image.rows/2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, 180, 1.0);
	warpAffine(cv_ptr->image, img, rot_mat, cv_ptr->image.size());
 
//-----------------------------------------------------------//
// Blurring and implemeting discrete analog of the Laplacian operator
  GaussianBlur( img, img, Size(7,7), 0, 0, BORDER_DEFAULT );

  /// Convert the image to grayscale
  cvtColor( img, src_gray, CV_BGR2GRAY );
  //equalizeHist( src_gray, src_gray );

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

// Dilation

  namedWindow( window_name, CV_WINDOW_AUTOSIZE );
  cvMoveWindow( "Dilation Demo", img1.cols, 0 );

  /// Create Dilation Trackbar
  createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", window_name,&dilation_elem, max_elem, Dilation );

  createTrackbar( "Kernel size:\n 2n +1", window_name, &dilation_size, max_kernel_size, Dilation );

  /// Default start
  Dilation( 0, 0 );

//-----------------------------------------------------------//
// Mask Top half Image

 cv::Rect blankRoi(0, 0, dilation_dst.size().width, dilation_dst.size().height / 2.25);
 dilation_dst(blankRoi).setTo(cv::Scalar(0));

//
    
    Mat color_dst;
    cvtColor( dilation_dst, color_dst, CV_GRAY2BGR );
    vector<Vec4i> lines;
    HoughLinesP( dilation_dst, lines, 1, CV_PI/180, 100, 10, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( color_dst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
    }

    // Update GUI Window

    cv::imshow(OPENCV_WINDOW, color_dst);
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

  threshold( abs_dst, img1, threshold_value, max_BINARY_value,threshold_type );
  equalizeHist( img1, img1 );

  imshow( window_name, img1 );
}


/** @function Dilation */
void Dilation( int, void* )
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( img1, dilation_dst, element );
  imshow( "Dilation Demo", dilation_dst );
}
/** @function Masking/
void maskingimg(cv:: Mat &dilation_dst, cv:: Mat &dstGRAY, int tau)
{
 Mat dstGRAY;
 dstGRAY.setTo(0);
 int aux = 0;
 int tau;
 int i,j;
 for ( j=0 ; j<(dilation_dst.row) ; ++j )
 {
  unsigned char *ptRowSrc = dilation_dst.ptr<uchar>(j);
  unsigned char *ptRowDst = dstGRAY.ptr<uchar>(j);

  for ( i=tau ; j<(dilation_dst.cols) - tau ; ++i )
  {
    if(ptRowSrc[i]!=0)
	{
         aux = 2*ptRowSrc[i];	 
         aux += -ptRowSrc[i-tau];
         aux += -ptRowSrc[i+tau];
         aux += -abs((int)(ptRowSrc[i-tau]-ptRowSrc[i+tau]));
         aux = (aux<0)?(0):(aux);
         aux = (aux>255)?(255):(aux);
         ptRowDst[i] = (unsigned char)aux;
	}
   }
 }
    }
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

