#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <geometry_msgs/Point.h>

#define CAMERAINTERFACE 0

using namespace cv;

int main(int argc, char* argv[])
{
  /* Initialize ROS stuff */
  ros::init(argc, argv, "opencv_candlefinder");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Point>("flame_coord", 1000);

  ros::Rate rate(10); // 10 Hz

  /* Initialize camera stuff */
  int cameraNum = CAMERAINTERFACE;

  //comment out for now
  /*if(argc > 1) {
    char * p;
    cameraNum = strtol(argv[1], &p, 10);
  }*/

  ROS_INFO_STREAM("Camera Number" << cameraNum);

  /* Get camera feed */
  system(("./setExposure.sh "+ std::to_string(cameraNum)).c_str()); // sets exposure of video1 to 1
  VideoCapture cap(cameraNum); // open camera

  if(!cap.isOpened()) {  // check if we succeeded
    ROS_INFO_STREAM("it didn't open :(");
    return -1;
  }

  namedWindow("robovision",1); // computer output window

  /* Simple Blob Detector parameters */
  /*SimpleBlobDetector::Params params;
  // Change thresholds
  params.minThreshold = 10;
  params.maxThreshold = 200;
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 50;

  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);*/

  while(ros::ok())
  {
    Mat frame;
    cap >> frame; // get a new frame from camera

    /* Process frame image */
    cvtColor(frame, frame, COLOR_BGR2GRAY);
    GaussianBlur(frame, frame, Size(3,3), 1.5, 1.5); // remove noise

    //threshold hopefully
    // Set threshold and maxValue
    double thresh = 127;
    double maxValue = 255;

    // Binary Threshold
    threshold(frame,frame, thresh, maxValue, 1);
    imshow("robovision2", frame);
    bitwise_not(frame, frame); // invert colors - possibly unnecessary idk

    /* Search for flame and draw a rectangle around it */
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    findContours( frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    /// Approximate contours to polygons + get bounding rects and circles
    std::vector<std::vector<Point> > contours_poly( contours.size() );
    std::vector<Rect> boundRect( contours.size() );
    //std::vector<Point2f>center( contours.size() );
    //std::vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
      boundRect[i] = boundingRect( Mat(contours_poly[i]) );
      //minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }
    /// Draw polygonal contour + bonding rects + circles
    Mat drawing = Mat::zeros( frame.size(), CV_8UC3 );

    geometry_msgs::Point pointMsg;
    if(contours.size() > 0) {
      Rect r = boundRect[0];

      ROS_INFO_STREAM( "Flame coordinates: (" << (r.x+(r.width/2.0)) << ", " << (r.y+(r.height/2.0)) << ")" );

      pointMsg.x = (r.x+(r.width/2.0))
      pointMsg.y = keypoints[0].pt.y;

      pub.publish(pointMsg);
    }else {
      pointMsg.x = -1;
      pointMsg.y = -1;
    }

    pub.publish(pointMsg);

    imshow("robovision", dst ); // show frame on computer output window

    /* Escape = kill program
    int keypress = waitKey(10)%255;
    if(keypress == 27){
        ROS_INFO_STREAM( << "key pressed: " << keypress << " see you later! :)" );
      break;
    }
    */
    rate.sleep();
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
