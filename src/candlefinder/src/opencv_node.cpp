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
  SimpleBlobDetector::Params params;
  // Change thresholds
  params.minThreshold = 10;
  params.maxThreshold = 200;
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 50;

  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

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
    threshold(frame,frame, thresh, maxValue, THRESH_BINARY);

    bitwise_not(frame, frame); // invert colors - you are looking for the light but this searches for dark

    /* Search for flame and draw a red circle around it */
    std::vector<KeyPoint> keypoints;
    detector->detect( frame, keypoints);
    drawKeypoints( frame, keypoints, frame, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    geometry_msgs::Point pointMsg;
    if(keypoints.size() > 0) {
      ROS_INFO_STREAM( "Flame coordinates: (" << keypoints[0].pt.x << ", " << keypoints[0].pt.y << ")" );

      pointMsg.x = keypoints[0].pt.x;
      pointMsg.y = keypoints[0].pt.y;

      pub.publish(pointMsg);
    }else {
      pointMsg.x = -1;
      pointMsg.y = -1;
    }

    pub.publish(pointMsg);

    imshow("robovision", frame ); // show frame on computer output window

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
