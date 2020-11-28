#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros_tutorial_msgs/msgData.h"

#include <math.h>
#include <iostream>
#include <string>
#include <cmath>

#define DEG2RAD (M_PI / 180)
#define RAD2DEG (180 / M_PI)
#define Cam_height 60
#define theta 60

using namespace cv;
using namespace std;

geometry_msgs::Point end_point;
uint8_t Mode=0;
ros_tutorial_msgs::msgData pubdata;
//video size 680*480
//video center 340*240
void ImageCallback(const sensor_msgs::Image::ConstPtr &img){
  static ros::NodeHandle n;
  static ros::Publisher point_pub = n.advertise<geometry_msgs::Point>("/end_point",10);
  static ros::Publisher mode_pub = n.advertise<ros_tutorial_msgs::msgData>("mode_data",20);

  mode_pub.publish(pubdata);
  cv_bridge::CvImagePtr cv_ptr; //원본 영상
  Mat HSVImage; //HSV 영상
  Mat ThreshImage; //이진화 영상
  Mat img_labels, stats, centroids;
  int numOfLables;
  int max = -1, idx = 0;
  int left;
  int top;
  int width;
  int height;
  int x;
  int y;
  int area;
  int map_x;
  int map_y;
  int detect_count = 0;
  int dist_count=0;
  int dist_l=0;
  int dist_y=0;


  try {//start OpenCV
    //get video & change
    cv_ptr =cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cvtColor(cv_ptr->image,HSVImage,CV_BGR2HSV);
    inRange(HSVImage,Scalar(170,120,70),Scalar(180,255,255),ThreshImage);
    //remove noise
    erode(ThreshImage,ThreshImage,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
    dilate(ThreshImage,ThreshImage,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
    dilate(ThreshImage,ThreshImage,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
    erode(ThreshImage,ThreshImage,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));

    numOfLables = connectedComponentsWithStats(ThreshImage, img_labels,
                                               stats, centroids, 8, CV_32S);
    for (int j = 1; j < numOfLables; j++) {
      area = stats.at<int>(j, CC_STAT_AREA);
      if (max < area)
      {
        max = area;
        idx = j;
      }
    }

    left = stats.at<int>(idx, CC_STAT_LEFT);
    top = stats.at<int>(idx, CC_STAT_TOP);
    width = stats.at<int>(idx, CC_STAT_WIDTH);
    height = stats.at<int>(idx, CC_STAT_HEIGHT);
    x = centroids.at<double>(idx, 0);
    y = centroids.at<double>(idx, 1);
    map_x=x-320;
    map_y=-y+240;

    rectangle(cv_ptr->image, Point(left, top), Point(left + width, top + height),
              Scalar(0, 0, 255), 1);
    putText(cv_ptr->image, to_string(1), Point(left+20,top+20),
            FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0), 2);
    circle(cv_ptr->image, Point(x, y), 5, Scalar(255, 0, 0), 1);
    if(x!=319 && y!=239)
      detect_count=1;
    else
      detect_count=0;
    if(detect_count==1 && map_x>-80&&map_x<80 &&map_y>-160&&map_y<1 60)
    {
      dist_count=1;
      dist_l=Cam_height/cos(theta*DEG2RAD);
      dist_y=Cam_height*tan(theta*DEG2RAD);
      end_point.x = 0;
      end_point.y = 48;
      end_point.z = 0;
    }
    else {
      dist_count=0;
      dist_l=0;
      dist_y=0;
    }
    if(dist_count == 1){
      cout << "pose pub" << std::endl;
      Mode = 1;

      point_pub.publish(end_point);
    }
    //pubdata.mode = Mode;
  }

  catch (cv_bridge::Exception &e) {
    ROS_ERROR("Error to convert!");
  }

  ROS_INFO("Image Size (%d %d)", img->width,img->height);
  ROS_INFO("Image X,Y(%d %d)",x,y);
  ROS_INFO("Map X,Y(%d %d)",map_x,map_y);
  ROS_INFO("detect_count : %d",detect_count);
  ROS_INFO("dist_count : %d",dist_count);
  ROS_INFO("dist L,Y(%d %d)",dist_l,dist_y);

  cv::imshow("Image Show(raw)",cv_ptr->image);
  cv::imshow("Image Show(hsv)",HSVImage);
  cv::imshow("Image Show(thr)",ThreshImage);
  cv::waitKey(1);
}
void poseCallback(const geometry_msgs::Point::ConstPtr &data) {
  cout << data->x << data->y << data->z << std::endl;

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_test_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe("image_raw", 100, ImageCallback);
  ros::Subscriber _end_pose = nh.subscribe("end_point",10,poseCallback);
  ROS_INFO("##########START!##########");

  ros::spin();
}
