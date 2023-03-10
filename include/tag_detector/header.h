#pragma once

#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "algorithm"
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "dynamic_reconfigure/server.h"
#include "tag_detector/dynamicConfig.h"
#include"iostream"
#include"fstream"
#include "thread"
#include "mutex"
#include "future"
#include "chrono"
#include "ctime"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

class Tag
{
public:
    void onInit();
    void dynamicCallback(tag_detector::dynamicConfig& config);

    void receiveFromCam(const sensor_msgs::ImageConstPtr &image);
    void imgProcess();
    void contoursProcess(const cv::Mat *mor_ptr,int color);
    void resultVisualizaion(const std::vector<cv::Point2i> &hull,double scale);
    void pubMessage(const cv::Mat &rvec,const cv::Mat &tvec);

    ros::NodeHandle nh_;
    ros::Subscriber img_subscriber_;
    ros::Subscriber depth_subscriber_;
    ros::Subscriber points_subscriber_;

    ros::Publisher hsv_red_publisher_;
    ros::Publisher hsv_blue_publisher_;
    ros::Publisher masked_red_publisher_;
    ros::Publisher masked_blue_publisher_;
    ros::Publisher segmentation_publisher_;
    ros::Publisher pnp_publisher_;

    dynamic_reconfigure::Server<tag_detector::dynamicConfig> server_;
    dynamic_reconfigure::Server<tag_detector::dynamicConfig>::CallbackType callback_;
    cv_bridge::CvImagePtr cv_image_;
    cv_bridge::CvImagePtr depth_image_;

    cv::Mat distortion_coefficients_;
    cv::Mat camera_matrix_;

    int morph_type_;
    int morph_iterations_;
    int morph_size_;

    int lower_red_hsv_h_;
    int lower_red_hsv_s_;
    int lower_red_hsv_v_;
    int upper_red_hsv_h_;
    int upper_red_hsv_s_;
    int upper_red_hsv_v_;

    int lower_blue_hsv_h_;
    int lower_blue_hsv_s_;
    int lower_blue_hsv_v_;
    int upper_blue_hsv_h_;
    int upper_blue_hsv_s_;
    int upper_blue_hsv_v_;

    std::vector<cv::Point> hull_a_;
    std::vector<cv::Point> hull_b_;
    std::vector<cv::Point> hull_c_;
    std::vector<cv::Point> hull_d_;
    std::vector<cv::Point> hull_e_;

};