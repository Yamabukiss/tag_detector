#include "tag_detector/header.h"

void Tag::onInit()
{
    img_subscriber_= nh_.subscribe("/stereo_inertial_publisher/color/image", 1, &Tag::receiveFromCam,this);
    hsv_red_publisher_ = nh_.advertise<sensor_msgs::Image>("tag_red_hsv_publisher", 1);
    hsv_blue_publisher_ = nh_.advertise<sensor_msgs::Image>("tag_blue_hsv_publisher", 1);
    masked_red_publisher_ = nh_.advertise<sensor_msgs::Image>("tag_masked_red_publisher", 1);
    masked_blue_publisher_ = nh_.advertise<sensor_msgs::Image>("tag_masked_blue_publisher", 1);
    segmentation_publisher_ = nh_.advertise<sensor_msgs::Image>("tag_segmentation_publisher", 1);
    callback_ = boost::bind(&Tag::dynamicCallback, this, _1);
    server_.setCallback(callback_);

    cv::Mat temp_A=cv::imread("/home/yamabuki/detect_ws/src/tag_detector/A.png",cv::IMREAD_GRAYSCALE);
    cv::Mat temp_B=cv::imread("/home/yamabuki/detect_ws/src/tag_detector/B.png",cv::IMREAD_GRAYSCALE);
    cv::Mat temp_C=cv::imread("/home/yamabuki/detect_ws/src/tag_detector/C.png",cv::IMREAD_GRAYSCALE);
    cv::Mat temp_D=cv::imread("/home/yamabuki/detect_ws/src/tag_detector/D.png",cv::IMREAD_GRAYSCALE);
    cv::Mat temp_E=cv::imread("/home/yamabuki/detect_ws/src/tag_detector/E.png",cv::IMREAD_GRAYSCALE);

    cv::Mat binary_a,binary_b,binary_c,binary_d,binary_e;

    cv::threshold(temp_A,binary_a,0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    cv::threshold(temp_B,binary_b,0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    cv::threshold(temp_C,binary_c,0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    cv::threshold(temp_D,binary_d,0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
    cv::threshold(temp_E,binary_e,0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> contours_a_vec;
    std::vector<std::vector<cv::Point>> contours_b_vec;
    std::vector<std::vector<cv::Point>> contours_c_vec;
    std::vector<std::vector<cv::Point>> contours_d_vec;
    std::vector<std::vector<cv::Point>> contours_e_vec;

    std::vector<cv::Point> hull_a;
    std::vector<cv::Point> hull_b;
    std::vector<cv::Point> hull_c;
    std::vector<cv::Point> hull_d;
    std::vector<cv::Point> hull_e;

    cv::findContours(binary_a,contours_a_vec,cv::RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    std::sort(contours_a_vec.begin(),contours_a_vec.end(),[](const std::vector<cv::Point>& contour1,const std::vector<cv::Point>& contour2){return contour1.size()>contour2.size();});
//    cv::convexHull(contours_a_vec[1],hull_a, true);
    cv::polylines(temp_A,hull_a, true,cv::Scalar::all(0),3);

    hull_a_=hull_a;

    cv::findContours(binary_b,contours_b_vec,cv::RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    std::sort(contours_b_vec.begin(),contours_b_vec.end(),[](const std::vector<cv::Point>& contour1,const std::vector<cv::Point>& contour2){return contour1.size()>contour2.size();});
//    cv::convexHull(contours_b_vec[1],hull_b, true);
    cv::polylines(temp_B,hull_b, true,cv::Scalar::all(0),3);

    hull_b_=hull_b;

    cv::findContours(binary_c,contours_c_vec,cv::RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    std::sort(contours_c_vec.begin(),contours_c_vec.end(),[](const std::vector<cv::Point>& contour1,const std::vector<cv::Point>& contour2){return contour1.size()>contour2.size();});
//    cv::convexHull(contours_c_vec[1],hull_c, true);
    cv::polylines(temp_C,hull_c, true,cv::Scalar::all(0),3);

    hull_c_=hull_c;

    cv::findContours(binary_d,contours_d_vec,cv::RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    std::sort(contours_d_vec.begin(),contours_d_vec.end(),[](const std::vector<cv::Point>& contour1,const std::vector<cv::Point>& contour2){return contour1.size()>contour2.size();});
//    cv::convexHull(contours_d_vec[1],hull_d, true);
    cv::polylines(temp_D,hull_d, true,cv::Scalar::all(0),3);

    hull_d_=hull_d;

    cv::findContours(binary_e,contours_e_vec,cv::RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    std::sort(contours_e_vec.begin(),contours_e_vec.end(),[](const std::vector<cv::Point>& contour1,const std::vector<cv::Point>& contour2){return contour1.size()>contour2.size();});
    cv::convexHull(contours_e_vec[1],hull_e, true);
//    cv::polylines(temp_E,hull_e, true,cv::Scalar::all(0),3);
    hull_e_=hull_e;
//    cv::imshow("outputa",temp_A);
//    cv::imshow("outputb",temp_B);
//    cv::imshow("outputc",temp_C);
//    cv::imshow("outputd",temp_D);
//    cv::imshow("outpute",temp_E);
//    cv::waitKey(0);
//    cv::destroyAllWindows();
    std::cout<<"temp init finished"<<std::endl;
}

void Tag::receiveFromCam(const sensor_msgs::ImageConstPtr& image)
{
    cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));
    imgProcess();
//    cv::cvtColor(cv_image_->image,cv_image_->image,CV_GRAY2BGR);
    segmentation_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),cv_image_->encoding , cv_image_->image).toImageMsg());
//    getTemplateImg();
}

void Tag::dynamicCallback(tag_detector::dynamicConfig &config)
{
    morph_type_ = config.morph_type;
    morph_iterations_ = config.morph_iterations;
    
    lower_red_hsv_h_=config.lower_red_hsv_h;
    lower_red_hsv_s_=config.lower_red_hsv_s;
    lower_red_hsv_v_=config.lower_red_hsv_v;
    
    upper_red_hsv_h_=config.upper_red_hsv_h;
    upper_red_hsv_s_=config.upper_red_hsv_s;
    upper_red_hsv_v_=config.upper_red_hsv_v;
    
    lower_blue_hsv_h_=config.lower_blue_hsv_h;
    lower_blue_hsv_s_=config.lower_blue_hsv_s;
    lower_blue_hsv_v_=config.lower_blue_hsv_v;
    
    upper_blue_hsv_h_=config.upper_blue_hsv_h;
    upper_blue_hsv_s_=config.upper_blue_hsv_s;
    upper_blue_hsv_v_=config.upper_blue_hsv_v;
    

    morph_size_=config.morph_size;
    moment_bias_=config.moment_bias;
    approx_epsilon_=config.approx_epsilon;
}

void Tag::resultVisualizaion(const std::vector<cv::Point2i> &hull)
{
    if (cv::matchShapes(hull,hull_a_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_)
    {
        auto moment = cv::moments(hull);
        //centriod
        int cx = int(moment.m10 / moment.m00);
        int cy = int(moment.m01 / moment.m00);
        cv::Point2f centroid(cx, cy);
        // centroid and polylines green
        cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
        cv::putText(cv_image_->image,"A",centroid,1,3,cv::Scalar(0,255,255),3);
    }
    else if (cv::matchShapes(hull,hull_b_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_)
    {
        auto moment = cv::moments(hull);
        //centriod
        int cx = int(moment.m10 / moment.m00);
        int cy = int(moment.m01 / moment.m00);
        cv::Point2f centroid(cx, cy);
        // centroid and polylines green
        cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
        cv::putText(cv_image_->image,"B",centroid,1,3,cv::Scalar(0,255,255),3);
    }
    else if (cv::matchShapes(hull,hull_c_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_)
    {
        auto moment = cv::moments(hull);
        //centriod
        int cx = int(moment.m10 / moment.m00);
        int cy = int(moment.m01 / moment.m00);
        cv::Point2f centroid(cx, cy);
        // centroid and polylines green
        cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
        cv::putText(cv_image_->image,"C",centroid,1,3,cv::Scalar(0,255,255),3);
    }
    else if (cv::matchShapes(hull,hull_d_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_)
    {
        auto moment = cv::moments(hull);
        //centriod
        int cx = int(moment.m10 / moment.m00);
        int cy = int(moment.m01 / moment.m00);
        cv::Point2f centroid(cx, cy);
        // centroid and polylines green
        cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
        cv::putText(cv_image_->image,"D",centroid,1,3,cv::Scalar(0,255,255),3);
    }
    else if (cv::matchShapes(hull,hull_e_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_)
    {
        auto moment = cv::moments(hull);
        //centriod
        int cx = int(moment.m10 / moment.m00);
        int cy = int(moment.m01 / moment.m00);
        cv::Point2f centroid(cx, cy);
        // centroid and polylines green
        cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
        cv::putText(cv_image_->image,"E",centroid,1,3,cv::Scalar(0,255,255),3);
    }
    else std::cout<<"matches fail"<<std::endl;

}

void Tag::contoursProcess(const cv::Mat *mor_ptr,int color)
{
    auto * contours_vec_ptr = new std::vector<std::vector<cv::Point>> ();
    cv::findContours(*mor_ptr,*contours_vec_ptr,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    auto * blank_mask_ptr= new cv::Mat();
    *blank_mask_ptr=cv::Mat::zeros(cv_image_->image.rows,cv_image_->image.cols,CV_8UC1);
    auto * hull_vec_ptr = new std::vector<std::vector<cv::Point2i>> ();
    for (auto &contours : *contours_vec_ptr)
    {
        std::vector<cv::Point2i> hull;
        cv::convexHull(contours, hull, true);
        hull_vec_ptr->emplace_back(hull);
    }
    std::sort(hull_vec_ptr->begin(),hull_vec_ptr->end(),[](const std::vector<cv::Point2i> &hull1,const std::vector<cv::Point2i> &hull2){return cv::contourArea(hull1) > cv::contourArea(hull2);});

    if (hull_vec_ptr->empty())
    {
        std::cout<<"can not find mineral in this frame"<<std::endl;
        return;
    }

    auto max_area_hull=hull_vec_ptr[0][0];
    delete  hull_vec_ptr;
    delete contours_vec_ptr;
//    cv::fillConvexPoly(*blank_mask_ptr,max_area_hull,cv::Scalar(255));

    resultVisualizaion(max_area_hull);
//    auto moment = cv::moments(max_area_hull);
//    //centriod
//    int cx = int(moment.m10 / moment.m00);
//    int cy = int(moment.m01 / moment.m00);
//    cv::Point2f centroid(cx, cy);
//    // centroid and polylines green
//    cv::polylines(cv_image_->image, max_area_hull, true, cv::Scalar(0, 255, 0), 2);
//    cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);


//    auto * mask_ptr=new cv::Mat();
//    cv::bitwise_and(*mor_ptr,*blank_mask_ptr,*mask_ptr);
//    cv::bitwise_xor(*blank_mask_ptr,*mask_ptr,*mask_ptr);
//    if (color) masked_red_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , *mask_ptr).toImageMsg());
//    else masked_blue_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , *mask_ptr).toImageMsg());
//
//    delete blank_mask_ptr;
//    std::vector<std::vector<cv::Point>> contours;
//    cv::findContours(*mask_ptr,contours,cv::RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
//    delete mask_ptr;
//
//    for (auto &contour : contours)
//    {
//        std::vector<cv::Point2i> hull;
//        cv::convexHull(contour, hull, true);
//        resultVisualizaion(hull);
//    }
}



void Tag::imgProcess()
{
    //segementation
    auto * hsv_red_ptr = new cv::Mat();
    auto * hsv_blue_ptr = new cv::Mat();

    auto * mor_red_ptr=new cv::Mat();
    auto * mor_blue_ptr=new cv::Mat();

    auto * binary_red_ptr=new cv::Mat();
    auto * binary_blue_ptr=new cv::Mat();

    cv::cvtColor(cv_image_->image,*hsv_red_ptr,cv::COLOR_BGR2HSV);
    cv::cvtColor(cv_image_->image,*hsv_blue_ptr,cv::COLOR_BGR2HSV);
    cv::inRange(*hsv_red_ptr,cv::Scalar(lower_red_hsv_h_,lower_red_hsv_s_,lower_red_hsv_v_),cv::Scalar(upper_red_hsv_h_,upper_red_hsv_s_,upper_red_hsv_v_),*binary_red_ptr);
    cv::inRange(*hsv_blue_ptr,cv::Scalar(lower_blue_hsv_h_,lower_blue_hsv_s_,lower_blue_hsv_v_),cv::Scalar(upper_blue_hsv_h_,upper_blue_hsv_s_,upper_blue_hsv_v_),*binary_blue_ptr);

    delete hsv_red_ptr;
    delete hsv_blue_ptr;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1+2*morph_size_, 1+2*morph_size_), cv::Point(-1, -1));
    cv::morphologyEx(*binary_red_ptr,*mor_red_ptr,morph_type_,kernel,cv::Point(-1,-1),morph_iterations_);
    cv::morphologyEx(*binary_blue_ptr,*mor_blue_ptr,morph_type_,kernel,cv::Point(-1,-1),morph_iterations_);

    delete binary_red_ptr;
    delete binary_blue_ptr;
    hsv_red_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , *mor_red_ptr).toImageMsg());
    hsv_blue_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , *mor_blue_ptr).toImageMsg());

    contoursProcess(mor_red_ptr,1);
    contoursProcess(mor_blue_ptr,0);

    delete mor_red_ptr;
    delete mor_blue_ptr;
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "tag_detector_node");
    Tag tag;
    tag.onInit();
    while (ros::ok())
    {
        ros::spinOnce();
    }

}
