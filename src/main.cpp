#include "tag_detector/header.h"

void Tag::onInit()
{
    img_subscriber_= nh_.subscribe("/stereo_inertial_publisher/color/image", 1, &Tag::receiveFromCam,this);
    pnp_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("tag_pnp_publisher", 1);

    hsv_red_publisher_ = nh_.advertise<sensor_msgs::Image>("tag_red_hsv_publisher", 1);
    hsv_blue_publisher_ = nh_.advertise<sensor_msgs::Image>("tag_blue_hsv_publisher", 1);
    masked_red_publisher_ = nh_.advertise<sensor_msgs::Image>("tag_masked_red_publisher", 1);
    masked_blue_publisher_ = nh_.advertise<sensor_msgs::Image>("tag_masked_blue_publisher", 1);
    segmentation_publisher_ = nh_.advertise<sensor_msgs::Image>("tag_segmentation_publisher", 1);
    callback_ = boost::bind(&Tag::dynamicCallback, this, _1);
    server_.setCallback(callback_);

    distortion_coefficients_ = (cv::Mat_<double>(1,5)<<-0.228270, 0.063140, 0.002289, -0.000506, 0.000000);
    camera_matrix_ = (cv::Mat_<double>(3,3)<<773.90787,   0.     , 635.16925,
            0.     , 774.67309, 366.48879,
            0.     ,   0.     ,   1. );

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

    cv::resize(binary_a,binary_a,cv::Size(80,80),0,0,cv::INTER_NEAREST);
    cv::resize(binary_b,binary_b,cv::Size(80,80),0,0,cv::INTER_NEAREST);
    cv::resize(binary_c,binary_c,cv::Size(80,80),0,0,cv::INTER_NEAREST);
    cv::resize(binary_d,binary_d,cv::Size(80,80),0,0,cv::INTER_NEAREST);
    cv::resize(binary_e,binary_e,cv::Size(80,80),0,0,cv::INTER_NEAREST);

    binary_a_=binary_a;
    binary_b_=binary_b;
    binary_c_=binary_c;
    binary_d_=binary_d;
    binary_e_=binary_e;

//    cv::imshow("outputa",binary_a);
//    cv::imshow("outputb",binary_b);
//    cv::imshow("outputc",binary_c);
//    cv::imshow("outputd",binary_d);
//    cv::imshow("outpute",binary_e);
//    cv::waitKey(0);
//    cv::destroyAllWindows();
    std::cout<<"temp init finished"<<std::endl;
}


void Tag::receiveFromCam(const sensor_msgs::ImageConstPtr& image)
{
    cv_image_ = boost::make_shared<cv_bridge::CvImage>(*cv_bridge::toCvShare(image, image->encoding));

    imgProcess();
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
}

void Tag::pubMessage(const cv::Mat &rvec,const cv::Mat &tvec)
{
//    cv::Mat rotate_mat;
//    cv::Rodrigues(rvec, rotate_mat);


//    tf::Matrix3x3 tf_rotate_matrix(rotate_mat.at<double>(0, 0), rotate_mat.at<double>(0, 1), rotate_mat.at<double>(0, 2),
//                                   rotate_mat.at<double>(1, 0), rotate_mat.at<double>(1, 1), rotate_mat.at<double>(1, 2),
//                                   rotate_mat.at<double>(2, 0), rotate_mat.at<double>(2, 1), rotate_mat.at<double>(2, 2));

    tf::Matrix3x3 tf_rotate_matrix(1,0,0
                                   ,0,1,0
                                   ,0,0,1);
    tf::Vector3 tf_tvec(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    tf::Quaternion quaternion;
    double r;
    double p;
    double y;
    static geometry_msgs::PoseStamped pose_stamped;
    tf_rotate_matrix.getRPY(r, p, y);
    quaternion.setRPY(y, p, r);

    pose_stamped.pose.position.x=tvec.at<double>(0,0);
    pose_stamped.pose.position.y=tvec.at<double>(0,1);
    pose_stamped.pose.position.z=tvec.at<double>(0,2);
    pose_stamped.pose.orientation.w=quaternion.w();
    pose_stamped.pose.orientation.x=quaternion.x();
    pose_stamped.pose.orientation.y=quaternion.y();
    pose_stamped.pose.orientation.z=quaternion.z();

    pnp_publisher_.publish(pose_stamped);
    tf::Transform transform;
    transform.setRotation(quaternion);
    transform.setOrigin(tf_tvec);
    tf::StampedTransform stamped_Transfor(transform, ros::Time::now(), "oak_rgb_camera_optical_frame","tag");
    static tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(stamped_Transfor);
}

void Tag::resultVisualizaion(const std::vector<cv::Point2i> &hull,const cv::Point2f (&vertex)[4],const int angle,const int signal)
{
    auto moment = cv::moments(hull);
    int cx = int(moment.m10 / moment.m00);
    int cy = int(moment.m01 / moment.m00);
    cv::Point2f centroid(cx, cy);


    if(angle<=0)
    {
        std::vector<cv::Point3f> w_points_vec;
        w_points_vec.push_back(cv::Point3f (0.075,0.075,0)); //bl
        w_points_vec.push_back(cv::Point3f(-0.075,0.075,0)); //tl
        w_points_vec.push_back(cv::Point3f(-0.075,-0.075,0)); //tr
        w_points_vec.push_back(cv::Point3f(0.075,-0.075,0)); //br
        std::vector<cv::Point2f> p_points_vec;
        for (int i = 0; i < 4; i++)
        {
            cv::line(cv_image_->image, vertex[i], vertex[(i + 1) % 4], cv::Scalar(0, 255, 0),3);
            cv::putText(cv_image_->image,std::to_string(i),vertex[i],1,3,cv::Scalar(0,255,0),3);
            p_points_vec.push_back(vertex[i]);
        }
        cv::Mat rvec;
        cv::Mat tvec;
        cv::solvePnP(w_points_vec,p_points_vec,camera_matrix_,distortion_coefficients_,rvec,tvec,bool(),cv::SOLVEPNP_ITERATIVE);
        pubMessage(rvec,tvec);
        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
        cv::putText(cv_image_->image,std::to_string(signal),centroid,1,3,cv::Scalar(255,0,0),3);
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,0)),centroid-cv::Point2f (100,100),1,3,cv::Scalar(0,0,255),3);
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,1)),centroid-cv::Point2f (50,50),1,3,cv::Scalar(0,0,255),3);
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,2)),centroid-cv::Point2f (10,10),1,3,cv::Scalar(0,0,255),3);
        cv::putText(cv_image_->image,std::to_string(angle),centroid+cv::Point2f (50,50),1,3,cv::Scalar(255,0,255),3);
    }

    else
    {
        std::vector<cv::Point3f> w_points_vec;
        w_points_vec.push_back(cv::Point3f(0.075,-0.075,0)); //br
        w_points_vec.push_back(cv::Point3f (0.075,0.075,0)); //bl
        w_points_vec.push_back(cv::Point3f(-0.075,0.075,0)); //tl
        w_points_vec.push_back(cv::Point3f(-0.075,-0.075,0)); //tr
        std::vector<cv::Point2f> p_points_vec;
        for (int i = 0; i < 4; i++)
        {
            cv::line(cv_image_->image, vertex[i], vertex[(i + 1) % 4], cv::Scalar(0, 255, 0),3);
            cv::putText(cv_image_->image,std::to_string(i),vertex[i],1,3,cv::Scalar(0,255,0),3);
            p_points_vec.push_back(vertex[i]);
        }
        cv::Mat rvec;
        cv::Mat tvec;
        cv::solvePnP(w_points_vec,p_points_vec,camera_matrix_,distortion_coefficients_,rvec,tvec,bool(),cv::SOLVEPNP_ITERATIVE);
        pubMessage(rvec,tvec);
        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
        cv::putText(cv_image_->image,std::to_string(signal),centroid,1,3,cv::Scalar(255,0,0),3);
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,0)),centroid-cv::Point2f (100,100),1,3,cv::Scalar(0,0,255),3);
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,1)),centroid-cv::Point2f (50,50),1,3,cv::Scalar(0,0,255),3);
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,2)),centroid-cv::Point2f (10,10),1,3,cv::Scalar(0,0,255),3);
        cv::putText(cv_image_->image,std::to_string(angle),centroid+cv::Point2f (50,50),1,3,cv::Scalar(255,0,255),3);

    }

}

int Tag::recognizeLetter(const cv::Mat * reverse_mask_ptr)
{
    std::vector<std::pair<int,int>> similar_value_vec;
    cv::Mat tmp_a,tmp_b,tmp_c,tmp_d,tmp_e;
    cv::bitwise_xor(*reverse_mask_ptr,binary_a_,tmp_a);
    cv::bitwise_xor(*reverse_mask_ptr,binary_b_,tmp_b);
    cv::bitwise_xor(*reverse_mask_ptr,binary_c_,tmp_c);
    cv::bitwise_xor(*reverse_mask_ptr,binary_d_,tmp_d);
    cv::bitwise_xor(*reverse_mask_ptr,binary_e_,tmp_e);

    similar_value_vec.emplace_back(std::make_pair(0,cv::countNonZero(tmp_a)));
    similar_value_vec.emplace_back(std::make_pair(1,cv::countNonZero(tmp_b)));
    similar_value_vec.emplace_back(std::make_pair(2,cv::countNonZero(tmp_c)));
    similar_value_vec.emplace_back(std::make_pair(3,cv::countNonZero(tmp_d)));
    similar_value_vec.emplace_back(std::make_pair(4,cv::countNonZero(tmp_e)));

    std::sort(similar_value_vec.begin(),similar_value_vec.end(),[](std::pair<int,int> val1, std::pair<int,int> val2){return val1.second<val2.second;});

    return similar_value_vec[0].first;
}



void Tag::contoursProcess(const cv::Mat *mor_ptr,int color) {
    auto *contours_vec_ptr = new std::vector<std::vector<cv::Point>>();
    cv::findContours(*mor_ptr, *contours_vec_ptr, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    auto *hull_vec_ptr = new std::vector<std::vector<cv::Point2i>>();
    for (auto &contours: *contours_vec_ptr) {
        std::vector<cv::Point2i> hull;
        cv::convexHull(contours, hull, true);
        hull_vec_ptr->emplace_back(hull);
    }
    std::sort(hull_vec_ptr->begin(), hull_vec_ptr->end(),
              [](const std::vector<cv::Point2i> &hull1, const std::vector<cv::Point2i> &hull2) {
                  return cv::contourArea(hull1) > cv::contourArea(hull2);
              });

    if (hull_vec_ptr->empty()) {
        std::cout << "can not find mineral in this frame" << std::endl;
        return;
    }

    auto max_area_hull = hull_vec_ptr[0][0];
    delete hull_vec_ptr;
    delete contours_vec_ptr;

    auto rotate_rect = cv::minAreaRect(max_area_hull);
    cv::Point2f vertex[4];
    rotate_rect.points(vertex);


    int angle;
    if (rotate_rect.size.width <= rotate_rect.size.height) {
        angle = rotate_rect.angle + 90;
        int tm = rotate_rect.size.width;
        rotate_rect.size.width = rotate_rect.size.height;
        rotate_rect.size.height = tm;
    } else {
        angle = rotate_rect.angle;
    }


    auto trans_matrix = cv::getRotationMatrix2D(rotate_rect.center, angle, 1);
    auto *mask = new cv::Mat();
    auto *reverse_mask = new cv::Mat();
    auto *warp_result = new cv::Mat();
    cv::warpAffine(*mor_ptr, *warp_result, trans_matrix, cv::Size(mor_ptr->cols, mor_ptr->rows));
    cv::getRectSubPix(*warp_result, rotate_rect.size, rotate_rect.center, *mask);
    delete warp_result;
    cv::bitwise_not(*mask, *reverse_mask);
    delete mask;
    if(reverse_mask->data == nullptr)
    {
        std::cout<<"wrong object,return now"<<std::endl;
        return;
    }
    cv::resize(*reverse_mask,*reverse_mask,cv::Size(80,80),0,0,cv::INTER_NEAREST);
    if (color)
        masked_red_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", *reverse_mask).toImageMsg());
    else masked_blue_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", *reverse_mask).toImageMsg());
    int signal=recognizeLetter(reverse_mask);
    delete reverse_mask;
    resultVisualizaion(max_area_hull,vertex,angle,signal);
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
//    contoursProcess(mor_blue_ptr,0);

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
