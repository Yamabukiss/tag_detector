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

    distortion_coefficients_ = (cv::Mat_<double>(1,5)<<-0.226600, 0.060101, -0.000846, 0.000819, 0.000000);
    camera_matrix_ = (cv::Mat_<double>(3,3)<<774.28829,   0.     , 636.65422,
            0.     , 775.6586 , 386.23152,
            0.     ,   0.     ,   1.     );

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
    cv::convexHull(contours_a_vec[1],hull_a, true);
//    cv::polylines(temp_A,hull_a, true,cv::Scalar::all(0),3);
    hull_a_=hull_a;

    cv::findContours(binary_b,contours_b_vec,cv::RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    std::sort(contours_b_vec.begin(),contours_b_vec.end(),[](const std::vector<cv::Point>& contour1,const std::vector<cv::Point>& contour2){return contour1.size()>contour2.size();});
    cv::convexHull(contours_b_vec[1],hull_b, true);
//    cv::polylines(temp_B,hull_b, true,cv::Scalar::all(0),3);

    hull_b_=hull_b;

    cv::findContours(binary_c,contours_c_vec,cv::RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    std::sort(contours_c_vec.begin(),contours_c_vec.end(),[](const std::vector<cv::Point>& contour1,const std::vector<cv::Point>& contour2){return contour1.size()>contour2.size();});
    cv::convexHull(contours_c_vec[1],hull_c, true);
//    cv::polylines(temp_C,hull_c, true,cv::Scalar::all(0),3);

    hull_c_=hull_c;

    cv::findContours(binary_d,contours_d_vec,cv::RETR_TREE,CV_CHAIN_APPROX_SIMPLE);
    std::sort(contours_d_vec.begin(),contours_d_vec.end(),[](const std::vector<cv::Point>& contour1,const std::vector<cv::Point>& contour2){return contour1.size()>contour2.size();});
    cv::convexHull(contours_d_vec[1],hull_d, true);
//    cv::polylines(temp_D,hull_d, true,cv::Scalar::all(0),3);

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


void Tag::resultVisualizaion(const std::vector<cv::Point2i> &hull,double scale)
{
//    if (cv::matchShapes(hull,hull_a_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_)
//    {
//        auto moment = cv::moments(hull);
//        //centriod
//        int cx = int(moment.m10 / moment.m00);
//        int cy = int(moment.m01 / moment.m00);
//        cv::Point2f centroid(cx, cy);
//        // centroid and polylines green
//        cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
//        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
//        cv::putText(cv_image_->image,std::to_string(scale),centroid-cv::Point2f (10,10),1,3,cv::Scalar(0,255,255),3);
//        cv::putText(cv_image_->image,"A",centroid,1,3,cv::Scalar(0,255,255),3);
//    }
//    else if (cv::matchShapes(hull,hull_b_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_)
//    {
//        auto moment = cv::moments(hull);
//        //centriod
//        int cx = int(moment.m10 / moment.m00);
//        int cy = int(moment.m01 / moment.m00);
//        cv::Point2f centroid(cx, cy);
//        // centroid and polylines green
//        cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
//        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
//        cv::putText(cv_image_->image,"B",centroid,1,3,cv::Scalar(0,255,255),3);
//    }
//    else if (cv::matchShapes(hull,hull_c_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_)
//    {
//        auto moment = cv::moments(hull);
//        //centriod
//        int cx = int(moment.m10 / moment.m00);
//        int cy = int(moment.m01 / moment.m00);
//        cv::Point2f centroid(cx, cy);
//        // centroid and polylines green
//        cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
//        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
//        cv::putText(cv_image_->image,"C",centroid,1,3,cv::Scalar(0,255,255),3);
//    }
//    else if (cv::matchShapes(hull,hull_d_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_)
//    {
//        auto moment = cv::moments(hull);
//        //centriod

//        int cx = int(moment.m10 / moment.m00);
//        int cy = int(moment.m01 / moment.m00);
//        cv::Point2f centroid(cx, cy);
//        // centroid and polylines green
//        cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
//        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
//        cv::putText(cv_image_->image,"D",centroid,1,3,cv::Scalar(0,255,255),3);
//    }
//    else if (cv::matchShapes(hull,hull_e_,cv::CONTOURS_MATCH_I2,0)<=moment_bias_)
//    {
//        auto moment = cv::moments(hull);
//        //centriod
//        int cx = int(moment.m10 / moment.m00);
//        int cy = int(moment.m01 / moment.m00);
//        cv::Point2f centroid(cx, cy);
//        // centroid and polylines green
//        cv::polylines(cv_image_->image, hull, true, cv::Scalar(0, 255, 0), 2);
//        cv::circle(cv_image_->image, centroid, 2, cv::Scalar(0, 255, 0), 2);
//        cv::putText(cv_image_->image,"E",centroid,1,3,cv::Scalar(0,255,255),3);
//    }
//    else std::cout<<"matches fail"<<std::endl;
    auto moment = cv::moments(hull);
    //centriod
    int cx = int(moment.m10 / moment.m00);
    int cy = int(moment.m01 / moment.m00);
    cv::Point2f centroid(cx, cy);

    auto rotate_rect=cv::minAreaRect(hull);
    cv::Point2f vertex[4];
    rotate_rect.points(vertex);

    int angle;
    cv::Mat rotate_points;
    cv::boxPoints(rotate_rect,rotate_points);
    if (rotate_rect.size.width <= rotate_rect.size.height)
    {
        angle = rotate_rect.angle + 90;
        int tm = rotate_rect.size.width;
        rotate_rect.size.width = rotate_rect.size.height;
        rotate_rect.size.height = tm;
    }
    else
    {
        angle = rotate_rect.angle;
    }
    cv::putText(cv_image_->image,std::to_string(angle),rotate_rect.center,1,3,cv::Scalar(0,255,0),3);

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
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,0)),centroid-cv::Point2f (100,100),1,3,cv::Scalar(0,0,255),3);
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,1)),centroid-cv::Point2f (50,50),1,3,cv::Scalar(0,0,255),3);
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,2)),centroid-cv::Point2f (10,10),1,3,cv::Scalar(0,0,255),3);
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
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,0)),centroid-cv::Point2f (100,100),1,3,cv::Scalar(0,0,255),3);
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,1)),centroid-cv::Point2f (50,50),1,3,cv::Scalar(0,0,255),3);
        cv::putText(cv_image_->image,std::to_string(tvec.at<double>(0,2)),centroid-cv::Point2f (10,10),1,3,cv::Scalar(0,0,255),3);
    }

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
    auto rect =cv::boundingRect(max_area_hull);
    rect.x=rect.x;
    rect.y=rect.y;
    rect.width=rect.width;
    rect.height=rect.height;
//    cv::rectangle(cv_image_->image,rect,cv::Scalar(100,200,50),3);
    auto * roi=new cv::Mat (*mor_ptr,rect);
//    *mor_ptr(rect).copyTo(*roi);
    if(color) masked_red_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , *roi).toImageMsg());
    else     masked_blue_publisher_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8" , *roi).toImageMsg());
    int white_area=cv::countNonZero(*roi);
    delete roi;
    double white_area_scale=white_area/cv::contourArea(max_area_hull) ;

    resultVisualizaion(max_area_hull,white_area_scale);
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
//    contoursProcess(mor_blue_ptr,0);

    delete mor_red_ptr;
    delete mor_blue_ptr;
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "tag_detector_node");
    ros::MultiThreadedSpinner spinner(2);
    Tag tag;
    tag.onInit();
    while (ros::ok())
    {
        ros::spinOnce();
    }

}
