#include "vision.h"

Vision::Vision(int argc,char** argv,int drone_ID){
    this->argc = argc;
    this->argv = argv;
    
    this->ID = drone_ID;
    
}

Vision::~Vision(){

};

void Vision::odom_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    odom = *msg;
}


void Vision::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
    try
    {
        // Convert ROS image message to OpenCV image
        img_captured = cv_bridge::toCvShare(msg, "bgr8")->image;
        outputImage = img_captured.clone(); 
        
        if(markerDetectionEnabled){
            cv::Mat img_processed = img_preprocessing(outputImage);
            aruco_detection(img_processed);
        }

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // Create an OpenCV window for displaying the image
    cv::namedWindow("Captured Image", cv::WINDOW_NORMAL);  
    // Display the captured image
    cv::imshow("Captured Image", outputImage);
    cv::waitKey(1);
  
}




cv::Mat Vision::img_preprocessing(cv::Mat img){
 
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);  
    //          First, too dark areas made 0 intensity, others 255 intensity
    
        for (int row = 0; row < img.rows; row++)
    {
        for (int col = 0; col < img.cols; col++)
        {
            uchar intensity = img.at<uchar>(row, col);

            if (intensity >= 0 && intensity <= 10)
                img.at<uchar>(row, col) = 0;
            else
                img.at<uchar>(row, col) = 255;
        }
    }
    //          Morphological Operations are conducted

    // Define the structuring element for morphology operations
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    // Perform opening operation
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, element);

    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // Apply erosion to straighten the black pixels
    cv::erode(img, img, element);

    return img.clone();
}


void Vision::adjustCameraParameters(){
    // Set coordinate system
    objPoints  = cv::Mat(4, 1, CV_32FC3);

    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

    //Set Camera Calibration for fpv model(Read from camerainfo topic)
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    double focal_length_x = 277.191356;
    double focal_length_y = 277.191356;
    double image_center_x = 160;//cx ve cy'yi camera_info'daki gibi almadım çünkü kamera orjinini camera pixellerinin
    double image_center_y = 120;//bile dışına taşıyacak şekilde veriyor camera_info. Height ve width'in yarısı olarak aldım.
    // Assigning values to the camera matrix
    cameraMatrix.at<double>(0, 0) = focal_length_x; // Focal length in the x direction
    cameraMatrix.at<double>(1, 1) = focal_length_y; // Focal length in the y direction
    cameraMatrix.at<double>(0, 2) = image_center_x; // X coordinate of the image center
    cameraMatrix.at<double>(1, 2) = image_center_y; // Y coordinate of the image center
    
    // Define the distortion coefficients
    distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0) = 0; // First distortion coefficient
    distCoeffs.at<double>(1) = 0; // Second distortion coefficient
    distCoeffs.at<double>(2) = 0; // Third distortion coefficient
    distCoeffs.at<double>(3) = 0; // Fourth distortion coefficient
    distCoeffs.at<double>(4) = 0; // Fifth distortion coefficient
}

void Vision::projectPoints(cv::Vec3d rvecs, cv::Vec3d tvecs){
        //Project Points
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(objPoints, rvecs, tvecs, cameraMatrix, distCoeffs, projectedPoints);

    // Draw the projected points on the image
    for (const auto& point : projectedPoints) {
        cv::circle(outputImage, point, 3, cv::Scalar(255, 255, 0), cv::FILLED);
    }
}

void Vision::marker_positioning(cv::Vec3d rvecs, cv::Vec3d tvecs){
                //DRONE LOCAL POZİSYONU GLOBALDEN FARKLI OLDUĞUNDAN Z'Yİ YANLIŞ ALIYORUM -> BUNU SOR
    //Burada tvecsin negatif olmasının sebebi image plane orientantation
    marker_pos_msg.pose.position.x  = odom.pose.position.x - tvecs[1];//tvecs'in 2. elemanı droneun frameinde x'e denk geliyor.
    marker_pos_msg.pose.position.y  = odom.pose.position.y - tvecs[0];
    marker_pos_msg.pose.position.z  = odom.pose.position.z - tvecs[2] +0.2;// Bu 0.2'nın sebebi set_gp_originde odom pose 0lanıyor. o fark ortalama 0.2.
    //Daha açık olması açısından odom mesajı drone_node çalıştırılmaya bağlandıktan sonra izlenmelidir. set_gp_originden sonra z direkt 0.2den 0 oluyor

    marker_pos_pub.publish(marker_pos_msg);
}

void Vision::aruco_detection(cv::Mat img_processed){

    cv::aruco::detectMarkers(img_processed,dictionaryPtr,markerCorners,markerIds,detectorParamsPtr);
    
    if(markerIds.size()>0){

        nMarkers = markerCorners.size();
        cv::Vec3d rvecs(nMarkers), tvecs(nMarkers);

        // Calculate pose for each marker
        solvePnP(objPoints, markerCorners.at(0), cameraMatrix, distCoeffs, rvecs, tvecs);
       
        //Project Points
        projectPoints(rvecs, tvecs);


        // Draw axis for marker
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs, tvecs, 1);

        marker_positioning(rvecs, tvecs);
        marker_detection_error();    
    }
}
void Vision::marker_detection_error(){
    marker_position_estimation_error.x = marker_pos_msg.pose.position.x - marker_position_ground_truth[0];
    marker_position_estimation_error.y = marker_pos_msg.pose.position.y - marker_position_ground_truth[1];
    marker_position_estimation_error.z = marker_pos_msg.pose.position.z - marker_position_ground_truth[2];

    //std::cout<<marker_position_estimation_error<<std::endl;//ERRORU OKUMAK İÇİN BUNU UNCOMMENTLEYBİLİRSİNİZ.
}

bool Vision::set_marker_detection_status_cb(drone_pkg::MarkerDetectionStatus::Request& req,
                       drone_pkg::MarkerDetectionStatus::Response& res) {
    markerDetectionEnabled = req.command;
    ROS_INFO("Marker Detection Status: %s", markerDetectionEnabled ? "Enabled" : "Disabled");
    res.success = true; 
    return true;
}


//////---------------------oooooooooooooooooooooooo     MAIN WORK FUNCTİON   oooooooooooooooooooooooooooooo------------------------/////////////////


void Vision::work(){
    set_marker_detection_status = nh.advertiseService("/vision/set_marker_detection_status", &Vision::set_marker_detection_status_cb,this);

    adjustCameraParameters();
    image_transport::ImageTransport it(nh);
    //Subscribers
    odom_sub  = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,&Vision::odom_cb,this);
    image_transport::Subscriber sub = it.subscribe("/iris/usb_cam/image_raw", 1, &Vision::imageCallback,this);

    //Publishers
    marker_pos_pub         = nh.advertise<geometry_msgs::PoseStamped>("/vision/MarkerPosition",10);


    ros::spin();

    cv::destroyWindow("Captured Image");
}



