/*
 File name: vision.h
 Author: Ender Yağcılar
 Utopia Project - Drone Vision
 E-mail: enderyagcilar1@hotmail.com
 Date created: 02.07.2023
 Date last modified: -
 */

#include <ros/ros.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>//Quarternion olarak kullanmıyorum direkt euler angle

#include <drone_pkg/MarkerDetectionStatus.h>

class Vision{
    public:
        Vision(int argc,char** argv,int);
        ~Vision();
        void work();
        

    private:
        int argc;
        char** argv;
        int ID;

        ros::NodeHandle nh;
        
        //This boolean stores variable for starting vision node marker detection. While this variable is false, vision node waits in an infinite loop.
        //When drone_node makes a service call for starting marker detection, this variable becomes true and marker detection starts
        bool markerDetectionEnabled = false;
        ros::ServiceServer set_marker_detection_status;
        bool set_marker_detection_status_cb(drone_pkg::MarkerDetectionStatus::Request& req,
                       drone_pkg::MarkerDetectionStatus::Response& res);




        //Aruco Detection
        std::vector<int> markerIds;
        int nMarkers;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParamsPtr = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary>  dictionaryPtr = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        double markerArea;
        cv::Point2f marker_center;
        float markerLength = 0.5;//Markerın boyutu
        //Camera Parameters
        cv::Mat objPoints;
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;

        cv::Mat img_captured;
        cv::Mat outputImage;


        //Messages
        geometry_msgs::PoseStamped marker_pos_msg;//Quarternion olarak kullanmıyorum direkt euler angle
        geometry_msgs::PoseStamped odom;//Konum datası -> local_positiondan çekiliyor

        //Subscribers
        ros::Subscriber camSub;
        ros::Subscriber odom_sub;

        //Publishers
        ros::Publisher marker_pos_pub;
        image_transport::Publisher pub;

        //Methods
        cv::Mat img_preprocessing(cv::Mat img);
        void adjustCameraParameters();
        void projectPoints(cv::Vec3d rvecs, cv::Vec3d tvecs);
        void aruco_detection(cv::Mat img_processed);
        void marker_positioning(cv::Vec3d rvecs, cv::Vec3d tvecs);
        
        //Callbacks
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        void odom_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);


        /*      Marker center's position in gazebo world(This is used for error calculation)    */
        float marker_position_ground_truth[3] = {1,-1,0.1};//x,y,z ,MARKER KONUMU DEĞİŞTİYSE BUNU GÜNCELLENMELİDİR.(Markerın sdf dosyasında merkezi ortası değil altı)
        cv::Point3f marker_position_estimation_error;
        //This method calculates vision marker position detection's error using marker_pos_ground_truth[2] s ground truth.
        void marker_detection_error();

};