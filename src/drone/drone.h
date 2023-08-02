/*
 File name: drone.h
 Author: Ender Yağcılar
 Utopia Project - Drone Movement
 E-mail: enderyagcilar1@hotmail.com
 Date created: 06.02.2023
 Date last modified: -
 */


#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>//Droneu armla
#include <mavros_msgs/ParamSet.h>//Her şey başlarken simülasyondaki DLL ve RCL'yi 0a eşitle
#include <mavros_msgs/State.h>//Drone statei al
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <thread>
#include <math.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <drone_pkg/MarkerDetectionStatus.h>



#define PI 3.14159265359
#define LOOP_RATE 20

/*  Kısaltmalar:
gp->global position
pos->position
acc->acceleration
prev->previous
curr->current
msg->message
cmd->command
param->parameter
att->attitude
*/


struct Point3D{
    float x;
    float y;
    float z;
};

class Drone{
    public:
        Drone(int argc,char** argv,int);
        ~Drone();
        void work();

    private:
        int argc;
        char** argv;
        int ID;


        ros::NodeHandle nh;

                    //----              Getting Drone Ready            ----//
        //Necessary subs and pubs are created to get drone working.
        //Publishers
        ros::Publisher gp_origin_pub;
        //Subscribers
        ros::Subscriber state_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber gp_get_origin_sub;//İlk başta global position'a home verilmesi gerekiyor ki land komutunu düzgün alsın
        //Global position verilmediği zaman no home hatası veriyor. Buradan en başta bir kez global position alınacak ve global position için home olarak atanacak.
        //Bu kod çalışmaya başladığı anda droneun olduğu yeri global position için home olarak alıcak

        //Clients
        ros::ServiceClient arming_client;
        ros::ServiceClient mode_client;
 
        /*Messages are listed below*/
        mavros_msgs::CommandBool arm_cmd;
        mavros_msgs::ParamSet param;
        mavros_msgs::State state;
        geometry_msgs::PoseStamped odom;//Konum datası -> local_positiondan çekiliyor
        mavros_msgs::SetMode fly_mode;//Offboard moda alırken kullandığım mesaj
        mavros_msgs::CommandTOL land_msg;//Land servisine verilecek mesaj bu
        geographic_msgs::GeoPointStamped gp_set_origin_msg;
        sensor_msgs::NavSatFix gp_get_origin_msg;    
        
        //Ros Publishers,Subscribers and Clients
        ros::ServiceClient param_set_client;
        ros::ServiceClient land_client;
    

        /*Callbacks*/
        void state_cb(const mavros_msgs::State::ConstPtr &msg);
        void odom_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void MPC_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void gp_get_cb(const sensor_msgs::NavSatFix::ConstPtr &msg);
        

        /*Methods*/
        void set_pubs_subs_clients();
        void wait_connection();//Waits till connected
        void gp_origin_set();
        void set_sim_params();
        void mode_offboard();
        void arm();





//                      ---------------------------------------ooooooooooooooooooooo--------------------------------------------                 //

                    //----              Position Control            ----//
        //Drone Position Trajectory
        std::vector<Point3D> trajectory {
            {0,0,2.5}
        };

        //Publishers
        ros::Publisher pos_pub;

        //Messages
        geometry_msgs::PoseStamped pos_msg;//hedef datası -> setpoint_position/local'e publanıyor

        //Methods
        void follow_trajectory();
        int reached_status(float tolerance);

        //Thread functions & Threads
        void goal_thr_func(double tolerance);
        std::thread goal_thr;    bool pos_pub_bool = true;

//                      ---------------------------------------ooooooooooooooooooooo--------------------------------------------                 //

                    //----                  LANDİNG                ----//
        //Buradaki methodlar da subscriberlar da marker_landing() çağırıldıktan sonra etkili olacak ve güncellenebilecektir

        //Landing Methods
        void search_marker();//Searches marker by traveling marker_search_trajectory
        void create_landing_trajectory(int current_landing_step);//This methods handles trajectory for landing operation
        void mode_land();//Drone is taken into landing mode
        void marker_landing();//All landing methods are combined in this method

        //Callbacks & Subscribers
        ros::Subscriber marker_pos_sub;
        void marker_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

        //Variables
        geometry_msgs::PoseStamped marker_pos_msg;
        bool is_marker_detected = false;//This stores boolean for marker detection. If marker is detected during search_marker(), this becomes true.
        std::vector<Point3D> marker_search_trajectory {//This Trajectory is searched for finding marker
            {-2,-2,2},
            {2,-2,2},

            {2,-1,2},
            {-2,-1,2},

            {-2,0,2},
            {2,0,2},

            {2,1,2},
            {-2,1,2},

            {2,2,2},
            {-2,2,2}
        };
        //Enable marker detection
        void SetMarkerDetectionStatus(bool status);
        ros::ServiceClient MarkerDetectionSetStatusClient;
        drone_pkg::MarkerDetectionStatus set_marker_detection_status_cmd;
        

//                      ---------------------------------------ooooooooooooooooooooo--------------------------------------------                 //
                    //----              Attitude Control            ----//
        //Publishers
        ros::Publisher att_pub;

        //Subscribers
        ros::Subscriber MPC_sub;

        //Messages
        geometry_msgs::TwistStamped mpc_cb_msg;
        mavros_msgs::AttitudeTarget att_msg;

        //Methods
        void MPC_Control();//Method for using MPC. It should be uncommented in the work() method for using MPC after trajectory following.
        void attitude_msg_angle_set(double roll,double pitch,double yaw);//Bunu kullanmıyoruz çünkü rate kullanıyoruz
        void attitude_msg_rate_set();//Bu metotla angular velocityleri veriyoruz. 
        //Açısal ivme modu olmadığından trapezoidal numerik toplamla açısal hız veriyoruz.
      
        //Thread functions & Threads
        void attitude_thr_func(int x);
        std::thread attitude_thr;bool is_mpc_publishing = false;

                //EFE MPC
        /*Drone Acceleration Initial values,first is old values and the second is current values*/
        float body_acc[2][3] = {{0,0,0},{0,0,0}};
        float delta_bodyrate[3] = {0,0,0};
        geometry_msgs::TwistStamped prev_mpc_msg;
        geometry_msgs::TwistStamped curr_mpc_msg;
        double time_diff;
        float kg;//Thrust in kg units

            /*      ---THRUST PİD VARİABLES---       */
        //Thrust PID Variables -> Nurullah MPC'nin thrustını ayarlayan sabitler
        double Kp = 0.1;   // Proportional gain
        double Ki = 0.2;   // Integral gain
        double Kd = 0.1;   // Derivative gain

        double prevError = 0;
        double integral = 0;
        double pid_dt=0;

        double pid_thrust;

        double pid_calculate(double setpoint, double measuredValue, double dt);
};
