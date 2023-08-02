#include "drone.h"


double Drone::pid_calculate(double setpoint, double measuredValue, double dt) {
    double error = setpoint - measuredValue;


    // Proportional term
    double pTerm = Kp * error;

    // Integral term
    integral += error * dt;
    double iTerm = Ki * integral;

    // Derivative term
    double dTerm = Kd * (error - prevError) / dt;
    prevError = error;

    // Calculate the output
    double output = pTerm + iTerm + dTerm;

    return output;
}

Drone::Drone(int argc,char** argv,int drone_ID){
    this->argc = argc;
    this->argv = argv;
    
    this->ID = drone_ID;
    
}

Drone::~Drone(){
    goal_thr.join();
    attitude_thr.join();
};



//------------------**********      Callbacks       *********-----------------------//

void Drone::state_cb(const mavros_msgs::State::ConstPtr &msg){
    
    state = *msg;
}

void Drone::odom_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    odom = *msg;
}

void Drone::gp_get_cb(const sensor_msgs::NavSatFix::ConstPtr &msg){
    gp_get_origin_msg = *msg;

    // Stop the subscriber
    gp_get_origin_sub.shutdown();
}

void Drone::MPC_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    mpc_cb_msg = *msg;
    is_mpc_publishing = true;
    pos_pub_bool = false;
}


//------------------**********      METHODS       *********-----------------------//

void Drone::set_pubs_subs_clients(){
        //Servisler subscriberlar vs definitionı
    param_set_client = nh.serviceClient<mavros_msgs::ParamSet> ("/mavros/param/set");
    arming_client    = nh.serviceClient<mavros_msgs::CommandBool> ("/mavros/cmd/arming");
    mode_client      = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
    land_client      = nh.serviceClient<mavros_msgs::CommandTOL> ("/mavros/cmd/land");

    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,&Drone::state_cb,this);
    odom_sub  = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,&Drone::odom_cb,this);
    MPC_sub   = nh.subscribe<geometry_msgs::TwistStamped>("/MPC_outputs",10,&Drone::MPC_cb,this);
    gp_get_origin_sub    = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global",10,&Drone::gp_get_cb,this);
    

    pos_pub         = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    gp_origin_pub   = nh.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin",10);
    att_pub         = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
}

void Drone::wait_connection(){

    ros::Rate loop_rate(LOOP_RATE);

    while(ros::ok() && state.connected ==false){

        ROS_INFO("Not connected,waiting...%d",state.connected);
        ros::spinOnce();
        ros::Duration(2.0).sleep();
    } 
}

void Drone::gp_origin_set(){

    gp_set_origin_msg.position.latitude=  gp_get_origin_msg.latitude;
    gp_set_origin_msg.position.longitude= gp_get_origin_msg.longitude;
    gp_set_origin_msg.position.altitude=  gp_get_origin_msg.altitude;
  
    gp_origin_pub.publish(gp_set_origin_msg);

}

void Drone::set_sim_params(){
    param.request.value.integer = 0;
    param.request.param_id = "NAV_DLL_ACT";
   
    if(param_set_client.call(param)==true && param.response.success == 1){
        ROS_INFO("NAV_DLL_ACT is set to 0.\n");
    }

    param.request.param_id = "NAV_RCL_ACT";
    if(param_set_client.call(param)==true && param.response.success == 1){
        ROS_INFO("NAV_RCL_ACT is set to 0.\n");
    }
}

void Drone::mode_offboard(){

    ros::Rate loop_rate(LOOP_RATE);
    for(int i = 0;i<100 && ros::ok();i++){
        pos_msg.pose.position.x = 0;
        pos_msg.pose.position.y = 0;
        pos_msg.pose.position.z = 0;
        pos_pub.publish(pos_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    fly_mode.request.custom_mode = "OFFBOARD";
    if(mode_client.call(fly_mode) == true){
        if(fly_mode.response.mode_sent == true){
            ROS_INFO("\nOFFBOARD mode: Active!\n");
        }
    }
}

void Drone::arm(){
    arm_cmd.request.value = 1;
    if(arming_client.call(arm_cmd) == true && arm_cmd.response.result == 0){
        ROS_INFO("Drone is armed!\n");
    }else{
        ROS_INFO("Drone couldn't armed!\n");
    }
}
//
//void Drone::mode_land(){
//
//    fly_mode.request.custom_mode = "AUTO.LAND";
//    if(mode_client.call(fly_mode) == true){
//        if(fly_mode.response.mode_sent == true){
//            ROS_INFO("\nLAND mode: Active!\n");
//        }
//    }
//}

int Drone::reached_status(float tolerance){
    float delta_x = abs(pos_msg.pose.position.x - odom.pose.position.x);
    float delta_y = abs(pos_msg.pose.position.y - odom.pose.position.y);
    float delta_z = abs(pos_msg.pose.position.z - odom.pose.position.z);

    float dist  = sqrt(  pow(delta_x,2) +   pow(delta_y,2) + pow(delta_z,2));

    if(dist < tolerance){
        return 1;
    }
    else{
        return 0;
    }
}

void Drone::goal_thr_func(double tolerance){
    ros::Rate looprate(20);

    while(ros::ok() && pos_pub_bool){       
        pos_pub.publish(pos_msg);
            
        ros::spinOnce();
        looprate.sleep();

    }
}

void Drone::follow_trajectory(){

    pos_msg.pose.position.x = 0;
    pos_msg.pose.position.y = 0;
    pos_msg.pose.position.z = 0;
    goal_thr = std::thread(&Drone::goal_thr_func,this,0.15);//Goal threadini başlatır

        
    for(int i = 0;i<trajectory.size() && ros::ok();){
        pos_msg.pose.position.x = trajectory.at(i).x;
        pos_msg.pose.position.y = trajectory.at(i).y;
        pos_msg.pose.position.z = trajectory.at(i).z;
        if(reached_status(0.15)){i++;}//Eğer traj_pos vectoründe sıradaki pozisyona vardıysa bir sonraki position'ı hedef veriyor.

        if(is_mpc_publishing){break;}//Eğer mpcden herhangi bir input gelirse direkt position'ı trajectory'yi boşver, mpcyi dikkate al
    }
    ROS_INFO("Trajectory is completed");
      
}

void Drone::MPC_Control(){

    while(!is_mpc_publishing && ros::ok()){}//Trajectory tamamlandıktan sonra Mpc publishlemiyorsa,MPC publishleyene kadar bekle  

    attitude_thr = std::thread(&Drone::attitude_thr_func,this,0.15);//MPC'yi başlat

    while(is_mpc_publishing && ros::ok()&& MPC_sub.getNumPublishers()>0){}//MPC threadi çalışsın
}


//////---------------------oooooooooooooooooooooooo     MAIN WORK FUNCTİON   oooooooooooooooooooooooooooooo------------------------/////////////////


void Drone::work(){

    set_pubs_subs_clients();
    wait_connection();//State Connected:true olana kadar bekle
    gp_origin_set();//Drone'un o anlık konumu home olarak al
    set_sim_params();//simülasyon için parametreleri kapat
    mode_offboard();//offboard moda al
    arm();//Drone'u armla

    follow_trajectory();
    
    //MPC_Control();//Comment this if MPC is not implemented after trajectory following

    //Marker Landing
    marker_landing();

}

