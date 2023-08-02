#include "drone.h"

void Drone::marker_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    marker_pos_msg = *msg;
    is_marker_detected = true;
}

// Function to set the marker detection status using a service call(true-> vision_node tries to detect marker and publish its position; false-> does not detect marker)
void Drone::SetMarkerDetectionStatus(bool status){

    MarkerDetectionSetStatusClient = nh.serviceClient<drone_pkg::MarkerDetectionStatus>("/vision/set_marker_detection_status");
    set_marker_detection_status_cmd.request.command = status;
        // Make the service call
    if (MarkerDetectionSetStatusClient.call(set_marker_detection_status_cmd)) {
        if (set_marker_detection_status_cmd.response.success) {
            ROS_INFO("Service call was successful");
        } else {
            ROS_WARN("Service call was not successful");
        }
    } else {
        ROS_ERROR("Failed to call service");
    }

}

void Drone::search_marker(){
    ROS_INFO("Marker is being search for landing.\n");
    param.request.value.real= 2;
    param.request.param_id = "MPC_XY_VEL_MAX ";
   
    if(param_set_client.call(param)==true && param.response.success == 1){
        ROS_INFO("Max horizontal speed parameter is set to %.2f for marker searching\n",param.request.value.real);
    }

    for(int i = 0;i<marker_search_trajectory.size() && ros::ok();){
        pos_msg.pose.position.x = marker_search_trajectory.at(i).x;
        pos_msg.pose.position.y = marker_search_trajectory.at(i).y;
        pos_msg.pose.position.z = marker_search_trajectory.at(i).z;
        if(reached_status(0.15)){i++;}//Eğer traj_pos vectoründe sıradaki pozisyona vardıysa bir sonraki position'ı hedef veriyor.

        if(is_marker_detected){
            ROS_INFO("Marker is found!\n");
            break;}
    }
}

void Drone::mode_land(){

    param.request.value.real = 4;
    param.request.param_id = "MPC_LAND_SPEED";
   
    if(param_set_client.call(param)==true && param.response.success == 1){
        ROS_INFO("Landing speed is set to %.2f\n",param.request.value.real);
    }

    fly_mode.request.custom_mode = "AUTO.LAND";
    if(mode_client.call(fly_mode) == true){
        if(fly_mode.response.mode_sent == true){
            ROS_INFO("\nAuto.Land mode: Active!\n");
        }
    }

}

void Drone::create_landing_trajectory(int current_landing_step){
                //DRONE LOCAL POZİSYONU GLOBALDEN FARKLI OLDUĞUNDAN Z'Yİ YANLIŞ ALIYORUM -> BUNU SOR(fark 0.2)
    //Burada tvecsin negatif olmasının sebebi image plane orientantation
    pos_msg.pose.position.x = marker_pos_msg.pose.position.x;//tvecs'in 2. elemanı droneun frameinde x'e denk geliyor.
    pos_msg.pose.position.y = marker_pos_msg.pose.position.y;
    pos_msg.pose.position.z = marker_pos_msg.pose.position.z + (1.5 - current_landing_step*0.2);

    
}



void Drone::marker_landing(){

    SetMarkerDetectionStatus(true);//This calls service for making vision_node to detect and publish marker.

    //Marker'a iniş komutu geldiğinde subscriberı oluştur
    marker_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vision/MarkerPosition",10,&Drone::marker_pos_cb,this);

    ros::Rate loop_rate(LOOP_RATE);
    int current_landing_step = 0;

    param.request.value.real = 4;
    param.request.param_id = "MPC_Z_VEL_MAX_DN";
   
    if(param_set_client.call(param)==true && param.response.success == 1){
        ROS_INFO("\nMax descend velocity is changed to %.2f\n",param.request.value.real);
    }
    


    //              Marker Arama Trajectorysini gezerek markerı  ara
    search_marker();

    //                     Markerı  Bulduktan sonra
    while(ros::ok()){

        
        create_landing_trajectory(current_landing_step);//Marker'a yaklaşırken adım adım yaklaşma için gereken mesajları oluşturur
        
        if(reached_status(0.15)){
            current_landing_step++;
        }

        if(odom.pose.position.z < 0.6){break;}//0.5 metreden az kaldıysa otonom inişe al

        ros::spinOnce();
        loop_rate.sleep();

    }
    pos_pub_bool=false;

    mode_land();
}