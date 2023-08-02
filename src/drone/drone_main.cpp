#include <ros/ros.h>
#include "drone.h"


int main(int argc,char** argv){
    ros::init(argc,argv,"drone");

    Drone* drone1 = new Drone(argc,argv,1);
    drone1->work();

}