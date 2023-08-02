#include <ros/ros.h>
#include "vision.h"


int main(int argc,char** argv){
    ros::init(argc,argv,"vision");

    Vision* vision = new Vision(argc,argv,1);
    vision->work();

}