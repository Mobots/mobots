#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>

ros::ServiceClient gazeboSetClient;
ros::ServiceClient gazeboGetClient;


main(){
  ros::init(argc, argv, "mouse_node");
  
  ros::NodeHandle n;
  gazeboSetClient = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); //true für persistente Verbindung <> GEHT NICHT IN DIESEM FALL >_<
  gazeboSetClient.waitForExistence();
  gazeboGetClient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true);
  gazeboGetClient.waitForExistence();
  
  FILE *fmouse;
  char b[3];
  fmouse = fopen("/dev/input/mice","r");
  int xd=0,yd=0; //x/y movement delta
  int xo=0,yo=0; //x/y overflow (out of range -255 to +255)
  int lb=0,mb=0,rb=0,hs=0,vs=0; //left/middle/right mousebutton
  while(1){
    fread(b, sizeof(char), 3, fmouse);
    /*lb=(b[0]&1)>0;
    rb=(b[0]&2)>0;
    mb=(b[0]&4)>0;
    hs=(b[0]&16)>0;
    vs=(b[0]&32)>0;
    xo=(b[0]&64)>0;
    yo=(b[0]&128)>0;
    xd=b[1];
    yd=b[2];
    printf("hs=%d,vs=%d,lb=%d rm=%d mb=%d xo=%d yo=%d xd=%d yd=%d\n",hs,vs,lb,rb,mb,xo,yo,xd,yd);*/
    gazebo_msgs::GetModelState getState;
    getState.request.model_name = std::string("mobot");
    getState.request.relative_entity_name = std::string("world");
    gazeboGetClient.call(getState);

    gazebo_msgs::ModelState modelState;
    modelState.model_name = std::string("mobot");
    modelState.reference_frame = std::string("world"); 

    modelState.twist.x = b[1];
    modelState.twist.y = b[2];

    modelState.pose = getState.response.pose;
    
    gazebo_msgs::SetModelState setModelState; 
    setModelState.request.model_state = modelState;
    gazeboSetClient.call(setModelState);    
  }
  fclose(fmouse);
}
