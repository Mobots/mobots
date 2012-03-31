#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <ros/ros.h>
/*#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>

ros::ServiceClient gazeboSetClient;
ros::ServiceClient gazeboGetClient;*/

void handleSignal(int signal){
  exit(0);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "mouse_node");
  
  /*ros::NodeHandle n;
  gazeboSetClient = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazeboSetClient.waitForExistence();
  gazeboGetClient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state", true);
  gazeboGetClient.waitForExistence();*/
  
  FILE *fmouse;
  char b[3];
  fmouse = fopen("/dev/input/mice", "r");
  int xd=0,yd=0; //x/y movement delta
  int xo=0,yo=0; //x/y overflow (out of range -255 to +255)
  int lb=0,mb=0,rb=0,hs=0,vs=0; //left/middle/right mousebutton

  signal(SIGINT, handleSignal);
  while(1){
    fread(b, sizeof(char), 3, fmouse);
    lb=(b[0]&1)>0;
    rb=(b[0]&2)>0;
    mb=(b[0]&4)>0;
    hs=(b[0]&16)>0;
    vs=(b[0]&32)>0;
    xo=(b[0]&64)>0;
    yo=(b[0]&128)>0;
    xd=b[1];
    yd=b[2];
    printf("xdelta = %d ydelta = %d\n", xd, yd);
    /*gazebo_msgs::GetModelState getState;
    getState.request.model_name = std::string("mobot");
    getState.request.relative_entity_name = std::string("world");
    gazeboGetClient.call(getState);

    gazebo_msgs::ModelState modelState;
    modelState.model_name = std::string("mobot");
    modelState.reference_frame = std::string("world"); 

    modelState.twist.linear.x = double(xd); //b[1];
    modelState.twist.linear.y = double(yd); //b[2];
    modelState.twist.linear.z = 0;
    modelState.twist.angular.x = 0;
    modelState.twist.angular.y = 0;
    modelState.twist.angular.z = 0;
    
    modelState.pose = getState.response.pose;
    
    gazebo_msgs::SetModelState setModelState; 
    setModelState.request.model_state = modelState;
    gazeboSetClient.call(setModelState);*/
  }
  fclose(fmouse);
}
