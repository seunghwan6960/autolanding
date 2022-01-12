#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>





double gaussianRandom(double average, double stdev) {
  double v1, v2, s, temp;

  do {
    v1 =  2 * ((double) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 
    v2 =  2 * ((double) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 
    s = v1 * v1 + v2 * v2;
  } while (s >= 1 || s == 0);

  s = sqrt( (-2 * log(s)) / s );

  temp = v1 * s;
  temp = (stdev * temp) + average;

  return temp;
}




int main(int argc, char** argv)
{
  
  ros::init(argc, argv, " ");
  ros::NodeHandle nh;
  
  double v, w ,v_n,w_n,t;
  double hz;

  
  
  nh.param<double>("/linear_vel",v, 0);
  nh.param<double>("/angular_vel",w, 0);
  nh.param<double>("/linear_noise",v_n, 0);
  nh.param<double>("/angular_noise",w_n, 0);
  nh.param<double>("/time_for_desired_velocity",t, 0);
  nh.param<double>("/hz",hz, 10);
  

  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("ground_vehicle_cmd_vel", 10);
  geometry_msgs::Twist vel;
  vel.linear.x=0;
  vel.angular.z=0;

  int i=0;
  int i_max=int(t*hz);
  double d_v=v/(t*hz);
  double d_w=w/(t*hz);
  //std::cout<<"d_v: "<<d_v<<",  d_w: " <<d_w<<std::endl;

  ros::Rate loop_rate(hz);
  while(ros::ok()){

    if(i<i_max){
      vel.linear.x=vel.linear.x+d_v;
      vel.angular.z=vel.angular.z+d_w;
      i++;
    }
    else{
      vel.linear.x=v+gaussianRandom(0,v_n);
      vel.angular.z=w+gaussianRandom(0,w_n);
    }
    //std::cout<<"v: "<<vel.linear.x<<",  w: " <<vel.angular.z<<std::endl;
    pub.publish(vel);
    ros::spinOnce();
    loop_rate.sleep();
    
    
  }



  
  return 0;
}