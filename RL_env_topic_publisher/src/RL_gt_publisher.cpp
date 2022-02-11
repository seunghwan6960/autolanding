#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <algorithm>
#include <ros/console.h>
#include <tf/tf.h>


#include <mutex>
#include <map>
#include <numeric>

#include <fstream>
#include <vector>
#include <string>
#include <pwd.h>
#include <eigen3/Eigen/Dense>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double,12,1 > Vector12d;
typedef Eigen::Matrix<double,3,1 > Vector3d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double,6,1 > Vector6d;
typedef Eigen::Matrix<double,4,4 > Matrix4d;
typedef Eigen::Matrix<double,3,3 > Matrix3d;

const double pi = M_PI;

static double Extrinsic[16]={0, -1, 0, 0, -1, 0, 0, 0, 0, 0, -1, -0.1, 0, 0, 0, 1  };

class gt_publisher{
private:

ros::NodeHandle nh;

ros::Subscriber drone_gt_sub;
ros::Subscriber pad_gt_sub;

ros::Publisher pad_postion_pub;
ros::Publisher pad_velocity_pub;



Vector3d pad_position_gf;
Vector3d pad_velocity_gf;
Vector3d drone_velocity;
Vector6d drone_pose;


double gt_topic_hz;

std::mutex pose_marker_lock;

std_msgs::Float64MultiArray msg_gt_position;
std_msgs::Float64MultiArray msg_gt_vel;

public:
  gt_publisher(): nh("~"){ 

    msg_gt_position.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_gt_position.layout.dim[0].label="gt_position";
    msg_gt_position.layout.dim[0].size=3;
    msg_gt_position.layout.dim[0].stride=3;  

    msg_gt_vel.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg_gt_vel.layout.dim[0].label="gt_velocity";
    msg_gt_vel.layout.dim[0].size=3;
    msg_gt_vel.layout.dim[0].stride=3;  
   
    drone_gt_sub=nh.subscribe("/gazebo_groundtruth/drone_odometry",1, &gt_publisher::drone_odom_callback,this,ros::TransportHints().tcpNoDelay());
    pad_gt_sub=nh.subscribe("/gazebo_groundtruth/ground_vehicle",1, &gt_publisher::pad_odom_callback,this,ros::TransportHints().tcpNoDelay());
    
    
    pad_postion_pub= nh.advertise<std_msgs::Float64MultiArray>("gt_position", 1);
    pad_velocity_pub= nh.advertise<std_msgs::Float64MultiArray>("gt_velocitiy", 1);

    ros::spin();
  }

  void drone_odom_callback(const nav_msgs::Odometry& msg){
    pose_marker_lock.lock();
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(msg.pose.pose.orientation,quat_tf);
    tf::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    drone_pose<< msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,roll,pitch,yaw;
    drone_velocity<< msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z;
    pose_marker_lock.unlock();

  }

  void pad_odom_callback(const nav_msgs::Odometry& msg){
      pad_position_gf<< msg.pose.pose.position.x,msg.pose.pose.position.y,(msg.pose.pose.position.z+0.4);
      pad_velocity_gf<< msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z;


      Vector3d pad_position_df =position_fr_change(pad_position_gf);
      Vector3d pad_vel_df =vel_fr_change(pad_velocity_gf);

      std::vector <double> position={pad_position_df(0),pad_position_df(1),pad_position_df(2)};
      std::vector <double> vel={pad_vel_df(0),pad_vel_df(1),pad_vel_df(2)};


      msg_gt_position.data=position;
      msg_gt_vel.data=vel;  

      pad_postion_pub.publish(msg_gt_position);
      pad_velocity_pub.publish(msg_gt_vel);
  }


  

Vector3d position_fr_change(Vector3d x_g){
    Vector3d x_d;
    Vector3d t;
    Matrix3d R;
    
   
    R=rotation_matrix(drone_pose(3),drone_pose(4),drone_pose(5));
    t<<drone_pose(0),drone_pose(1),drone_pose(2);
    
    x_d=R.inverse()*x_g+(-R.inverse()*t);
    
    return x_d;
}

Vector3d vel_fr_change(Vector3d v_g){
    Vector3d v_d;
    Vector3d t;
    Matrix3d R;

   
    R=rotation_matrix(drone_pose(3),drone_pose(4),drone_pose(5));
    
    v_d=R.inverse()*v_g+(-R.inverse()*drone_velocity);
    
    return v_d;
}

  Eigen::Matrix<double,3,4> inverse_TF(Matrix4d T){
    Eigen::Matrix<double,3,4> T_i;
    Matrix3d R;
    Vector3d t;
    R=T.topLeftCorner(3,3);
    t<<T(0,3),T(1,3),T(2,3);
    T_i.topLeftCorner(3,3)=R.inverse();
    T_i.topRightCorner(3,1)=-R.inverse()*t;

    return T_i;
  }

  Matrix4d make_TF(Vector6d xx){
    Matrix4d T;
    T=Matrix4d::Zero();
    T(0,3)=xx(0);T(1,3)=xx(1);T(2,3)=xx(2);T(3,3)=1.0;
    T.topLeftCorner(3, 3)=rotation_matrix(xx(3),xx(4),xx(5));

    return T;
  }

  Matrix3d rotation_matrix(double phi, double theta, double psi){
    Matrix3d R_phi, R_theta, R_psi, R;
    R<<cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi),
      sin(psi)*cos(theta), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi),
      -sin(theta), cos(theta)*sin(phi),  cos(theta)*cos(phi);

    return R;
  }

  Vector6d TF_to_vec(Matrix4d T){
    Vector6d z;
    z(0)=T(0,3);z(1)=T(1,3);z(2)=T(2,3);

    Vector3d ori;
    ori=rotiation_matrix_to_vec(T.topLeftCorner(3, 3));

    z(3)=ori(0);z(4)=ori(1);z(5)=ori(2);

    return z;
  }

  Vector3d rotiation_matrix_to_vec(Matrix3d R){
    Vector3d r;


    r(2)=atan2(R(1,0),R(0,0));
    r(1)=atan2(-R(2,0),sqrt(pow(R(0,0),2)+pow(R(1,0),2)));
    r(0)=atan2(R(2,1),R(2,2));

    
    return r;
  }

double angle_manipulation(double y){
  while(y>pi){
    y=y-2*pi;
  }

  while(y<-pi){
    y=y+2*pi;
  }
  return y;
}

};

int main(int argc, char** argv)
{
  


ros::init(argc, argv, "RL_gt_pub");

gt_publisher TP;

  
  return 0;
}