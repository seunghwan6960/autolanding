#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <algorithm>
#include <ros/console.h>
#include <tf/tf.h>

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

class topic_publisher{
private:

ros::NodeHandle nh;

ros::Subscriber sub_cam1_m1_check;
ros::Subscriber sub_cam1_m2_check;
ros::Subscriber sub_cam1_m3_check;
ros::Subscriber sub_cam2_m1_check;
ros::Subscriber sub_cam2_m2_check;
ros::Subscriber sub_cam2_m3_check;

ros::Subscriber sub_cam1_m1_pose;
ros::Subscriber sub_cam1_m2_pose;
ros::Subscriber sub_cam1_m3_pose;
ros::Subscriber sub_cam2_m1_pose;
ros::Subscriber sub_cam2_m2_pose;
ros::Subscriber sub_cam2_m3_pose;

ros::Subscriber sub_cam1_m1_pixel;
ros::Subscriber sub_cam1_m2_pixel;
ros::Subscriber sub_cam1_m3_pixel;
ros::Subscriber sub_cam2_m1_pixel;
ros::Subscriber sub_cam2_m2_pixel;
ros::Subscriber sub_cam2_m3_pixel;

 ros::Subscriber drone_pose_sub;


ros::Publisher pub_pixel_array_cam1;
ros::Publisher pub_pixel_array_cam2;
ros::Publisher pub_pose_array_cam1;
ros::Publisher pub_pose_array_cam2;

std_msgs::Float64MultiArray cam1_pixel_list;
std_msgs::Float64MultiArray cam2_pixel_list;

std_msgs::Float64MultiArray cam1_pose_list;
std_msgs::Float64MultiArray cam2_pose_list;

std::vector <double> cam1_pixels;
std::vector <double> cam2_pixels;
std::vector <double> cam1_poses;
std::vector <double> cam2_poses;


bool check_drone_pose_recive;
double drone_pose[6];

public:
  topic_publisher(): nh("~"){
    sub_cam1_m1_check=nh.subscribe("/cam1_detector1/check_detection",1,&topic_publisher::c1_m1_check_callback,this);
    sub_cam1_m2_check=nh.subscribe("/cam1_detector2/check_detection",1,&topic_publisher::c1_m2_check_callback,this);
    sub_cam1_m3_check=nh.subscribe("/cam1_detector3/check_detection",1,&topic_publisher::c1_m3_check_callback,this);
    sub_cam2_m1_check=nh.subscribe("/cam2_detector1/check_detection",1,&topic_publisher::c2_m1_check_callback,this);
    sub_cam2_m2_check=nh.subscribe("/cam2_detector2/check_detection",1,&topic_publisher::c2_m2_check_callback,this);
    sub_cam2_m3_check=nh.subscribe("/cam2_detector3/check_detection",1,&topic_publisher::c2_m3_check_callback,this);  

    drone_pose_sub=nh.subscribe("/mavros/local_position/pose",1, &topic_publisher::drone_pose_callback,this);  

    
    pub_pixel_array_cam1= nh.advertise<std_msgs::Float64MultiArray>("long_focal_pixel_list", 10);
    pub_pixel_array_cam2= nh.advertise<std_msgs::Float64MultiArray>("wide_focal_pixel_list", 10);
    pub_pose_array_cam1 = nh.advertise<std_msgs::Float64MultiArray>("long_focal_pose_list", 10);
    pub_pose_array_cam2 = nh.advertise<std_msgs::Float64MultiArray>("wide_focal_pose_list", 10);


    cam1_pixel_list.layout.dim.push_back(std_msgs::MultiArrayDimension());
    cam1_pixel_list.layout.dim[0].label="cam1_pixel-list";
    cam1_pixel_list.layout.dim[0].size=24;
    cam1_pixel_list.layout.dim[0].stride=24;

    cam2_pixel_list.layout.dim.push_back(std_msgs::MultiArrayDimension());
    cam2_pixel_list.layout.dim[0].label="cam2_pixel-list";
    cam2_pixel_list.layout.dim[0].size=24;
    cam2_pixel_list.layout.dim[0].stride=24;

    cam1_pose_list.layout.dim.push_back(std_msgs::MultiArrayDimension());
    cam1_pose_list.layout.dim[0].label="cam1_pose-list";
    cam1_pose_list.layout.dim[0].size=9;
    cam1_pose_list.layout.dim[0].stride=9;


    cam2_pose_list.layout.dim.push_back(std_msgs::MultiArrayDimension());
    cam2_pose_list.layout.dim[0].label="cam2_pose-list";
    cam2_pose_list.layout.dim[0].size=9;
    cam2_pose_list.layout.dim[0].stride=9;

    cam1_pixels={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    cam2_pixels={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    cam1_poses={0,0,0,0,0,0,0,0,0};
    cam2_poses={0,0,0,0,0,0,0,0,0};
    
  }

  void drone_pose_callback(const geometry_msgs::PoseStamped &msg){

    

    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(msg.pose.orientation,quat_tf);
    tf::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    drone_pose[0]=msg.pose.position.x;
    drone_pose[1]=msg.pose.position.y;
    drone_pose[2]=msg.pose.position.z;
    drone_pose[3]=angle_manipulation(roll);
    drone_pose[4]=angle_manipulation(pitch);
    drone_pose[5]=angle_manipulation(yaw);

    
  }

  void c1_m1_check_callback(const std_msgs::Bool& check){
    if(check.data){
      sub_cam1_m1_pose=nh.subscribe("/cam1_detector1/pose",1, &topic_publisher::c1_m1_pose_callback,this);
      sub_cam1_m1_pixel=nh.subscribe("/cam1_detector1/pixel_list",1, &topic_publisher::c1_m1_pixel_callback,this);
    }
    else{
      for(int i=0;i<8;i++){
        cam1_pixels[i]=0;
      }
      
      for(int i=0;i<3;i++){
        cam1_poses[i]=0;
      }
    }
    cam1_pixel_list.data=cam1_pixels;
    cam2_pixel_list.data=cam2_pixels;
    cam1_pose_list.data=cam1_poses;
    cam2_pose_list.data=cam2_poses;


    pub_pixel_array_cam1.publish(cam1_pixel_list);
    pub_pixel_array_cam2.publish(cam2_pixel_list);
    pub_pose_array_cam1.publish(cam1_pose_list); 
    pub_pose_array_cam2.publish(cam2_pose_list);  


    


  }

  void c1_m2_check_callback(const std_msgs::Bool& check){
    if(check.data){
      sub_cam1_m2_pose=nh.subscribe("/cam1_detector2/pose",1, &topic_publisher::c1_m2_pose_callback,this);
      sub_cam1_m2_pixel=nh.subscribe("/cam1_detector2/pixel_list",1, &topic_publisher::c1_m2_pixel_callback,this);
    }
    else{
      for(int i=0;i<8;i++){
        cam1_pixels[8+i]=0;
      }
      
      for(int i=0;i<3;i++){
        cam1_poses[3+i]=0;
      }
    }
  }

   void c1_m3_check_callback(const std_msgs::Bool& check){
    if(check.data){
      sub_cam1_m3_pose=nh.subscribe("/cam1_detector3/pose",1, &topic_publisher::c1_m3_pose_callback,this);
      sub_cam1_m3_pixel=nh.subscribe("/cam1_detector3/pixel_list",1, &topic_publisher::c1_m3_pixel_callback,this);
    }
    else{
      for(int i=0;i<8;i++){
        cam1_pixels[2*8+i]=0;
      }
      
      for(int i=0;i<3;i++){
        cam1_poses[2*3+i]=0;
      }
    }
  }

  void c2_m1_check_callback(const std_msgs::Bool& check){
    if(check.data){
      sub_cam2_m1_pose=nh.subscribe("/cam2_detector1/pose",1, &topic_publisher::c2_m1_pose_callback,this);
      sub_cam2_m1_pixel=nh.subscribe("/cam2_detector1/pixel_list",1, &topic_publisher::c2_m1_pixel_callback,this);
    }
    else{
      for(int i=0;i<8;i++){
        cam2_pixels[i]=0;
      }
      
      for(int i=0;i<3;i++){
        cam2_poses[i]=0;
      }
    }
  }

  void c2_m2_check_callback(const std_msgs::Bool& check){
    if(check.data){
      sub_cam2_m2_pose=nh.subscribe("/cam2_detector2/pose",1, &topic_publisher::c2_m2_pose_callback,this);
      sub_cam2_m2_pixel=nh.subscribe("/cam2_detector2/pixel_list",1, &topic_publisher::c2_m2_pixel_callback,this);
    }
    else{
      for(int i=0;i<8;i++){
        cam2_pixels[8+i]=0;
      }
      
      for(int i=0;i<3;i++){
        cam2_poses[3+i]=0;
      }
    }
  }

   void c2_m3_check_callback(const std_msgs::Bool& check){
    if(check.data){
      sub_cam2_m3_pose=nh.subscribe("/cam2_detector3/pose",1, &topic_publisher::c2_m3_pose_callback,this);
      sub_cam2_m3_pixel=nh.subscribe("/cam2_detector3/pixel_list",1, &topic_publisher::c2_m3_pixel_callback,this);
    }
    else{
      for(int i=0;i<8;i++){
        cam2_pixels[2*8+i]=0;
      }
      
      for(int i=0;i<3;i++){
        cam2_poses[2*3+i]=0;
      }
    }
  }

  void c1_m1_pixel_callback(const std_msgs::Float64MultiArray msg){
    for (int i=0;i<8;i++){
      cam1_pixels[i]=msg.data[i];
    }
  }

  void c1_m2_pixel_callback(const std_msgs::Float64MultiArray msg){
    for (int i=0;i<8;i++){
      cam1_pixels[8+i]=msg.data[i];
    }
  }

  void c1_m3_pixel_callback(const std_msgs::Float64MultiArray msg){
    for (int i=0;i<8;i++){
      cam1_pixels[2*8+i]=msg.data[i];
    }
  }

   void c2_m1_pixel_callback(const std_msgs::Float64MultiArray msg){
    for (int i=0;i<8;i++){
      cam2_pixels[i]=msg.data[i];
    }
  }

  void c2_m2_pixel_callback(const std_msgs::Float64MultiArray msg){
    for (int i=0;i<8;i++){
      cam2_pixels[8+i]=msg.data[i];
    }
  }

  void c2_m3_pixel_callback(const std_msgs::Float64MultiArray msg){
    for (int i=0;i<8;i++){
      cam2_pixels[2*8+i]=msg.data[i];
    }
  }

  void c1_m1_pose_callback(const geometry_msgs::PoseStamped &msg){
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(msg.pose.orientation,quat_tf);
    tf::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    Vector6d x_c, x_g;
    x_c<<msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,roll,pitch,yaw;
    x_g=inv_fn_hx(x_c,Extrinsic);

    for(int i=0;i<3;i++){
        cam1_poses[i]=x_g(i);
    }
  }

  void c1_m2_pose_callback(const geometry_msgs::PoseStamped &msg){
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(msg.pose.orientation,quat_tf);
    tf::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    Vector6d x_c, x_g;
    x_c<<msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,roll,pitch,yaw;
    x_g=inv_fn_hx(x_c,Extrinsic);

    for(int i=0;i<3;i++){
        cam1_poses[3+i]=x_g(i);
    }
  }

  void c1_m3_pose_callback(const geometry_msgs::PoseStamped &msg){
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(msg.pose.orientation,quat_tf);
    tf::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    Vector6d x_c, x_g;
    x_c<<msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,roll,pitch,yaw;
    x_g=inv_fn_hx(x_c,Extrinsic);

    for(int i=0;i<3;i++){
        cam1_poses[2*3+i]=x_g(i);
    }
  }

   void c2_m1_pose_callback(const geometry_msgs::PoseStamped &msg){
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(msg.pose.orientation,quat_tf);
    tf::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    Vector6d x_c, x_g;
    x_c<<msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,roll,pitch,yaw;
    x_g=inv_fn_hx(x_c,Extrinsic);

    for(int i=0;i<3;i++){
        cam2_poses[i]=x_g(i);
    }
  }

  void c2_m2_pose_callback(const geometry_msgs::PoseStamped &msg){
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(msg.pose.orientation,quat_tf);
    tf::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    Vector6d x_c, x_g;
    x_c<<msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,roll,pitch,yaw;
    x_g=inv_fn_hx(x_c,Extrinsic);

    for(int i=0;i<3;i++){
        cam2_poses[3+i]=x_g(i);
    }
  }

  void c2_m3_pose_callback(const geometry_msgs::PoseStamped &msg){
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(msg.pose.orientation,quat_tf);
    tf::Matrix3x3 m(quat_tf);
    double roll, pitch, yaw;
    m.getEulerYPR(yaw, pitch, roll);

    Vector6d x_c, x_g;
    x_c<<msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,roll,pitch,yaw;
    x_g=inv_fn_hx(x_c,Extrinsic);

    for(int i=0;i<3;i++){
        cam2_poses[2*3+i]=x_g(i);
    }
  }
  

  Vector6d inv_fn_hx(Vector6d m, double Extrinsic[16]){
    Vector6d x;
    Eigen::Matrix<double,6, 12> hh;
    
    Matrix4d T_m, T_c, T_d, T_x, T_c_i;
    
    Vector6d d;
    d<<drone_pose[0], drone_pose[1],drone_pose[2],drone_pose[3],drone_pose[4],drone_pose[5];
    

    T_c<<Extrinsic[0],Extrinsic[1],Extrinsic[2],Extrinsic[3]
      ,Extrinsic[4],Extrinsic[5],Extrinsic[6],Extrinsic[7]
      ,Extrinsic[8],Extrinsic[9],Extrinsic[10],Extrinsic[11]
      ,Extrinsic[12],Extrinsic[13],Extrinsic[14],Extrinsic[15];

    T_m=make_TF(m);
    T_d=make_TF(d);
    
    T_c_i=inverse_TF(T_c);
    //T_x=T_d*T_c_i*T_m;
    T_x=T_c_i*T_m;
    x=TF_to_vec(T_x);


    return x;
  }

  Matrix4d inverse_TF(Matrix4d T){
    Matrix4d T_i;
    Matrix3d R;
    Vector3d t;
    R=T.topLeftCorner(3,3);
    t<<T(0,3),T(1,3),T(2,3);
    T_i=Matrix4d::Zero();
    T_i(3,3)=1;
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
  


ros::init(argc, argv, "RL_topic_pub");

topic_publisher TP;
 
  ros::spin();
  

 
  
  return 0;
}