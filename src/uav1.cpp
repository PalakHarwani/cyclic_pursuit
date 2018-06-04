/**
* @file website_node.cpp
* @brief offboard example node, written with mavros version 0.14.2, px4 flight
* stack and tested in Gazebo SITL
*/

#include <ros/ros.h>        
#include <geometry_msgs/PoseStamped.h> 
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandTOL.h>
#include <math.h>
#include <iostream>
//#include <time.h>
//#include <px4_mavros_fly/comm_data.h>
#include <stdlib.h> 
//#include <px4_mavros_fly/teleop_msg.h>

mavros_msgs::State current_state;
sensor_msgs::Joy joyvar;
geometry_msgs::PoseStamped quad_pose;
geometry_msgs::PoseStamped quad_foll_pose;
geometry_msgs::PoseStamped quad_dest;
//px4_mavros_fly::comm_data comm_tx, comm_rx;
//px4_mavros_fly::teleop_msg teleop_rec_msg;

#define X 0
#define Y 4
#define zo 1
#define ID "1"
#define IDfoll "2"


float V=0.5;
double alpha_max = 1;
float k;
float rho;





int UAV_id=atoi(ID);
int foll_id=atoi(IDfoll);


float x_quad,y_quad, z_quad;
float x_desired, y_desired;
float x_wp,y_wp,z_wp,xt,yt;
double alpha_desired;
double alpha = 0;

struct state 
{
  float x;
  float y;
  double a;
};


int way_point_reached (void)
{ 
  float d= sqrt((x_wp-x_quad)*(x_wp-x_quad)+(y_wp-y_quad)*(y_wp-y_quad)+(z_wp-z_quad)*(z_wp-z_quad)); 
  if(d<0.15)
    return 1;
  else 
    return 0;
}


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}


void pose_fb_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
          
  quad_pose = *msg;
}

void pose2_fb_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{    
           
  quad_foll_pose = *msg;
}

void get_pose_desired(const geometry_msgs::PoseStamped quad_foll_pose)
{
  x_desired = rho*(quad_foll_pose.pose.position.x) + (1 - rho)*(X);
  y_desired = rho*(quad_foll_pose.pose.position.y) + (1 - rho)*(Y);
}



double get_alpha_desired(const geometry_msgs::PoseStamped quad_pose)
{
  double alphad;
  alphad = atan2((y_desired - quad_pose.pose.position.y), (x_desired - quad_pose.pose.position.x));
  return alphad;

}

state fdot(float x0, float y0, double a0)
{
  state control;
  control.x = V*cos(a0);
  control.y = V*sin(a0);
  control.a = k*(alpha_desired - a0);
  return control;
}

state rk4(float x0, float y0, float a0,float h)
{

  state NextSt ;
  state k1, k2, k3, k4 ;
  //h == dt;
  k1 = fdot(x0, y0, a0);
  k2 = fdot(x0 + 0.5*h*k1.x, y0 + 0.5*h*k1.y, a0 + 0.5*h*k1.a);
  k3 = fdot(x0 + 0.5*h*k2.x, y0 + 0.5*h*k2.y, a0 + 0.5*h*k2.a);
  k4 = fdot(x0 + h*k3.x, y0 + h*k3.y, a0 + h*k3.a);
  NextSt.x = x0 + (1.0/6.0)*(k1.x + 2*k2.x + 2*k3.x + k4.x);
  NextSt.y = y0 + (1.0/6.0)*(k1.y + 2*k2.y + 2*k3.y + k4.y);
  NextSt.a = a0 + (1.0/6.0)*(k1.a + 2*k2.a + 2*k3.a + k4.a);
  return NextSt;
}

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "uav"ID"");
  ros::NodeHandle nh;

  k=std::atof(argv[1]);
  rho=std::atof(argv[2]);
  
  state NextSt;
  float h = 0.1;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("uav"ID "/mavros/state", 10, state_cb);
  ros::Subscriber pose_fb_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav"ID"/mavros/local_position/pose", 10, pose_fb_cb);
  ros::Subscriber pose2_fb_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav"IDfoll"/mavros/local_position/pose", 10, pose2_fb_cb);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("uav"ID"/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("uav"ID"/mavros/set_mode");
  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("uav"ID"/mavros/setpoint_velocity/cmd_vel", 10);
  //ros::Publisher comm_pub = nh.advertise<px4_mavros_fly::comm_data>//("uav"ID"/comm_data", 20);        
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("uav"ID"/mavros/setpoint_position/local", 10);
  ros::ServiceClient  land_client = nh.serviceClient<mavros_msgs::CommandTOL>("uav"ID"/mavros/cmd/land");
  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(10.0);
  // wait for FCU connection
  while(ros::ok() && current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped pose_cmd;
  geometry_msgs::TwistStamped vel_cmd;
  //  pose.pose.position.x = 0;
  // pose.pose.position.y = 0;
  // pose.pose.position.z = 1;

  vel_cmd.twist.linear.x = 0;
  vel_cmd.twist.linear.y = 0;
  vel_cmd.twist.linear.z = 0;
  //send a few setpoints before starting
  for(int i = 50; ros::ok() && i > 0; --i){
  local_vel_pub.publish(vel_cmd);
  ros::spinOnce();
  rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandTOL land;
  mavros_msgs::CommandBool arm_cmd;

  ros::Time last_request = ros::Time::now();

  quad_dest.pose.position.x = quad_pose.pose.position.x;
  quad_dest.pose.position.y = quad_pose.pose.position.y;
  quad_dest.pose.position.z = zo;


  while(ros::ok())
  {
    if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } 
    else 
    {
      if( !current_state.armed &&
      (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        arm_cmd.request.value = true;
        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    get_pose_desired(quad_foll_pose);
    alpha_desired = get_alpha_desired(quad_pose);


    
   // NextSt = rk4(quad_dest.pose.position.x, quad_dest.pose.position.y, alpha, 0.1);                       //runge-kutta
    
    NextSt.x = quad_dest.pose.position.x + h*V*cos(alpha);                                                 //Euler
    NextSt.y = quad_dest.pose.position.y + h*V*sin(alpha);
    NextSt.a = alpha + h*k*(alpha_desired - alpha);

    quad_dest.pose.position.x = NextSt.x;
    quad_dest.pose.position.y = NextSt.y;
    quad_dest.pose.position.z = zo;
    alpha = NextSt.a;
    local_pos_pub.publish(quad_dest);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}