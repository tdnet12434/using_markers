/*
   Copyright (c) 2010, Willow Garage, Inc.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

 *     * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
         contributors may be used to endorse or promote products derived from
         this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_fusion_comm/DoubleArrayStamped.h"
#include "beginner_tutorials/Status.h"
//-----------interactive control
#include <interactive_markers/interactive_marker_server.h>
#include <sstream>
//-----tf transform------------
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
//-----mavros
#include <mavros_msgs/PositionTarget.h>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

sensor_msgs::Imu imu;
sensor_msgs::LaserScan scan;
geometry_msgs::PoseStamped p_goal, p_goalf;
geometry_msgs::PoseWithCovarianceStamped visual_val,vision_before_correction, gps_before_correction;
nav_msgs::Odometry pose_gps;
nav_msgs::Odometry pose_nav;
nav_msgs::Odometry pose_quad;
mavros_msgs::PositionTarget veld_val;
nav_msgs::Path q_path;

beginner_tutorials::Status status_val;
// sensor_fusion_comm::DoubleArrayStamped state_out_msf;
enum {
  px=0,
  py,
  pz,
  vx,
  vy,
  vz,
  q0,
  q1,
  q2,
  q3,
  b_wx,
  b_wy,
  b_wz,
  b_ax,
  b_ay,
  b_az,
  L,
  qwv0,
  qwv1,
  qwv2,
  qwv3,
  pwvx,
  pwvy,
  pwvz,
  qic0,
  qic1,
  qic2,
  qic3,
  picx,
  picy,
  picz,
  pipx,
  pipy,
  pipz,
  bp,
  sizestate
};

// 0 1 2 //p
// 3 4 5 //v
// 6 7 8 9 //q
// 10 11 12 //b_w
// 13 14 15 //b_a
// 16  //L
// 17 18 19 20 //q_wv
// 21 22 23 //p_wv
// 24 25 26 27 //q_ic
// 28 29 30 //p_ic
// 31 32 33 //p_ip
// 34 //b_p
sensor_fusion_comm::DoubleArrayStamped state_msf;

float scale_msf = 0;
float pwv_msf[3];

float vx_ef = 0, vy_ef = 0;


//test-----tf transform------------

void poseCallback(const geometry_msgs::Pose& msg, std::string turtle_name) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  //q.setRPY(0, 0, msg->theta);    //set quaternion q from RPY

  transform.setOrigin( tf::Vector3(msg.position.x, msg.position.y, msg.position.z) );
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", turtle_name));
}





double roll_, pitch_, yaw_ondemand;
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define wrap_pi(x) (x < -3.14 ? x+6.28 : (x > 3.14 ? x - 6.28: x))
#define HALF_M_PI 1.570796







double roll, pitch, yaw;



void imucallback(const sensor_msgs::Imu::ConstPtr &data)
{
  // imu = *data;
  imu = *data;
  tf::Quaternion q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  //ROS_INFO("ax:[%f]", yaw);
}
void pose_gps_callback(const nav_msgs::Odometry::ConstPtr& data)
{
  pose_gps = *data;
  // ROS_INFO("pz:[%f]", data.pose.pose.position.z);
  gps_before_correction.header = pose_gps.header;
  gps_before_correction.pose.pose.position.x = pose_gps.pose.pose.position.x + status_val.GPS_OFFSET.x;
  gps_before_correction.pose.pose.position.y = pose_gps.pose.pose.position.y + status_val.GPS_OFFSET.y;
  gps_before_correction.pose.pose.orientation = imu.orientation;
  gps_before_correction.pose.covariance=pose_gps.pose.covariance;
}
void pose_nav_callback(const nav_msgs::Odometry::ConstPtr& data)
{
  pose_nav = *data;
  // ROS_INFO("pose_nav_pz:[%f]", data.pose.pose.position.z);
}
void pose_quad_callback(const nav_msgs::Odometry::ConstPtr& data)
{
  pose_quad = *data;
  // ROS_INFO("pose_nav_pz:[%f]", data.pose.pose.position.z);
}
void scancallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
  scan = *data;

  //ROS_INFO("sx:[%f]", data.ranges[256]);
}
void vel_des_callback(const mavros_msgs::PositionTarget::ConstPtr& data) {
  veld_val = *data;
}
void visual_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data) {
  visual_val = *data;
  vision_before_correction = visual_val;

  vision_before_correction.pose.pose.position.x=vision_before_correction.pose.pose.position.x/scale_msf+pwv_msf[0];
  vision_before_correction.pose.pose.position.y=vision_before_correction.pose.pose.position.y/scale_msf+pwv_msf[1];
  vision_before_correction.pose.pose.position.z=vision_before_correction.pose.pose.position.z/scale_msf+pwv_msf[2];
  

}
void state_out_callback(const sensor_fusion_comm::DoubleArrayStamped::ConstPtr& data) {
  static bool initvar=false;
  if(!initvar) { initvar=true;state_msf.data.resize(sizestate);}
  state_msf.data = data->data;
  scale_msf = state_msf.data[L];
  pwv_msf[0] = state_msf.data[pwvx];
  pwv_msf[1] = state_msf.data[pwvy];
  pwv_msf[2] = state_msf.data[pwvz];
}

void status_callback(const beginner_tutorials::Status::ConstPtr& data) {
  status_val = *data;
}

visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.scale * 0.35;
  marker.scale.y = msg.scale * 0.35;
  marker.scale.z = msg.scale * 5;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}
// void makeButtonMarker( const tf::Vector3& position )
// {
//   visualization_msgs::InteractiveMarker int_marker;
//   int_marker.header.frame_id = "base_link";
//   tf::pointTFToMsg(position, int_marker.pose.position);
//   int_marker.scale = 1;

//   int_marker.name = "button";
//   int_marker.description = "Button\n(Left Click)";

//   visualization_msgs::InteractiveMarkerControl control;

//   control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
//   control.name = "button_control";

//   visualization_msgs::Marker marker = makeBox( int_marker );
//   control.markers.push_back( marker );
//   control.always_visible = true;
//   int_marker.controls.push_back(control);

//   server->insert(int_marker);
//   server->setCallback(int_marker.name, &processFeedback);
// }
void init_msf_filter() {
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::BoolParameter bool_param;
  dynamic_reconfigure::Config conf;

  bool_param.name = "core_init_filter";
  bool_param.value = true;
  conf.bools.push_back(bool_param);

  bool_param.name = "reset_coordinate";
  bool_param.value = true;
  conf.bools.push_back(bool_param);

  srv_req.config = conf;

  ros::service::call("/pose_from_compass_position_gps/position_pose_pressure_sensor/set_parameters", srv_req, srv_resp);
}



void processFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  p_goal.pose.position = feedback->pose.position;


  /*tf::Quaternion q(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_ondemand);
  // p_goal.orientation.x = feedback->pose.orientation.x;
  // p_goal.orientation.y = feedback->pose.orientation.y;
  // p_goal.orientation.z = feedback->pose.orientation.z;
  // p_goal.orientation.w = feedback->pose.orientation.w;

  p_goal.pose.orientation.w = wrap_pi(yaw_ondemand - HALF_M_PI);*/

  p_goal.pose.orientation = feedback->pose.orientation;


  //ROS_INFO("%f",p_goal.pose.position.x);
  // ROS_INFO_STREAM("position is " << p_goal.pose.position.x << "," << p_goal.pose.position.y "," << p_goal.pose.position.z "and yaw = " << yaw_ondemand);
  // ROS_INFO_STREAM( feedback->marker_name << " is now at "
  //     << feedback->pose.position.x << ", " << feedback->pose.position.y
  //     << ", " << feedback->pose.position.z );
}
void buttonprocessFeedback2(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
    init_msf_filter();
  }
}
//--------------------------



double dist(double dx,double dy) {
  return sqrt(dx*dx+dy*dy);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(100);
  ros::Publisher  marker_pub =      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber imu_sub =         n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 2, imucallback);
  ros::Subscriber pos_gps_sub =     n.subscribe<nav_msgs::Odometry>("/imu_max/poseodo", 2, pose_gps_callback);
  ros::Subscriber pos_msf_sub =     n.subscribe<nav_msgs::Odometry>("/msf_core/odometry", 2, pose_nav_callback);
  ros::Subscriber pos_quad_sub =    n.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 2, pose_quad_callback);
  // ros::Subscriber scan_sub =        n.subscribe<sensor_msgs::LaserScan>("/scan", 2, scancallback);
  ros::Subscriber veld_sub =        n.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 2, vel_des_callback);
  ros::Publisher  pose_demand_pub = n.advertise<geometry_msgs::PoseStamped>("pose_demand", 1);
  ros::Publisher  q_path_pub =      n.advertise<nav_msgs::Path>("quad_path", 1);
  ros::Subscriber visual_sub =      n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/odom_pose_filtered", 2, visual_callback);
  ros::Subscriber state_out_msf_sub = n.subscribe<sensor_fusion_comm::DoubleArrayStamped>("/msf_core/state_out", 2, state_out_callback);
  ros::Subscriber status_sub =      n.subscribe<beginner_tutorials::Status>("/imu_max/status", 2, status_callback);
  ros::Publisher odom_pub =         n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/vision_before_correction", 10);
  ros::Publisher  gps_before_correction_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/gps_before_correction", 10);
  //test-----------------------------------------------------------------------
  interactive_markers::InteractiveMarkerServer server("GOAL_AND_HEADING_CONTROL");
  interactive_markers::InteractiveMarkerServer server2("INIT_FILTER");
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/odom";
  int_marker.name = "Quad_goal";
  int_marker.description = "Goal";
  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  box_marker.mesh_resource = "package://using_markers/src/meshes/quadrotor_2.stl";
  box_marker.mesh_use_embedded_materials = 1;

  box_marker.scale.x = 0.3;
  box_marker.scale.y = 0.3;
  box_marker.scale.z = 0.3;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  //------------------------------
  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.orientation.w = 1;
  rotate_control.orientation.x = 0;
  rotate_control.orientation.y = 1;
  rotate_control.orientation.z = 0;
  rotate_control.name = "move_x";
  rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(rotate_control);
  rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  // server.insert(int_marker, &processFeedback);

  tf::Vector3 position;
  position = tf::Vector3( 0, -2, 5);
  visualization_msgs::InteractiveMarker int_marker2;
  int_marker2.header.frame_id = "odom";
  tf::pointTFToMsg(position, int_marker2.pose.position);
  int_marker2.scale = 1;

  int_marker2.name = "button";
  int_marker2.description = "init msf filter\n(Left Click)";

  visualization_msgs::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  visualization_msgs::Marker marker2 = makeBox( int_marker2 );
  control.markers.push_back( marker2 );
  control.always_visible = true;
  int_marker2.controls.push_back(control);

  server.insert(int_marker, &processFeedback);
  // server.setCallback(int_marker.name, &processFeedback);
  server2.insert(int_marker2, &buttonprocessFeedback2);
  // 'commit' changes and send to all clients
  server.applyChanges();
  server2.applyChanges();
  //----------------------------------------------------------------------


  //test------------------------------------------------------------------






  // Set our initial shape type to be a cube
  //  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/odom";
  marker.ns = "quad_model";
  marker.id = 0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = 0.5f;
  marker.color.g = 0.5f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://using_markers/src/meshes/quadrotor_2.stl";
  marker.mesh_use_embedded_materials = 1;

  visualization_msgs::Marker home_marker;
  home_marker.header.frame_id = "/odom";
  home_marker.ns = "gps_home";
  home_marker.id = 1;
  home_marker.scale.x = 0.2;
  home_marker.scale.y = 0.2;
  home_marker.scale.z = 0.01;
  home_marker.color.r = 0.5f;
  home_marker.color.g = 0.5f;
  home_marker.color.b = 0.0f;
  home_marker.color.a = 1.0;
  home_marker.pose.orientation.x = 0;
  home_marker.pose.orientation.y = 0;
  home_marker.pose.orientation.z = 0;
  home_marker.pose.orientation.w = 1;
  home_marker.type = visualization_msgs::Marker::CUBE;


  visualization_msgs::Marker nav_path, gps_path;
  nav_path.header.frame_id = "/odom";
  nav_path.ns = "nav_path";
  nav_path.id = 9;
  nav_path.type = visualization_msgs::Marker::LINE_STRIP;
  nav_path.scale.x = 0.1;
  nav_path.color.b = 1.0;
  nav_path.color.a = 1.0;

  gps_path.header.frame_id = "/odom";
  gps_path.ns = "gps_path";
  gps_path.id = 3;
  gps_path.type = visualization_msgs::Marker::LINE_STRIP;
  gps_path.scale.x = 0.1;
  gps_path.color.g = 1.0;
  gps_path.color.a = 1.0;



  visualization_msgs::Marker vel_des_maker;
  vel_des_maker.header.frame_id = "/odom";
  vel_des_maker.type = visualization_msgs::Marker::ARROW;
  vel_des_maker.ns = "vel_des";
  vel_des_maker.id = 4;
  vel_des_maker.scale.x = 1.0;
  vel_des_maker.scale.y = 0.03;
  vel_des_maker.scale.z = 0.03;
  vel_des_maker.color.r = 0.0f;
  vel_des_maker.color.g = 1.0f;
  vel_des_maker.color.b = 0.0f;
  vel_des_maker.color.a = 1.0;

  vel_des_maker.pose.orientation.x = 0;
  vel_des_maker.pose.orientation.y = 0;
  vel_des_maker.pose.orientation.z = 0;
  vel_des_maker.pose.orientation.w = 1;

  visualization_msgs::Marker vel_nav_marker;
  vel_nav_marker.header.frame_id = "/vel_nav_bf";
  vel_nav_marker.type = visualization_msgs::Marker::ARROW;
  vel_nav_marker.ns = "vel_nav_bf";
  vel_nav_marker.id = 5;
  vel_nav_marker.scale.x = 1.0;
  vel_nav_marker.scale.y = 0.03;
  vel_nav_marker.scale.z = 0.03;
  vel_nav_marker.color.r = 1.0f;
  vel_nav_marker.color.g = 0.0f;
  vel_nav_marker.color.b = 0.0f;
  vel_nav_marker.color.a = 1.0;

  vel_nav_marker.pose.orientation.x = 0;
  vel_nav_marker.pose.orientation.y = 0;
  vel_nav_marker.pose.orientation.z = 0;
  vel_nav_marker.pose.orientation.w = 1;

  visualization_msgs::Marker vel_quad_marker;
  vel_quad_marker.header.frame_id = "/vel_quad_bf";
  vel_quad_marker.type = visualization_msgs::Marker::ARROW;
  vel_quad_marker.ns = "vel_quad";
  vel_quad_marker.id = 6;
  vel_quad_marker.scale.x = 1.0;
  vel_quad_marker.scale.y = 0.03;
  vel_quad_marker.scale.z = 0.03;
  vel_quad_marker.color.r = 1.0f;
  vel_quad_marker.color.g = 0.0f;
  vel_quad_marker.color.b = 0.0f;
  vel_quad_marker.color.a = 1.0;

  vel_quad_marker.pose.orientation.x = 0;
  vel_quad_marker.pose.orientation.y = 0;
  vel_quad_marker.pose.orientation.z = 0;
  vel_quad_marker.pose.orientation.w = 1;

  visualization_msgs::Marker goal_box;
  goal_box.header.frame_id = "/odom";
  goal_box.type = visualization_msgs::Marker::SPHERE;
  goal_box.ns = "pose_des";
  goal_box.id = 7;
  goal_box.scale.x = 0.05;
  goal_box.scale.y = 0.05;
  goal_box.scale.z = 0.05;
  goal_box.color.r = 1.0f;
  goal_box.color.g = 0.0f;
  goal_box.color.b = 0.0f;
  goal_box.color.a = 1.0;

  goal_box.pose.orientation.x = 0;
  goal_box.pose.orientation.y = 0;
  goal_box.pose.orientation.z = 0;
  goal_box.pose.orientation.w = 1;


  visualization_msgs::Marker nav_pose_text;
  nav_pose_text.header.frame_id = "/odom";
  nav_pose_text.ns = "nav_text";
  nav_pose_text.id = 8;
  nav_pose_text.scale.z = 0.5;
  nav_pose_text.color.r = 0.0f;
  nav_pose_text.color.g = 0.0f;
  nav_pose_text.color.b = 1.0f;
  nav_pose_text.color.a = 1.0;
  nav_pose_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  // gps_text.header.frame_id = "/odom";
  // gps_text.ns = "gps_text";
  // gps_text.id = 1;
  // gps_text.scale.z = 0.5;
  // gps_text.color.r = 0.0f;
  // gps_text.color.g = 1.0f;
  // gps_text.color.b = 0.0f;
  // gps_text.color.a = 1.0;
  // gps_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // std::string aaa = "POSITION :";
  // text.text= aaa;


  float f = 0.0;
  while (ros::ok())
  {
    //ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("imu_subscribe", 1000, imucallback);

    ros::Time cur_time = ros::Time::now();

    marker.header.stamp = cur_time;
    marker.action = visualization_msgs::Marker::ADD;

    home_marker.header.stamp = cur_time;
    home_marker.action = visualization_msgs::Marker::ADD;

    

    gps_path.header.stamp = cur_time;
    gps_path.action = visualization_msgs::Marker::ADD;


    //int_marker.action = visualization_msgs::Marker::ADD;


    



    geometry_msgs::Point p, p2;
    for (uint32_t i = 0; i < 1000; ++i)
    {

      if (!ros::ok()) break; //return when ros is not ok (shuting down)
      
      static int i_p = 0;
      static int i_p2= 0;

      if(dist(p.x - pose_nav.pose.pose.position.x,
              p.y - pose_nav.pose.pose.position.y) 
              > 0.3) 
      {
        i_p++;

        nav_path.header.stamp = cur_time;
        nav_path.action = visualization_msgs::Marker::ADD;

        p.x = pose_nav.pose.pose.position.x ;  //cm to m
        p.y = pose_nav.pose.pose.position.y ;
        p.z = pose_nav.pose.pose.position.z ;
        nav_path.points.push_back(p);
        if(i_p > 1000) { i_p = 0; nav_path.points.clear();}

        //if state change so new id
        static bool old_status = status_val.GPS_IN_USING;
        if(old_status!=status_val.GPS_IN_USING)
         { nav_path.id = nav_path.id+1;
          printf("changed color line\n");
          old_status = status_val.GPS_IN_USING;
          
          if(status_val.GPS_IN_USING) {
            nav_path.color.b = 1.0;
            nav_path.color.r = 0.0;
          }else{
            nav_path.color.b = 0.0;
            nav_path.color.r = 1.0;
          }
        }
      }
      marker_pub.publish(nav_path);

      if(dist(p2.x - pose_gps.pose.pose.position.x,
              p2.y - pose_gps.pose.pose.position.y) 
              > 0.3) 
      {
        i_p2++;
        p2.x = pose_gps.pose.pose.position.x ;  //cm to m
        p2.y = pose_gps.pose.pose.position.y ;
        p2.z = pose_gps.pose.pose.position.z ;
        gps_path.points.push_back(p2);
        if(i_p2 > 1000) { i_p2 = 0; gps_path.points.clear();}
      }

      marker_pub.publish(gps_path);

      marker.pose.position.x = pose_quad.pose.pose.position.x ;
      marker.pose.position.y = pose_quad.pose.pose.position.y ;
      marker.pose.position.z = pose_quad.pose.pose.position.z ;
      marker.pose.orientation  = imu.orientation;
      marker_pub.publish(marker);
      

      

      /////////////////PATH/////////////////////////////////////////
      q_path.header.stamp = ros::Time::now();
      q_path.header.frame_id = "/odom";

      geometry_msgs::PoseStamped q_path_input;
      q_path_input.pose.position.x = pose_nav.pose.pose.position.x;
      q_path_input.pose.position.y = pose_nav.pose.pose.position.y;
      q_path_input.pose.position.z = pose_nav.pose.pose.position.z;

      q_path.poses.push_back(q_path_input);

      pose_demand_pub.publish(p_goal);
      q_path_pub.publish(q_path);


      ////////////////////set the desired velocity arrow/////////////////////
      float vx = veld_val.velocity.x;
      float vy = veld_val.velocity.y;

      tf::Quaternion odom_quat;
      odom_quat.setRPY(0, 0, atan2(vy, vx));

      float v_lenght = sqrt(vy * vy + vx * vx);
      if (!v_lenght) v_lenght = 0.001;
      vel_des_maker.scale.x = v_lenght;
      vel_des_maker.pose.orientation.x = odom_quat[0];
      vel_des_maker.pose.orientation.y = odom_quat[1];
      vel_des_maker.pose.orientation.z = odom_quat[2];
      vel_des_maker.pose.orientation.w = odom_quat[3];
      vel_des_maker.pose.position.x = pose_quad.pose.pose.position.x;
      vel_des_maker.pose.position.y = pose_quad.pose.pose.position.y;
      vel_des_maker.pose.position.z = pose_quad.pose.pose.position.z;
      marker_pub.publish(vel_des_maker);
      goal_box.pose.position.x = veld_val.position.x;
      goal_box.pose.position.y = veld_val.position.y;
      goal_box.pose.position.z = veld_val.position.z;
      marker_pub.publish(goal_box);

      ////////////////////set the velocity arrow////////////
      vx = pose_nav.twist.twist.linear.x;
      vy = pose_nav.twist.twist.linear.y;
      // ROS_INFO("x:[%f] y:[%f]", vx, vy);
      odom_quat.setRPY(0, 0, atan2(vy, vx));
      v_lenght = sqrt(vy * vy + vx * vx);
      if (!v_lenght) v_lenght = 0.001;
      vel_nav_marker.scale.x = v_lenght;
      vel_nav_marker.pose.orientation.x = odom_quat[0];
      vel_nav_marker.pose.orientation.y = odom_quat[1];
      vel_nav_marker.pose.orientation.z = odom_quat[2];
      vel_nav_marker.pose.orientation.w = odom_quat[3];

      marker_pub.publish(vel_nav_marker);

      ////////////////////set the velocity arrow////////////
      vx = pose_quad.twist.twist.linear.x;
      vy = pose_quad.twist.twist.linear.y;
      // ROS_INFO("x:[%f] y:[%f]", vx, vy);
      odom_quat.setRPY(0, 0, atan2(vy, vx));
      v_lenght = sqrt(vy * vy + vx * vx);
      if (!v_lenght) v_lenght = 0.001;
      vel_quad_marker.scale.x = v_lenght;
      vel_quad_marker.pose.orientation.x = odom_quat[0];
      vel_quad_marker.pose.orientation.y = odom_quat[1];
      vel_quad_marker.pose.orientation.z = odom_quat[2];
      vel_quad_marker.pose.orientation.w = odom_quat[3];
      marker_pub.publish(vel_quad_marker);
      





      //////////////////////////TEXT ALL WE NEED STATUS//////////////////
      std::ostringstream strs;
      strs << "FILTER : " << pose_nav.pose.pose.position.x << "\t" << pose_nav.pose.pose.position.y  << "\t" << pose_nav.pose.pose.position.z
           << "\nGPS    : " << pose_gps.pose.pose.position.x << "\t" << pose_gps.pose.pose.position.y
           << "\tHACC : " << pose_gps.pose.covariance[0]
           << "\nVISUAL : " << visual_val.pose.pose.position.x << "\t" << visual_val.pose.pose.position.y << "\tcov " << visual_val.pose.covariance[0]
           << "\nMSF_SCALE : " << scale_msf
           << "\nGPS IN USE : " << (status_val.GPS_IN_USING ? "1":"0") << "\tVISION IN USE : " << (status_val.VISION_IN_USING ? "1":"0");
      std::string aaa = strs.str();
      nav_pose_text.text = aaa;
      nav_pose_text.pose.position.x = pose_nav.pose.pose.position.x;
      nav_pose_text.pose.position.y = pose_nav.pose.pose.position.y + 2;


      marker_pub.publish(nav_pose_text);

      /////////////////////GPS OFFSET HOME NOW/////////////////////////
      static ros::Time last_status_stamp = cur_time;  
      if(status_val.header.stamp!=last_status_stamp) {   //only apply when new data come
        last_status_stamp = status_val.header.stamp;
        home_marker.pose.position = status_val.GPS_OFFSET;
        marker_pub.publish(home_marker);
      }


      /////////////////////BUTTOM INIT FILTER/////////////////////////////
      if(!std::isfinite(scale_msf)) init_msf_filter();
      //button follow quad
      int_marker2.pose.position.x = pose_nav.pose.pose.position.x;
      int_marker2.pose.position.y = pose_nav.pose.pose.position.y - 2;
      int_marker2.pose.position.z = pose_nav.pose.pose.position.z;

      int_marker2.controls.clear();
      int_marker2.controls.push_back(control);
      server2.clear();
      server2.insert(int_marker2, &buttonprocessFeedback2);
      server2.applyChanges();
      ////////////////////////////////////////////////////////////////////
      odom_pub.publish(vision_before_correction);
      gps_before_correction_pub.publish(gps_before_correction);
      ros::spinOnce();
      r.sleep();  // config at rate 10 hz




      //public to tf
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      tf::Quaternion q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
      transform.setOrigin( tf::Vector3(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z) );
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link3"));

      // //public to tf
      // static tf::TransformBroadcaster br2;
      tf::Transform transform2;
      tf::Quaternion q2(0, 0, 0, 1);
      transform2.setOrigin( tf::Vector3(0, 0, 0) );
      transform2.setRotation(q2);
      br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "state", "vel_nav_bf"));

      // //public to tf
      // static tf::TransformBroadcaster br3;
      br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link3", "vel_quad_bf"));


      




      

    }
    q_path.poses.clear();

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    /*marker.pose.position.x = 1;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;*/
    /*marker.pose.orientation.x = imu.orientation.x;
      marker.pose.orientation.y = imu.orientation.y;
      marker.pose.orientation.z = imu.orientation.z;
      marker.pose.orientation.w = imu.orientation.w;*/



    marker.lifetime = ros::Duration();
  }
  // %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
