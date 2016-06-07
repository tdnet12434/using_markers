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
//test-----------interactive control
#include <interactive_markers/interactive_marker_server.h>
#include <sstream>
//test-----tf transform------------
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

sensor_msgs::Imu imu;
sensor_msgs::LaserScan scan;
geometry_msgs::PoseStamped p_goal, p_goalf;
geometry_msgs::PoseWithCovarianceStamped visual_val,visual_val_scaled;
nav_msgs::Odometry pose_gps;
nav_msgs::Odometry pose_nav;
nav_msgs::Odometry pose_quad;
nav_msgs::Odometry veld_val;
nav_msgs::Path q_path;

beginner_tutorials::Status status_val;
// sensor_fusion_comm::DoubleArrayStamped state_out_msf;
float scale_msf = 0;


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
void vel_des_callback(const nav_msgs::Odometry::ConstPtr& data) {
  veld_val = *data;
}
void visual_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data) {
  visual_val = *data;
  visual_val_scaled = visual_val;

  visual_val_scaled.pose.pose.position.x/=scale_msf;
  visual_val_scaled.pose.pose.position.y/=scale_msf;
  visual_val_scaled.pose.pose.position.z/=scale_msf;
  

}
void state_out_callback(const sensor_fusion_comm::DoubleArrayStamped::ConstPtr& data) {

  scale_msf = data->data[16];
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
  marker.scale.z = msg.scale * 0.35;
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

  ros::service::call("/pose_from_compass_position_gps/position_pose_sensor/set_parameters", srv_req, srv_resp);
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





int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(100);
  ros::Publisher  marker_pub =      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber imu_sub =         n.subscribe<sensor_msgs::Imu>("/imu_max", 2, imucallback);
  ros::Subscriber pos_gps_sub =     n.subscribe<nav_msgs::Odometry>("/imu_max/poseodo", 2, pose_gps_callback);
  ros::Subscriber pos_msf_sub =     n.subscribe<nav_msgs::Odometry>("/msf_core/odometry", 2, pose_nav_callback);
  ros::Subscriber pos_quad_sub =    n.subscribe<nav_msgs::Odometry>("/imu_max/pose_nav", 2, pose_quad_callback);
  ros::Subscriber scan_sub =        n.subscribe<sensor_msgs::LaserScan>("/scan", 2, scancallback);
  ros::Subscriber veld_sub =        n.subscribe<nav_msgs::Odometry>("/imu_max/pose_des", 2, vel_des_callback);
  ros::Publisher  pose_demand_pub = n.advertise<geometry_msgs::PoseStamped>("pose_demand", 1);
  ros::Publisher  q_path_pub =      n.advertise<nav_msgs::Path>("quad_path", 1);
  ros::Subscriber visual_sub =      n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/odom_pose_filtered", 2, visual_callback);
  ros::Subscriber state_out_msf_sub = n.subscribe<sensor_fusion_comm::DoubleArrayStamped>("/msf_core/state_out", 2, state_out_callback);
  ros::Subscriber status_sub =      n.subscribe<beginner_tutorials::Status>("/imu_max/status", 2, status_callback);
  ros::Publisher odom_pub =         n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/slam_msf", 10);

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
  box_marker.mesh_resource = "file:///home/intel/catkin_ws/src/using_markers/src/meshes/quadrotor_2.stl";
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
  position = tf::Vector3( 0, -2, 0);
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
  marker.mesh_resource = "file:///home/intel/catkin_ws/src/using_markers/src/meshes/quadrotor_2.stl";
  marker.mesh_use_embedded_materials = 1;

  visualization_msgs::Marker marker_home;
  marker_home.header.frame_id = "/odom";
  marker_home.ns = "gps_home";
  marker_home.id = 0;
  marker_home.scale.x = 0.2;
  marker_home.scale.y = 0.2;
  marker_home.scale.z = 0.01;
  marker_home.color.r = 0.5f;
  marker_home.color.g = 0.5f;
  marker_home.color.b = 0.0f;
  marker_home.color.a = 1.0;
  marker_home.pose.orientation.x = 0;
  marker_home.pose.orientation.y = 0;
  marker_home.pose.orientation.z = 0;
  marker_home.pose.orientation.w = 1;
  marker_home.type = visualization_msgs::Marker::CUBE;


  visualization_msgs::Marker line_strip, line_strip2;
  line_strip.header.frame_id = "/odom";
  line_strip.ns = "path_quad";
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.01;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  line_strip2.header.frame_id = "/odom";
  line_strip2.ns = "path_quad2";
  line_strip2.id = 2;
  line_strip2.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip2.scale.x = 0.01;
  line_strip2.color.g = 1.0;
  line_strip2.color.a = 1.0;



  visualization_msgs::Marker vel_vec;
  vel_vec.header.frame_id = "/vel_des";
  vel_vec.type = visualization_msgs::Marker::ARROW;
  vel_vec.ns = "vel_des";
  vel_vec.id = 5;
  vel_vec.scale.x = 1.0;
  vel_vec.scale.y = 0.03;
  vel_vec.scale.z = 0.03;
  vel_vec.color.r = 0.0f;
  vel_vec.color.g = 1.0f;
  vel_vec.color.b = 0.0f;
  vel_vec.color.a = 1.0;

  vel_vec.pose.orientation.x = 0;
  vel_vec.pose.orientation.y = 0;
  vel_vec.pose.orientation.z = 0;
  vel_vec.pose.orientation.w = 1;

  visualization_msgs::Marker vel_vec2;
  vel_vec2.header.frame_id = "/vel_ef";
  vel_vec2.type = visualization_msgs::Marker::ARROW;
  vel_vec2.ns = "vel_ef";
  vel_vec2.id = 6;
  vel_vec2.scale.x = 1.0;
  vel_vec2.scale.y = 0.03;
  vel_vec2.scale.z = 0.03;
  vel_vec2.color.r = 1.0f;
  vel_vec2.color.g = 0.0f;
  vel_vec2.color.b = 0.0f;
  vel_vec2.color.a = 1.0;

  vel_vec2.pose.orientation.x = 0;
  vel_vec2.pose.orientation.y = 0;
  vel_vec2.pose.orientation.z = 0;
  vel_vec2.pose.orientation.w = 1;

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
  nav_pose_text.id = 0;
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

    marker_home.header.stamp = cur_time;
    marker_home.action = visualization_msgs::Marker::ADD;

    line_strip.header.stamp = cur_time;
    line_strip.action = visualization_msgs::Marker::ADD;

    line_strip2.header.stamp = cur_time;
    line_strip2.action = visualization_msgs::Marker::ADD;


    //int_marker.action = visualization_msgs::Marker::ADD;


    line_strip.points.clear();
    line_strip2.points.clear();




    for (uint32_t i = 0; i < 1000; ++i)
    {

      if (!ros::ok()) break; //return when ros is not ok (shuting down)
      geometry_msgs::Point p, p2;
      // p.x = marker.pose.position.x;
      // p.y = marker.pose.position.y;
      // p.z = marker.pose.position.z;

      p.x = pose_nav.pose.pose.position.x ;  //cm to m
      p.y = pose_nav.pose.pose.position.y ;
      p.z = pose_nav.pose.pose.position.z ;

      p2.x = pose_gps.pose.pose.position.x ;  //cm to m
      p2.y = pose_gps.pose.pose.position.y ;
      p2.z = pose_gps.pose.pose.position.z ;


      ////////////////////set the desired velocity arrow/////////////////////
      float vx = veld_val.twist.twist.linear.x;
      float vy = veld_val.twist.twist.linear.y;

      tf::Quaternion odom_quat;
      odom_quat.setRPY(0, 0, atan2(vy, vx));

      float v_lenght = sqrt(vy * vy + vx * vx) * 10;
      if (!v_lenght) v_lenght = 0.001;
      vel_vec.scale.x = v_lenght;
      vel_vec.pose.orientation.x = odom_quat[0];
      vel_vec.pose.orientation.y = odom_quat[1];
      vel_vec.pose.orientation.z = odom_quat[2];
      vel_vec.pose.orientation.w = odom_quat[3];
      // ROS_INFO("x:[%f]", vel_vec.scale.x);
      marker_pub.publish(vel_vec);


      ////////////////////set the velocity arrow////////////
      vx = pose_nav.twist.twist.linear.x;
      vy = pose_nav.twist.twist.linear.y;
      ROS_INFO("x:[%f] y:[%f]", vx, vy);
      odom_quat.setRPY(0, 0, atan2(vy, vx));
      v_lenght = sqrt(vy * vy + vx * vx);
      if (!v_lenght) v_lenght = 0.001;
      vel_vec2.scale.x = v_lenght;
      vel_vec2.pose.orientation.x = odom_quat[0];
      vel_vec2.pose.orientation.y = odom_quat[1];
      vel_vec2.pose.orientation.z = odom_quat[2];
      vel_vec2.pose.orientation.w = odom_quat[3];

      // ROS_INFO("x:[%f]", vel_vec.scale.x);
      marker_pub.publish(vel_vec2);

      goal_box.pose.position.x = veld_val.pose.pose.position.x;
      goal_box.pose.position.y = veld_val.pose.pose.position.y;
      goal_box.pose.position.z = veld_val.pose.pose.position.z;
      marker_pub.publish(goal_box);





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
        marker_home.pose.position = status_val.GPS_OFFSET;
        marker_pub.publish(marker_home);
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
      odom_pub.publish(visual_val_scaled);
      
      ros::spinOnce();
      r.sleep();  // config at rate 10 hz

      //marker.pose.position = (p);
      // marker.pose.position.x = marker.pose.position.x*0.9+0.1*p_goal.position.x  ;
      // marker.pose.position.y = marker.pose.position.y*0.9+0.1*p_goal.position.y  ;
      // marker.pose.position.z = marker.pose.position.z*0.9+0.1*p_goal.position.z  ;
      marker.pose.position.x = pose_quad.pose.pose.position.x ;
      marker.pose.position.y = pose_quad.pose.pose.position.y ;
      marker.pose.position.z = pose_quad.pose.pose.position.z ;

      //convert to rpy then + pi/2 for rotate that wrong direct model stl
      /*tf::Quaternion q_hmt;
      q_hmt[0] = imu.orientation.x;
      q_hmt[1] = imu.orientation.y;
      q_hmt[2] = imu.orientation.z;
      q_hmt[3] = imu.orientation.w;
      tf::Matrix3x3 m(q_hmt);
      double _roll, _pitch, _yaw;
      m.getRPY(_roll, _pitch, _yaw);
      tf::Quaternion q_;
      q_.setRPY(_roll, _pitch, wrap_pi(_yaw)); //roll pitch yaw  (to set 0 degree at east "not" north)
      marker.pose.orientation.x = q_[0];
      marker.pose.orientation.y = q_[1];
      marker.pose.orientation.z = q_[2];
      marker.pose.orientation.w = q_[3];*/

      marker.pose.orientation  = imu.orientation;
      // marker.pose.orientation.x = marker.pose.orientation.x*0.9+0.1*p_goal.orientation.x;
      // marker.pose.orientation.y = marker.pose.orientation.y*0.9+0.1*p_goal.orientation.y;
      // marker.pose.orientation.z = marker.pose.orientation.z*0.9+0.1*p_goal.orientation.z;
      // marker.pose.orientation.w = marker.pose.orientation.w*0.9+0.1*p_goal.orientation.w;


      //public to tf
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      tf::Quaternion q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
      transform.setOrigin( tf::Vector3(marker.pose.position.x, marker.pose.position.y, marker.pose.position.z) );
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link3"));

      // //public to tf
      static tf::TransformBroadcaster br2;
      tf::Transform transform2;
      tf::Quaternion q2(0, 0, 0, 1);
      transform2.setOrigin( tf::Vector3(0, 0, 0) );
      transform2.setRotation(q2);
      br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link3", "vel_ef"));



      marker_pub.publish(marker);
      line_strip.points.push_back(p);
      marker_pub.publish(line_strip);

      line_strip2.points.push_back(p2);
      marker_pub.publish(line_strip2);




      q_path.header.stamp = ros::Time::now();
      q_path.header.frame_id = "/odom";

      geometry_msgs::PoseStamped q_path_input;
      q_path_input.pose.position.x = pose_nav.pose.pose.position.x;
      q_path_input.pose.position.y = pose_nav.pose.pose.position.y;
      q_path_input.pose.position.z = pose_nav.pose.pose.position.z;

      q_path.poses.push_back(q_path_input);

      pose_demand_pub.publish(p_goal);
      q_path_pub.publish(q_path);

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
