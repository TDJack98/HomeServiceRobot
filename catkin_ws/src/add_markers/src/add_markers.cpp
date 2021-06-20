#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <string>

bool picked_up = false;
bool dropped_off = false;
double pick[3] = {4.5, 2, 1};
double drop[3] = {4.5, 0, 1};

void pos_Callback(const std_msgs::String check)
{
   std::string str = check.data;
   if (str == "pickup")
   {
      picked_up = true;
   }
   else if (str == "dropoff")
   {
      dropped_off = true;
   }
}
 
int main( int argc, char** argv )
{
   ros::init(argc, argv, "basic_shapes");
   ros::NodeHandle n;
   ros::Rate r(1);
   ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

   ros::Subscriber odo_sub = n.subscribe("/posInfo", 10, pos_Callback);
 
   // Set our initial shape type to be a cube
   uint32_t shape = visualization_msgs::Marker::CUBE;
 
   while (ros::ok())
   {
     visualization_msgs::Marker marker;
     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
     marker.header.frame_id = "map";
     marker.header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     marker.ns = "basic_shapes";
     marker.id = 0;
 
     // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
     marker.type = shape;
 
     // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
     marker.action = visualization_msgs::Marker::ADD;
 
     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     marker.pose.position.x = pick[0];
     marker.pose.position.y = pick[1];
     marker.pose.position.z = 0;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = pick[2];
 
     // Set the scale of the marker -- 1x1x1 here means 1m on a side
     marker.scale.x = 0.1;
     marker.scale.y = 0.1;
     marker.scale.z = 0.1;
 
     // Set the color -- be sure to set alpha to something non-zero!
     marker.color.r = 0.0f;
     marker.color.g = 1.0f;
     marker.color.b = 0.0f;
     marker.color.a = 1.0;
 
     marker.lifetime = ros::Duration();
     marker_pub.publish(marker);

     if(picked_up)
     {
        ROS_INFO("Picking up...");
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);

        if(dropped_off)
        {
           ROS_INFO("Dropping off...");
           marker.pose.position.x = drop[0];
           marker.pose.position.y = drop[1];
           marker.pose.position.z = 0;
           marker.pose.orientation.w = drop[2];

           marker.scale.x = 0.1;
           marker.scale.y = 0.1;
           marker.scale.z = 0.1;
   
           marker.color.r = 0.0f;
           marker.color.g = 1.0f;
           marker.color.b = 0.0f;
           marker.color.a = 1.0;

           marker.action = visualization_msgs::Marker::ADD;
           marker_pub.publish(marker);
           marker.lifetime = ros::Duration();
        }
     }
     
 

     // Cycle between different shapes
/*
     switch (shape)
     {
     case visualization_msgs::Marker::CUBE:
       shape = visualization_msgs::Marker::SPHERE;
       break;
     case visualization_msgs::Marker::SPHERE:
       shape = visualization_msgs::Marker::ARROW;
       break;
     case visualization_msgs::Marker::ARROW:
       shape = visualization_msgs::Marker::CYLINDER;
       break;
     case visualization_msgs::Marker::CYLINDER:
       shape = visualization_msgs::Marker::CUBE;
       break;
     }
*/
 
     r.sleep();
   }
   ros::spin();
}
