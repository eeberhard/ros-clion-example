#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/TeleportAbsolute.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "circler");
  ros::NodeHandle n;
  auto vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  ros::ServiceClient client = n.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

  turtlesim::TeleportAbsolute srv;
  srv.request.x = 0;
  srv.request.y = 0.5;
  srv.request.theta = 0;

  if (client.call(srv)) {
    ROS_INFO("Teleporting to 0, 0.5, 0");
  } else {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    geometry_msgs::Twist vel;
    vel.linear.x = 2;
    vel.angular.z = 0.5;
    vel_pub.publish(vel);
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

// hello software group