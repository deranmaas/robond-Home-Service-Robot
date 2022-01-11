#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


visualization_msgs::Marker marker;
ros::Publisher marker_pub;

double pickup_x = 5.8;
double pickup_y = 3.8;
double drop_x = -1.7;
double drop_y = -3.5;


void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  // get the robot position
  double robot_x = msg->pose.pose.position.x;
  double robot_y = msg->pose.pose.position.y;

  // compute distance to the pickup and drop zone
  double d_pickup = sqrt(pow(robot_x - pickup_x, 2) + pow(robot_y - pickup_y, 2));
  double d_drop = sqrt(pow(robot_x - drop_x, 2) + pow(robot_y - drop_y, 2));
  ROS_INFO("pickup: %8.2f, drop: %8.2f", d_pickup, d_drop);

  if (d_pickup < 0.3) {
    // Hide the marker
    marker.pose.position.x = pickup_x;
    marker.pose.position.y = pickup_y;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
  }

  if (d_drop < 0.3) {
    // Show the marker
    marker.pose.position.x = drop_x;
    marker.pose.position.y = drop_y;
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
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
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = .5;
  marker.scale.y = .5;
  marker.scale.z = .5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  // Publish the marker at the pickup zone 
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pickup_x;
  marker.pose.position.y = pickup_y;
  marker_pub.publish(marker);

  ros::Subscriber sub = n.subscribe("/amcl_pose",1000, amclPoseCallback);
  ros::spin(); 

  return 0;
}