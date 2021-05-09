#include <cloud_processing/avoidance_path.h>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cloud_traitement");
  ros::NodeHandle node;

  // DataDownsample class to subscribe to velodyne2 at "/cloud_processing/passthrough"
  obstacle_detection::AvoidancePath avoidance_path (node);

  // Handle callbacks until shut down
  ros::spin ();

  return 0;
}