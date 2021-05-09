#ifndef _AVOIDANCE_PATH_H_
#define _AVOIDANCE_PATH_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <cloud_processing/object_classification.h>

namespace obstacle_detection {

  class AvoidancePath : public ObjectClassification
  {
    public:
      AvoidancePath (ros::NodeHandle node);
      ~AvoidancePath ();

      /**
       * @brief Callback function, that will be called in a loop by ROS
       *
       * @param input - cloud from velodyne
       */
      void cloud_cb (const pcl::PCLPointCloud2ConstPtr & input);


    private:
      ros::Subscriber cloud_process_  ;

      // For the avoidance path strategy
      ros::Publisher bezier_circle_pub_;
      ros::Publisher safety_circle_pub_;
      ros::Publisher information_pub_  ;

      /**
       * @brief: Send importante information about the obstacles, to be processed by the python script
       *         in the search for avoidance paths
       *
       * @param obstacle_info   - array that will be filled with information to be sent
       * @param ob_props        - properties of the obstacle that will be used by the python script in the
       *                          search for avoidance paths
       * @param object_count    - counts the number of objects in the input
       * @param information_pub - ROS publisher
       */
      void send_object_information (std_msgs::Float64MultiArray & obstacle_info,
                                    ObjectProperties ob_props,
                                    int object_count);
      /**
       * @brief: Prepares a point cloud to be showed as a circle cloud
       *
       * @param avoidance_cloud - empty cloud that will be constructed based on the ob_props
       * @param ob_props        - properties of the auxiliary clouds to run the algorithm
       */
      void enable_avoidance_cloud (cloud_param_set & avoidance_cloud,
                                   ObjectProperties ob_props);
  };

} // namespace obstacle_detection

#endif