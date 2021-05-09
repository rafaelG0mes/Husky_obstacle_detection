#include <cloud_processing/avoidance_path.h>
#include <cloud_processing/config.h>

namespace obstacle_detection {

  AvoidancePath::AvoidancePath (ros::NodeHandle node)
  {
    node_ = node;
    information_pub_   = node_.advertise<std_msgs::Float64MultiArray> ("/cloud_processing/object_information", 100);

    if (ENABLE_AVOIDANCE_PATH_CLOUDS){
      bezier_circle_pub_ = node_.advertise<pcl::PCLPointCloud2>         ("/cloud_processing/bezier_circle", 100);
      safety_circle_pub_ = node_.advertise<pcl::PCLPointCloud2>         ("/cloud_processing/safety_circle", 100);
    }

    //Define the topic that will send the information about the obstacle detection
    cloud_process_ = node_.subscribe ("/cloud_processing/passthrough", 1000, & AvoidancePath::cloud_cb, this, ros::TransportHints().tcpNoDelay(true));
  }


  AvoidancePath::~AvoidancePath() {}


  void AvoidancePath::send_object_information (std_msgs::Float64MultiArray & object_info,
                                               ObjectProperties ob_props,
                                               int object_count)
  {
    float radius_safety = SAFETY_CONSTANT + ob_props.radius;
    object_info.data.clear();

    object_info.data.push_back (999);
    object_info.data.push_back (ob_props.media_x);
    object_info.data.push_back (ob_props.media_y);
    object_info.data.push_back (ob_props.radius);
    object_info.data.push_back (radius_safety);
    object_info.data.push_back (object_count);
  }

  void AvoidancePath::enable_avoidance_cloud (cloud_param_set & avoidance_cloud,
                                              ObjectProperties ob_props)
  {
    avoidance_cloud.ob_props = ob_props;
    avoidance_cloud.ob_props.radius += avoidance_cloud.scale_factor_;
    get_circle_cloud (avoidance_cloud.cloud_, avoidance_cloud.ob_props);
  }

  void AvoidancePath::cloud_cb (const pcl::PCLPointCloud2ConstPtr & input)
  {
    cloud_param_set bezier_circle  (VELODYNE_FRAME, RADIUS_BEZIER);
    cloud_param_set safety_circle  (VELODYNE_FRAME, SAFETY_CONSTANT);
    cloud_param_set cloud_filtered (VELODYNE_FRAME);

    int object_count = 0;
    ObjectProperties ob_props;
    std::vector<pcl::PointIndices> cluster_indices;
    std_msgs::Float64MultiArray object_info;

    objects_preprocessing (input, cloud_filtered.cloud_, cluster_indices);
    clear_visual_clouds ();

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end (); ++it)
    {
      cloud_param_set iterator_cloud (VELODYNE_FRAME);

      // If there is no obstacle, send 1. If there is an obstacle, send 999.
      object_info.data.clear();
      object_info.data.push_back(1);

      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        iterator_cloud.cloud_->points.push_back (cloud_filtered.cloud_->points[*pit]);

      // Only enters if the object is detected as obstacle by the check_object function
      if (check_object (cloud_filtered.cloud_, cluster_indices, it, ob_props))
      {
        send_object_information (object_info, ob_props, object_count);

        if (ENABLE_AVOIDANCE_PATH_CLOUDS){
          enable_avoidance_cloud (bezier_circle, ob_props);
          enable_avoidance_cloud (safety_circle, ob_props);
        }
      }
      object_count++;
      information_pub_.publish (object_info);
    }

    if (ENABLE_AVOIDANCE_PATH_CLOUDS){
      bezier_circle_pub_.publish(bezier_circle.cloud_);
      safety_circle_pub_.publish(safety_circle.cloud_);
    }

  }
} // namespace obstacle_detection