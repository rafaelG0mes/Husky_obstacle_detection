#ifndef _FILTERS_H_
#define _FILTERS_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

namespace obstacle_detection {

  class Filters
  {
    public:
      Filters();
      ~Filters();

    protected:
      ros::NodeHandle node_;

      /**
       * @brief: Function that will call all the filters implemented, to be used by the ObjectClassification class
       *
       * @param input          - point cloud coming from the topic subscribed at the constructor of the AvoidancePath class
       * @param cloud_filtered - output point cloud
       */
      void downsample_filters (const pcl::PCLPointCloud2ConstPtr & input,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered);

    private:
      ros::Publisher voxel_pub_ ;
      ros::Publisher vision_pub_;
      /**
       * @brief: Filter that applies the voxel grid into the input cloud
       *
       * @param input          - point cloud coming from the topic subscribed at the constructor of the AvoidancePath class
       * @param cloud_filtered - output point cloud
       */
      void voxel_filter (const pcl::PCLPointCloud2ConstPtr & input,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered);
      /**
       * @brief: Filter that limits eliminates points out of a chosen region
       *
       * @param cloud_filtered - point cloud that will receive the filter
       */
      void field_vision_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered);
  };

} // namespace obstacle_detection

#endif