#ifndef _OBJECT_CLASSIFICATION_H_
#define _OBJECT_CLASSIFICATION_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <cloud_processing/filters.h>
#include <cloud_processing/config.h>
#include <std_msgs/Float64MultiArray.h>

namespace obstacle_detection {
  /**
   * @brief: Struct to handle individual objects after they are in clusters
   */
  struct ObjectProperties
  {
    float radius;
    float media_x;
    float media_y;
  };

  /**
   * @brief: Struct to initialize clouds and handle the option of visualizing them on Rviz
   */
  struct cloud_param_set
  {
    /**
     * @brief: Construct a new cloud param set object
     *
     * @param frame        - set the cloud frame to match the robot on Rviz
     * @param scale_factor - Adjust the sparsness of the points in the cloud that helps visualize
     */
    cloud_param_set (std::string frame, float scale_factor)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      cloud->header.frame_id = frame;
      cloud_.reset (new pcl::PointCloud<pcl::PointXYZ>);
      *cloud_ = *cloud;
      scale_factor_ = scale_factor;
    }
    /**
     * @brief: Construct a new cloud param set object
     *
     * @param frame - set the cloud frame to match the robot on Rviz
     */
    cloud_param_set (std::string frame)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      cloud->header.frame_id = frame;
      cloud_.reset (new pcl::PointCloud<pcl::PointXYZ>);
      *cloud_ = *cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    float scale_factor_;
    ObjectProperties ob_props;
  };


  class ObjectClassification: public Filters
  {
    public:
      ObjectClassification();
      ~ObjectClassification();


    protected:
      /**
       * @brief: Get the circle cloud object
       *
       * @param circle_cloud - output cloud, showing a circle based on the ob_props paramters
       * @param ob_props     - paramters of objects after the cluster stage
       */
      void get_circle_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr & circle_cloud,
                             ObjectProperties & ob_props);
      /**
       * @brief: Execute the preprocessing stages (downsample and cluster) on the input cloud
       *
       * @param input           - cloud from the velodyne
       * @param cloud_filtered  - output cloud
       * @param cluster_indices - indices obtained at the cluster stage
       */
      void objects_preprocessing (const pcl::PCLPointCloud2ConstPtr &input,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered,
                                  std::vector<pcl::PointIndices> & cluster_indices);

      /**
       * @brief: Clear the clouds that show the obstacles, non-obstacles and inflation circle
       */
      void clear_visual_clouds ();

      /**
       * @brief: Verifies if a given object is an obstacle or not, based on the distance of its center of
       * mass in comparison with the robot
       *
       * @param cloud_filtered  - downsampled cloud, which is a representation of the input cloud
       * @param cluster_indices - indices obtained at the cluster stage
       * @param it              - iterator that goes over all the clusters
       * @param ob_props        - important paramters of each object on a given iteration of the cluster
       * @return true           - if the object is an obstacle
       * @return false          - if the object is not an obstacle
       */
      bool check_object (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered,
                         std::vector<pcl::PointIndices> & cluster_indices,
                         std::vector<pcl::PointIndices>::const_iterator & it,
                         ObjectProperties & ob_props);
      private:
        ros::Publisher obstacles_pub_;
        ros::Publisher notObstacles_pub_;
        ros::Publisher obstacleInflate_pub_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr notObstacles_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacleInflate_cloud;

        /**
         * @brief: Finds the distances of the barycenter coordinates
         *
         * @param iterator_cloud - pcl::PointXYZ at each iteration of the it iterator
         * @param ob_props       - important paramters of each object on a given iteration of the cluster
         */
        void find_distances (pcl::PointCloud<pcl::PointXYZ>::Ptr & iterator_cloud,
                             ObjectProperties & ob_props);
        /**
         * @brief: Applies the kd-tree algorithm to cluster the input cloud
         *
         * @param cloud_filtered  - input cloud, already downsampled
         * @param cluster_indices - indices of the clusters
         */
        void cluster_objects (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered,
                              std::vector<pcl::PointIndices> & cluster_indices);
  };

} // namespace obstacle_detection

#endif