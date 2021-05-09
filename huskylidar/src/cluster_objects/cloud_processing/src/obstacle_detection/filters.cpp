#include <cloud_processing/filters.h>
#include <cloud_processing/config.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

namespace obstacle_detection {

  Filters:: Filters()
  {
    voxel_pub_  = node_.advertise<pcl::PCLPointCloud2> ("/cloud_processing/voxel_filter", 100);
    vision_pub_ = node_.advertise<pcl::PCLPointCloud2> ("/cloud_processing/visionField" , 100);
  }

  Filters::~Filters(){}

  void Filters::downsample_filters (const pcl::PCLPointCloud2ConstPtr & input,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered)
  {
    voxel_filter        (input, cloud_filtered);
    voxel_pub_.publish  (cloud_filtered);

    // field_vision_filter (cloud_filtered);
    // vision_pub_.publish  (cloud_filtered);
  }


  void Filters::voxel_filter (const pcl::PCLPointCloud2ConstPtr &input,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
  {
    pcl::PCLPointCloud2::Ptr cloud_voxel (new pcl::PCLPointCloud2);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (input);
    sor.setLeafSize   (0.1f, 0.1f, 0.1f);
    sor.filter        (*cloud_voxel);
    /* Conversion: The type received from the Lidar is sensor_msg::PointCloud2, here we need to use
    pcl::PointXYZ. */
    pcl::fromPCLPointCloud2 (*cloud_voxel, *cloud_filtered);
  }


  void Filters::field_vision_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
  {
    pcl::ExtractIndices<pcl::PointXYZ> extr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vision (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices vision_indices;
    *cloud_vision = *cloud_filtered;
    for (int i = 0; i < (*cloud_vision).size(); i++)
    {
      pcl::PointXYZ pt (cloud_vision->points[i].x,
                        cloud_vision->points[i].y,
                        cloud_vision->points[i].z);
      if ( pt.x >= 0)   // Abs function gives the desired vision area
        vision_indices.indices.push_back(i);
    }

    extr.setInputCloud  (cloud_filtered);
    extr.setIndices     (boost::make_shared<const pcl::PointIndices> (vision_indices));
    extr.setNegative    (true);
    extr.filter         (*cloud_filtered);
  }
} // namespace obstacle_detection
