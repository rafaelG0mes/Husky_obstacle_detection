#include <cloud_processing/object_classification.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <math.h>

namespace obstacle_detection {

  ObjectClassification::ObjectClassification()
  {
    clear_visual_clouds ();
    obstacles_pub_       = node_.advertise<pcl::PCLPointCloud2> ("/cloud_processing/obstacles_cloud"   , 100);
    notObstacles_pub_    = node_.advertise<pcl::PCLPointCloud2> ("/cloud_processing/notObstacles_cloud", 100);

    if (ENABLE_OBJECT_CLASSIFICATION_CLOUDS)
      obstacleInflate_pub_ = node_.advertise<pcl::PCLPointCloud2> ("/cloud_processing/safeCircle_cloud"  , 100);
  }

  ObjectClassification::~ObjectClassification() {}


  void ObjectClassification::cluster_objects (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered,
                                              std::vector<pcl::PointIndices> & cluster_indices)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud((*cloud_filtered).makeShared()); //Make sure to not forget the .makeShared part, otherwise it will return a "core dupemd segmentation faut error"

    //Cluster point cloud based on the kd-tree algorithm, and returns a vector of indices
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.4);
    ec.setMinClusterSize   (10);
    ec.setMaxClusterSize   (25000);
    ec.setSearchMethod     (tree);
    ec.setInputCloud       ((*cloud_filtered).makeShared());
    ec.extract             (cluster_indices);
  }


  void ObjectClassification::objects_preprocessing (const pcl::PCLPointCloud2ConstPtr &input,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered,
                                                    std::vector<pcl::PointIndices> & cluster_indices)
  {
    downsample_filters (input, cloud_filtered);
    cluster_objects    (cloud_filtered, cluster_indices);
  }

  bool ObjectClassification::check_object (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered,
                                           std::vector<pcl::PointIndices> &cluster_indices,
                                           std::vector<pcl::PointIndices>::const_iterator & it,
                                           ObjectProperties & ob_props)
  {
    bool is_obstacle;
    cloud_param_set iterator_cloud (VELODYNE_FRAME);

    // Creates an iterator that will pass by each cluster detected with the kd-tree
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      iterator_cloud.cloud_->points.push_back (cloud_filtered->points[*pit]);

    find_distances(iterator_cloud.cloud_, ob_props);

    //Defines the size of the safety circle
    float radius_safety = SAFETY_CONSTANT + ob_props.radius;

    //Only enters in this loop if object is considered as an obstacle
    if ((pow( pow(ob_props.media_x,2) + pow(ob_props.media_y,2), 0.5) - radius_safety) < 0.3){
      *obstacles_cloud += *iterator_cloud.cloud_;
      is_obstacle = true;
    }
    else {//If it is not an obstacle, it is saved in the notObstacles_cloud
      *notObstacles_cloud += *iterator_cloud.cloud_;
      is_obstacle = false;
    }
    obstacles_pub_.publish       (obstacles_cloud);
    notObstacles_pub_.publish    (notObstacles_cloud);

    if (ENABLE_OBJECT_CLASSIFICATION_CLOUDS){
      get_circle_cloud (obstacleInflate_cloud, ob_props);
      obstacleInflate_pub_.publish (obstacleInflate_cloud);
    }

    return is_obstacle;
  }


  void ObjectClassification::find_distances (pcl::PointCloud<pcl::PointXYZ>::Ptr &iterator_cloud,
                                             ObjectProperties & ob_props)
  {
    //Declares the variables to help to find the obstacles
    std::vector<float> min_dist;
    std::vector<float> x_value;
    std::vector<float> y_value;
    float cluster_minDist;
    float cluster_medDist;

    // Computes the distance between all the points and (0,0,0)
    for (int i = 0; i < (*iterator_cloud).size(); i++)
    {
      float x = iterator_cloud->points[i].x;
      float y = iterator_cloud->points[i].y;
      x_value.push_back(x);                     //Gets all x values for a given cluster cloud
      y_value.push_back(y);                     //Gets all y values for a given cluster cloud
      min_dist.push_back (pow(x*x + y*y, 0.5)); //For each point of a cluster point cloud, computes the distance between the point and the robot
    }

    //Find the smallest distance considering every point of the same object
    auto smallest   = std::min_element(std::begin(min_dist), std::end(min_dist));
    cluster_minDist = (float)*smallest;

    auto  min_x = std::min_element(std::begin(x_value), std::end(x_value));
    auto  max_x = std::max_element(std::begin(x_value), std::end(x_value));
    auto  min_y = std::min_element(std::begin(y_value), std::end(y_value));
    auto  max_y = std::max_element(std::begin(y_value), std::end(y_value));

    //Computes the barycenter of the cluster point cloud
    ob_props.media_x = ((float)*max_x - (float)*min_x)/2 + (float)*min_x;
    ob_props.media_y = ((float)*max_y - (float)*min_y)/2 + (float)*min_y;

    //Distance beetween center of the cluster point cloud and (0,0,0)
    cluster_medDist = (pow( pow(ob_props.media_x,2) + pow(ob_props.media_y,2),0.5));

    //Checks if the sparseness of points is greater on the x-axis or the y-axis, so the circle can include all the points of the cluster
    ob_props.radius = (((*max_x - *min_x)/2) > ((*max_y-*min_y)/2)) ? ((*max_x - *min_x)/2) : ((*max_y-*min_y)/2);
    ob_props.radius = ob_props.radius + RADIUS_FACTOR*ob_props.radius;
  }

  void ObjectClassification::get_circle_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr & circle_cloud,
                                               ObjectProperties & ob_props)
  {
    // Creates the cloud_aux that will help represent the ob_props of the object on Rviz
    cloud_param_set cloud_aux (VELODYNE_FRAME);
    (cloud_aux.cloud_)->height   = INFLATION_SCALE*(2*M_PI*ob_props.radius);
    (cloud_aux.cloud_)->width    = 1;
    (cloud_aux.cloud_)->is_dense = false;
    (cloud_aux.cloud_)->points.resize ((cloud_aux.cloud_)->height * (cloud_aux.cloud_)->width);

    /* Set variables to go over the circumference of the circle described by ob_props, based on
     * how many points the cloud_aux have
    */
    float x = 0;
    float step_size = ((*(cloud_aux.cloud_)).size()-1);
    double x_t = 2*M_PI/step_size;

    /* Transformation, bringing each point of the cloud_aux into the circumference of the circle
     * described by ob_props
    */
    for (int i = 0; i < (*(cloud_aux.cloud_)).size(); i++)
    {
      (cloud_aux.cloud_)->points[i].x = ob_props.radius*cos(x) + ob_props.media_x;
      (cloud_aux.cloud_)->points[i].y = ob_props.radius*sin(x) + ob_props.media_y;
      (cloud_aux.cloud_)->points[i].z = Z_HEIGHT_CLOUD;
      x = x + x_t;
    }

    /* Acumulates the old point cloud of the cluster iterator with the current iteration (for
     * visual purposes only)
    */
    *circle_cloud  += *(cloud_aux.cloud_);
  }

  void ObjectClassification::clear_visual_clouds ()
  {
    // Clear cloud content by reseting them
    obstacles_cloud.reset       (new pcl::PointCloud<pcl::PointXYZ>);
    notObstacles_cloud.reset    (new pcl::PointCloud<pcl::PointXYZ>);
    //Changes the header frame id of the new clouds, so they will match with the frame of the sensor ("Global Options/Fixed frame" on Rviz)
    obstacles_cloud->header.frame_id       = VELODYNE_FRAME;
    notObstacles_cloud->header.frame_id    = VELODYNE_FRAME;

    if(ENABLE_OBJECT_CLASSIFICATION_CLOUDS) {
      obstacleInflate_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
      obstacleInflate_cloud->header.frame_id = VELODYNE_FRAME;
    }

  }
} // namespace obstacle_detection