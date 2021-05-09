#ifndef _CONFIGURATION_FILE_H_
#define _CONFIGURATION_FILE_H_

  /**
   * @brief Configurations for the visualization of the algorithm. Enabling any of these will slowdown
   * (maybe significantly) the computing time for the obstacle detection.
   * ENABLE_OBJECT_CLASSIFICATION_CLOUDS - Enables visualization of the input cloud after each filter
   * ENABLE_AVOIDANCE_PATH_CLOUDS        - Enables visualization of the auxiliary clouds used on the algorithm
   */
  #define ENABLE_OBJECT_CLASSIFICATION_CLOUDS       false
  #define ENABLE_AVOIDANCE_PATH_CLOUDS              false


  /**
   * @brief Configurations for the object classification class
   * VELODYNE_FRAME  - this frame will match the point cloud frame with the robot frame,
   *                   to have the same reference
   * RADIUS_FACTOR   - is there to increase the radius, to ensure that the points of the
   *                   cluster will be inside of it
   * Z_HEIGHT_CLOUD  - sets the height in which the inflation circle of the object will be
   *                   displayed
   * SAFETY_CONSTANT - constant that will help to determine the obstacles. It can also be a
   *                   variable from a function.
   * INFLATION_SCALE - constant that will help to determine the sparsness of the inflation
   *                   circle cloud
   */
  #define VELODYNE_FRAME                            "velodyne2"
  #define RADIUS_FACTOR                              0.3
  #define Z_HEIGHT_CLOUD                            -1
  #define SAFETY_CONSTANT                            2
  #define INFLATION_SCALE                            15

  /**
   * @brief Configurations for the avoidance path class
   * RADIUS_BEZIER - constant that will help to define the radius of the auxiliary circle for the bezier
   *                 curves algorithm
   */
  #define RADIUS_BEZIER                              1

#endif