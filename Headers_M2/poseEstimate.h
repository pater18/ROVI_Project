#pragma once

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/spin_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <pcl/console/time.h> // TicToc
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>





using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
// using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureTT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureTT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureTT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
typedef Histogram<153> FeatureT;

void spatialFilter2(pcl::PointCloud<PointNT>::Ptr input_cloud, pcl::PointCloud<PointNT>::Ptr &output_cloud)
{

  // Create the filtering object
  pcl::PassThrough<PointNT> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1.5, -1);
  pass.filter(*output_cloud);

  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.371274, 0.197482);
  pass.filter(*output_cloud);
}

std::vector<Matrix4f> poseEstimatePCL(const std::string object_name, pcl::PointCloud<PointNT>::Ptr scene)
{
  // Point clouds
  // Load
  // pcl::PointCloud<PointT>::Ptr object(new pcl::PointCloud<PointT>);
  // pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
  // pcl::io::loadPLYFile<PointT> (object_name, *object);
  // pcl::io::loadPCDFile<PointT> (scene_name, *scene);

  PointCloudT::Ptr object(new PointCloudT);
  PointCloudT::Ptr object_aligned(new PointCloudT);
  FeatureCloudT::Ptr object_features(new FeatureCloudT);
  FeatureCloudT::Ptr scene_features(new FeatureCloudT);
  // pcl::PointCloud<FeatureT>::Ptr object_features(new pcl::PointCloud<FeatureT>);
  // pcl::PointCloud<FeatureT>::Ptr scene_features(new pcl::PointCloud<FeatureT>);
  std::vector<Matrix4f> global_local;
  // Get input object and scene

  // Load object and scene
  pcl::console::print_highlight("Loading point clouds...\n");
  if (pcl::io::loadPLYFile<PointNT>(object_name, *object) < 0)
  {
    pcl::console::print_error("Error loading object/scene file!\n");
  }

  std::cout << scene->height << std::endl
            << scene->width << std::endl;
  // {
  //     PCLVisualizer v("Before global alignment");
  //     v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
  //     v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
  //     v.spin();
  //     v.close ();
  // }

  //Downsample
  pcl::console::print_highlight("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.01f;
  grid.setLeafSize(leaf, leaf, leaf);
  //grid.setInputCloud (object);
  //grid.filter (*object);
  grid.setInputCloud(scene);
  grid.filter(*scene);

  // Estimate normals for scene
  pcl::console::print_highlight("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT, PointNT> nest;
  nest.setKSearch(10);
  nest.setInputCloud(scene);
  nest.compute(*scene);
  nest.setInputCloud(object);
  nest.compute(*object);

  spatialFilter2(scene, scene);

  // Estimate features
  pcl::console::print_highlight("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch(0.15);
  fest.setInputCloud(object);
  fest.setInputNormals(object);
  fest.compute(*object_features);
  fest.setInputCloud(scene);
  fest.setInputNormals(scene);
  fest.compute(*scene_features);


  // SpinImageEstimation<PointNT, PointNT, FeatureT> spin;
  // spin.setRadiusSearch(0.15);
  // spin.setInputCloud(object);
  // spin.setInputNormals(object);
  // spin.compute(*object_features);
  // spin.setInputCloud(scene);
  // spin.setInputNormals(scene);
  // spin.compute(*scene_features);

  // Perform alignment
  pcl::console::print_highlight("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureTT> align;
  align.setInputSource(object);
  align.setSourceFeatures(object_features);
  align.setInputTarget(scene);
  align.setTargetFeatures(scene_features);
  align.setMaximumIterations(100000 * 2);              // Number of RANSAC iterations
  align.setNumberOfSamples(3);                     // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(10);           // Number of nearest features to use
  align.setSimilarityThreshold(0.6f);              // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(5.0f * leaf); // Inlier threshold
  align.setInlierFraction(0.10f);                  // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align(*object_aligned);
  }

  Eigen::Matrix4f global_pose;
  // if (align.hasConverged ())
  // {
  // Print results
  printf("\n");
  global_pose = align.getFinalTransformation();
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", global_pose(0, 0), global_pose(0, 1), global_pose(0, 2));
  pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", global_pose(1, 0), global_pose(1, 1), global_pose(1, 2));
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", global_pose(2, 0), global_pose(2, 1), global_pose(2, 2));
  pcl::console::print_info("\n");
  pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", global_pose(0, 3), global_pose(1, 3), global_pose(2, 3));
  pcl::console::print_info("\n");
  pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());



  // Create a k-d tree for scene
  search::KdTree<PointNormal> tree2;
  tree2.setInputCloud(scene);

  // Set ICP parameters
  const size_t iter2 = 100;
  const float thressq2 = 0.01 * 0.01;

  // Start ICP
  Matrix4f pose_local = Matrix4f::Identity();
  //Matrix4f pose_local = pose;
  {
    ScopeTime t("ICP");
    cout << "Starting ICP..." << endl;
    for (size_t i = 0; i < iter2; ++i)
    {
      // 1) Find closest points
      vector<vector<int>> idx;
      vector<vector<float>> distsq;
      tree2.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);

      // Threshold and create indices for object/scene and compute RMSE
      vector<int> idxobj;
      vector<int> idxscn;
      for (size_t j = 0; j < idx.size(); ++j)
      {
        if (distsq[j][0] <= thressq2)
        {
          idxobj.push_back(j);
          idxscn.push_back(idx[j][0]);
        }
      }

      // 2) Estimate transformation
      Matrix4f T;
      TransformationEstimationSVD<PointNormal, PointNormal> est;
      est.estimateRigidTransformation(*object_aligned, idxobj, *scene, idxscn, T);

      // 3) Apply pose_local
      transformPointCloud(*object_aligned, *object_aligned, T);

      // 4) Update result
      pose_local = T * pose_local;
    }

    // Compute inliers and RMSE
    vector<vector<int>> idx;
    vector<vector<float>> distsq;
    tree2.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
    size_t inliers = 0;
    float rmse = 0;
    for (size_t i = 0; i < distsq.size(); ++i)
      if (distsq[i][0] <= thressq2)
        ++inliers, rmse += distsq[i][0];
    rmse = sqrtf(rmse / inliers);

    // Print pose_local
    // cout << "Got the following pose_local:" << endl << pose_local << endl;
    // cout << "Inliers: " << inliers << "/" << object->size() << endl;
    // cout << "RMSE: " << rmse << endl;
  } // End timing

  //Show result
  // {
  //     PCLVisualizer v("After local alignment");
  //     v.addPointCloud<PointNormal>(object_aligned, PointCloudColorHandlerCustom<PointNormal>(object_aligned, 0, 255, 0), "object_aligned");
  //     v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
  //     v.spin();
  // }


  Matrix4f local_global = pose_local * global_pose;

  std::cout << "Final pose: " << local_global << std::endl;

  global_local.push_back(global_pose);
  global_local.push_back(pose_local);

  return global_local;
}