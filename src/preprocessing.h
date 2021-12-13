#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
//using namespace pcl::registration;
using namespace pcl::search;
//using namespace pcl::visualization;
using namespace Eigen;

typedef PointNormal PointT;
typedef Histogram<153> FeatureT;

void voxelGrid( pcl::PointCloud<PointT>::Ptr input_cloud, pcl::PointCloud<PointT>::Ptr &output_cloud )
{
  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (input_cloud);
  const float leaf = 0.00005f;
  sor.setLeafSize (leaf, leaf, leaf);
  sor.filter (*output_cloud);
	
}


// void outlierRemoval( pcl::PointCloud<PointT>::Ptr input_cloud, pcl::PointCloud<PointT>::Ptr &output_cloud )
// {

//   // Create the filtering object
//   pcl::StatisticalOutlierRemoval<PointT> sor;
//   sor.setInputCloud (input_cloud);
//   sor.setMeanK (200);
//   sor.setStddevMulThresh (0.01);
//   sor.filter (*output_cloud);

// }


// void smoothing( pcl::PointCloud<PointT>::Ptr input_cloud, pcl::PointCloud<PointT>::Ptr &output_cloud )
// {

//   // Create a KD-Tree
//   pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

//   // Output has the PointNormal type in order to store the normals calculated by MLS
//   pcl::PointCloud<PointT> mls_points;

//   // Init object (second point type is for the normals, even if unused)
//   pcl::MovingLeastSquares<PointT, PointT> mls;
 
//   mls.setComputeNormals (true);

//   // Set parameters
//   mls.setInputCloud (input_cloud);
//   mls.setPolynomialOrder (3);
//   mls.setSearchMethod (tree);
//   mls.setSearchRadius (10.0f);
  
//   // Reconstruct
//   mls.process (mls_points);
  
//   pcl::copyPointCloud<PointT, PointT>(mls_points, *output_cloud); 
  
// }

void preprocess(pcl::PointCloud<PointT>::Ptr input_cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered)
{

    std::cerr << "PointCloud before filtering: " << input_cloud->width * input_cloud->height 
       << " data points (" << pcl::getFieldsList (*input_cloud) << ")." << std::endl;

    voxelGrid( input_cloud, cloud_filtered );

    // outlierRemoval( cloud_filtered, cloud_filtered );
    
    //smoothing( cloud_filtered, cloud_filtered );
  
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

}