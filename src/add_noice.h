#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
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
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;


typedef PointNormal PointT;

void spatialFilter( pcl::PointCloud<PointT>::Ptr input_cloud, pcl::PointCloud<PointT>::Ptr &output_cloud )
{
	
  // Create the filtering object
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1.5, -1);
  pass.filter (*output_cloud);

  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.371274, 0.197482);
  pass.filter (*output_cloud);

}

void addNoice(const std::string object_name, const std::string scene_name )
{

    // Load
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile<PointT> (scene_name, *scene); 
    pcl::PointCloud<PointT>::Ptr object(new pcl::PointCloud<PointT>);
    pcl::io::loadPLYFile<PointT> (object_name, *scene);

    spatialFilter(scene, scene);


    pcl::PointCloud<PointT>::Ptr scene_filtered (new pcl::PointCloud<PointT> ());
    scene_filtered->points.resize (scene->points.size());


    {
        PCLVisualizer v("Before global alignment");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0), "scene");
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.spin();
        v.close();
    }

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.01, 0.02);

    for (size_t point_i = 0; point_i < scene->points.size(); ++point_i)
    {
        scene_filtered->points[point_i].x = scene->points[point_i].x + static_cast<float>(distribution(generator));
        scene_filtered->points[point_i].y = scene->points[point_i].y + static_cast<float>(distribution(generator));
        scene_filtered->points[point_i].z = scene->points[point_i].z + static_cast<float>(distribution(generator));
    }

    {
        PCLVisualizer v("Before global alignment");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0), "scene");
        v.spin();
    }
}
