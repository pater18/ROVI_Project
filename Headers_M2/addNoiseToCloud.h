#pragma once

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
#include <pcl/io/file_io.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <random>





using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;


typedef PointNormal PointT;
//typedef pcl::PointXYZ PointT;



pcl::PointCloud<PointT>::Ptr addNoice(const std::string scene_name, float std)
{

    // Load
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile<PointT> (scene_name, *scene);

    
    pcl::PointCloud<PointT>::Ptr scene_filtered(new pcl::PointCloud<PointT>);
    scene_filtered->points.resize (scene->points.size());
    scene_filtered->header = scene->header;
    scene_filtered->width = scene->width;
    scene_filtered->height = scene->height;

    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0, std);

    for (size_t point_i = 0; point_i < scene->points.size(); point_i++)
    {
        scene_filtered->points[point_i].x = scene->points[point_i].x + (distribution(generator));
        scene_filtered->points[point_i].y = scene->points[point_i].y + (distribution(generator));
        scene_filtered->points[point_i].z = scene->points[point_i].z + (distribution(generator));
    }

    return scene_filtered;
}
