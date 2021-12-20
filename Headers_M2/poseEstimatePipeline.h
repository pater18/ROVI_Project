#pragma once

//#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/core/Log.hpp>
#include <rw/core.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/kinematics.hpp>
#include <rw/proximity.hpp>
#include <rw/trajectory.hpp>
#include <rw/loaders/image/ImageLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
//#include "reachability.h"
#include "denseStereo.h"
#include "poseEstimate.h"
//#include "poseEstimate2.h"
#include "addNoiseToCloud.h"
#include "saveToCSV.h"
#include <pcl/point_types.h>
#include "errorOfPose.h"
#include <random>
#include <functional>
#include <pcl/kdtree/kdtree_flann.h>
#include "RRT.h"

#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;

typedef PointNormal PointT;

void poseEstimatePipelineForMain()
{
    // Bottle poses
    std::vector<Vector3D<>> vectors;

    std::mt19937 generator(5);
    std::uniform_real_distribution<float> uniform_distributionX(-0.3, 0.3);
    std::uniform_real_distribution<float> uniform_distributionY(0.4, 0.5);

    int num_scenes = 2; //Change this to change the number of experiments

    for (int i = 0; i < num_scenes; i++)
    {
        vectors.push_back(Vector3D<>(uniform_distributionX(generator), uniform_distributionY(generator), 0.21));
    }

    RPY<> R1 = RPY<>(-1.571, 0, 1.571);

    std::vector<rw::math::Transform3D<>> bottle_transformations;
    rw::math::Transform3D<> temp = Transform3D<>(vectors[0], R1.toRotation3D());

    for (size_t i = 0; i < vectors.size(); i++)
    {
        temp = Transform3D<>(vectors[i], R1.toRotation3D());
        bottle_transformations.push_back(temp);
    }
    std::vector<rw::math::Transform3D<>> poses_actual = makePointCloudFromScene(bottle_transformations);

    std::vector<Matrix4f> poses_for_rrt;

    for (float std = 0.0; std < 0.015; std += 0.0025)
    {
        std::vector<std::vector<double>> csv_data;

        for (int i = 0; i < num_scenes; i++)
        {
            std::cout << "Proseccing cloud numer " << i << " / " << num_scenes << " with std: " << std << std::endl;

            pcl::PointCloud<PointT>::Ptr noisy_cloud = addNoice("scene_clouds/cloud_scene" + std::to_string(i) + ".pcd", std);
            std::vector<Matrix4f> pose = poseEstimatePCL("bottle2_1.ply", noisy_cloud);
            csv_data.push_back(calcErrorOnPose(bottle_transformations[i], pose, poses_for_rrt));
        }
        saveVelAcc(csv_data, "../csv_files/error_pos_ang_noice_" + std::to_string(int(std * 10000)) + ".csv");
    }

    // rrtPlanning(poses_for_rrt);

}