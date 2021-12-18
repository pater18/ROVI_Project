

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
#include "interpolation.h"
#include "reachability.h"
#include "denseStereo.h"
#include "poseEstimate.h"
#include "poseEstimate2.h"
#include <pcl/point_types.h>
#include "errorOfPose.h"
#include <random>
#include <functional>





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

double uniform0to1Random()
{
    double r = random();
    return r / ((double)RAND_MAX + 1);
}

double myRandom()
{
  return 0.6 * uniform0to1Random() - 0.3;
}

int main(int argc, char** argv)
{
	// Bottle poses
    std::vector<Vector3D<> > vectors;
    
    std::mt19937 generator(5);
    std::uniform_real_distribution<float> uniform_distributionX(-0.3, 0.3);
    std::uniform_real_distribution<float> uniform_distributionY(0.4, 0.5);
    
    int num_scenes = 50;

    for (int i = 0; i < num_scenes; i++)
    {
        vectors.push_back(Vector3D<>( uniform_distributionX(generator), uniform_distributionY(generator), 0.21));
        std::cout << vectors[i] << std::endl;
    }

    RPY<> R1 = RPY<>(-1.571, 0, 1.571);

    std::vector<rw::math::Transform3D<> > bottle_transformations;
    rw::math::Transform3D<> temp = Transform3D<>(vectors[0], R1.toRotation3D());

    for (size_t i = 0; i < vectors.size(); i++)
    {
        temp = Transform3D<>(vectors[i], R1.toRotation3D());
        bottle_transformations.push_back(temp);
    }
	std::vector<rw::math::Transform3D<> > poses_actual = makePointCloudFromScene(bottle_transformations);


    for (float std = 0.0; std < 0.01; std += 0.0005)


	// //std::vector<Matrix4f> pose = poseEstimatePCL("bottle.ply", "scene_clouds/cloud_scene0.pcd");
	// //poseEstimatePCL("bottle2.ply", "scene_clouds/cloud_scene3.pcd");

    // //Matrix4f pose;    
    // for (size_t i = 0; i < bottle_transformations.size(); i++)
    // {
    //     std::vector<Matrix4f> pose = poseEstimatePCL("bottle2.ply", "scene_clouds/cloud_scene" + std::to_string(i) + ".pcd");

    //     calcErrorOnPose(bottle_transformations[i], pose);

    // }
	

	return 0;
}



