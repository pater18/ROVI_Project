

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

int main(int argc, char** argv)
{
	// Bottle poses
    std::vector<Vector3D<> > vectors;
    vectors.push_back(Vector3D<>(-0.30, 0.40, 0.21));
    vectors.push_back(Vector3D<>(-0.30, 0.50, 0.21));
    vectors.push_back(Vector3D<>(-0.10, 0.40, 0.21));
    vectors.push_back(Vector3D<>(-0.10, 0.50, 0.21));
    vectors.push_back(Vector3D<>( 0.10, 0.40, 0.21));
    vectors.push_back(Vector3D<>( 0.10, 0.50, 0.21));
    vectors.push_back(Vector3D<>( 0.30, 0.40, 0.21));
    vectors.push_back(Vector3D<>( 0.30, 0.50, 0.21));

    RPY<> R1 = RPY<>(-1.571, 0, 1.571);

    std::vector<rw::math::Transform3D<> > bottle_transformations;
    rw::math::Transform3D<> temp = Transform3D<>(vectors[0], R1.toRotation3D());

    for (size_t i = 0; i < vectors.size(); i++)
    {
        temp = Transform3D<>(vectors[i], R1.toRotation3D());
        bottle_transformations.push_back(temp);
    }

	//std::vector<rw::math::Transform3D<> > poses_actual = getPoseWithDenseStereo();
	//Matrix4f pose = poseEstimatePCLAnders("bottle.ply", "scene_clouds/cloud_scene3.pcd");
	poseEstimatePCL("bottle2.ply", "scene_clouds/cloud_scene3.pcd");

    Matrix4f pose;    
    for (size_t i = 0; i < bottle_transformations.size(); i++)
    {
        calcErrorOnPose(bottle_transformations[i], pose);

    }
	

	return 0;
}



