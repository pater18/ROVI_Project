
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

//#include "RRT_planning.h"
//#include "interpolate_linear.h"
#include "interpolate_parabolic.h"

#include <iostream>
#include <string>

//USE_ROBWORK_NAMESPACE
//using namespace robwork;


int main(int argc, char** argv)
{
    //rrtPlanning();
    interpolatePara();
    //interpolateLinear();
	return 0;

}




