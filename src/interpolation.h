
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


#include "reachability.h"

#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;


std::vector<rw::math::Q> getQConfigs(const std::string nameGoal, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence

    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
	rw::kinematics::Frame* frameTcp = wc->findFrame("GraspTCP");

    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << frameGoal->getName() << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << frameTcp->getName() << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);
}


void printDeviceNames(const WorkCell& workcell)
{
	std::cout << "Workcell " << workcell << " contains devices:" << std::endl;
	for(const Device::CPtr device : workcell.getDevices()) 
	{
		std::cout << "- " << device->getName() << std::endl;
    }
}

void interpolateBetweenPoints(rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_points, rw::models::SerialDevice::Ptr robot, rw::trajectory::TimedStatePath &tStatePath, double &current_time, rw::kinematics::State state)
{
	for(float i=0.0; i<interpolated_points.duration(); i += 0.01)
		{
			robot->setQ(interpolated_points.x(i), state);
			tStatePath.push_back(rw::trajectory::TimedState(current_time, state));
			current_time += 0.01;
		}
}

int interpolateLinear()
{
	//load workcell
	rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../Scene.wc.xml");
	printDeviceNames(*wc);

	if(NULL==wc){
		RW_THROW("COULD NOT LOAD scene... check path!");
		return -1;
	}

	// find relevant frames
	rw::kinematics::Frame* cylinderFrame = wc->findFrame("Cylinder");
	if(NULL==cylinderFrame){
		RW_THROW("COULD not find movable frame Cylinder ... check model");
		return -1;
	}	
	
	rw::kinematics::Frame* squareFrame = wc->findFrame("Square");
	if(NULL==cylinderFrame){
		RW_THROW("COULD not find movable frame Cylinder ... check model");
		return -1;
	}

	rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
	if(NULL==robotUR5){
		RW_THROW("COULD not find device UR5 ... check model");
		return -1;
	}

	rw::models::TreeDevice::Ptr Gripper = wc->findDevice<rw::models::TreeDevice>("WSG50");
	if(NULL==Gripper){
		RW_THROW("COULD not find device Gripper ... check model");
		return -1;
	}
	
	State state = wc->getDefaultState();
	Gripper->setQ(rw::math::Q(0.055), state);

	rw::math::Q start_q = robotUR5->getQ(state);
	rw::math::Q interpolation_step1 = rw::math::Q(0.824267, -2.68465, -0.831859, -2.76666, 2.39506, -0);
	
	rw::math::Q interpolation_step2 = rw::math::Q(0.39264, 0.46807, 0.21718, -1.55226, 0.159715, 1.80128);
	rw::math::Q interpolation_step3 = rw::math::Q(0.39264, 0.46807, 0.21718, -1.55226, 0.159715, 1.80128);

	
	// std::vector<rw::math::Q> solutions = getQConfigs("Square", robotUR5, wc, state);


	// rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
	// std::vector<rw::math::Q> collisionFreeSolutions;


	for(unsigned int i=0; i<solutions.size(); i++)
	{
		// set the robot in that configuration and check if it is in collision
		robotUR5->setQ(solutions[i], state);
		if( !detector->inCollision(state,NULL,true) )
		{
			collisionFreeSolutions.push_back(solutions[i]); // save it
			 // we only need one
		}
	}
	
	std:: cout << "Current position of the robot vs object to be grasped has: "
			   << collisionFreeSolutions.size()
			   << " collision-free inverse kinematics solutions!" << std::endl;


	if (!collisionFreeSolutions.empty())
	{
		rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_pointsSt1(start_q, interpolation_step1, 2);
		rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_points1tE(interpolation_step1, collisionFreeSolutions[0], 1);
		rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_pointsEt1(collisionFreeSolutions[0], interpolation_step1, 1);
		rw::trajectory::TimedStatePath tStatePath;
		double time = 0;


		interpolateBetweenPoints(interpolated_pointsSt1, robotUR5, tStatePath, time, state);
		interpolateBetweenPoints(interpolated_points1tE, robotUR5, tStatePath, time, state);


		rw::kinematics::Frame* frameRobotTcp = wc->findFrame(robotUR5->getName() + ".TCP");
		rw::kinematics::Frame* frameSquare = wc->findFrame("Square");
		if (frameSquare == NULL || frameRobotTcp == NULL)
		{
			std::cout << "One of the frames not found!!!" << std::endl;
		}
	    if( Kinematics::isDAF( frameSquare ) )
		{
       		// attach mframe to end of serial device
       		Kinematics::gripFrame(frameSquare, robotUR5->getEnd(), state);
    	}

		interpolateBetweenPoints(interpolated_pointsEt1, robotUR5, tStatePath, time, state);





		
		rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "../replays/visu.rwplay");
		std::cout << "Saved a replay with size: " 
				  << tStatePath.size() 
				  << " in folder: /replays" << std::endl;
	}



	return 0;
}



