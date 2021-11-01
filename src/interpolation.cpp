
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



int main(int argc, char** argv)
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

	// rw::models::TreeDevice* Gripper = wc->findDevice<rw::models::TreeDevice>("WSG50");
	// rw::kinematics::Frame* grip = wc->findFrame("Gripper")
    // if (NULL==Gripper)
	// {
    //     RW_THROW("COULD not find tree device WSG50 ... check model");
	// 	return -1;
	// }

    // Gripper.setQ(sdurw.Q(0.055), state)

	
	
	std::vector<rw::math::Q> solutions = getQConfigs("Square", robotUR5, wc, state);


	rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
	std::vector<rw::math::Q> collisionFreeSolutions;



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


	// // visualize them
	// TimedStatePath tStatePath;
	// double time=0;
	// for(unsigned int i=0; i<collisionFreeSolutions.size(); i++){
	// 	robotUR5->setQ(collisionFreeSolutions[i], state);
	// 	tStatePath.push_back(TimedState(time,state));
	// 	time+=0.01;
	// }

	// rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "../scene/visu.rwplay");

	return 0;
}



