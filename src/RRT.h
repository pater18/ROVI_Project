
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
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/math/Metric.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rwlibs/pathoptimization/PathLengthOptimizer.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>

#include "saveToCSV.h"

#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rw::pathplanning;



std::vector<rw::math::Q> getQConfigs(rw::kinematics::MovableFrame::Ptr frameGoal, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence

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

void RRTp2p(rw::math::Q from, rw::math::Q to, double &total_length, double &total_time, rw::models::SerialDevice::Ptr robot, rw::proximity::CollisionDetector::Ptr detector, rw::trajectory::TimedStatePath &tStatePath, double &current_time, rw::kinematics::State state, rw::kinematics::Frame* frameTcp)
{
	typedef rw::math::Q V;
	typedef Ptr<Metric<V>> VMetricPtr;
	const PlannerConstraint con = PlannerConstraint::make(detector, robot, state);

	rw::core::Ptr<QSampler> sampler = rw::pathplanning::QSampler::makeUniform(robot);
	VMetricPtr metric = MetricFactory::makeEuclidean<V>();
	//const QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner (con, sampler, metric, 2, rwlibs::pathplanners::RRTPlanner::PlannerType::RRTConnect);
	const QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(con, robot, rwlibs::pathplanners::RRTPlanner::PlannerType::RRTConnect);

	ProximityData pdata;
	robot->setQ(from, state);
	if (detector->inCollision(state, pdata))
		RW_THROW("Initial configuration in collision! can not plan a path.");
	robot->setQ(to, state);
	if (detector->inCollision(state, pdata))
		RW_THROW("Final configuration in collision! can not plan a path.");


	QPath rrt_points;

	if (planner->query(from, to, rrt_points))
	{
		std::cout << "Planned path with " << rrt_points.size();
		std::cout << " Q configurations" << std::endl;
	}

	

	rwlibs::pathoptimization::PathLengthOptimizer optimizer(con, metric);
	rrt_points = optimizer.shortCut(rrt_points);	
	//rrt_points = optimizer.pathPruning(rrt_points);	

	
	rw::pathplanning::PathAnalyzer analyzer(robot, state);
	rw::pathplanning::PathAnalyzer::CartesianAnalysis analysis = analyzer.analyzeCartesian(rrt_points, frameTcp);
	rw::pathplanning::PathAnalyzer::TimeAnalysis analysis_time = analyzer.analyzeTime(rrt_points);
	total_length += analysis.length;
	total_time += analysis_time.time1;
	std::cout <<"Length of path: " <<  analysis.length << std::endl;
	std::cout <<"Length of total path: " <<  total_length << std::endl;
	std::cout <<"Time for path:" << analysis_time.time1 << std::endl;
	std::cout <<"Time for total path: " << total_time << std::endl;



	// for (int i = 0; i < result.size(); i++)
	// {
	// 	std::cout << result[i] << std::endl;
	// }

	for (size_t i = 0; i < rrt_points.size() - 1; i++)
	{
		rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_points(rrt_points[i], rrt_points[i + 1], 0.05);
		interpolateBetweenPoints(interpolated_points, robot, tStatePath, current_time, state);
	}


}

int rrtPlanning(std::vector<Eigen::Matrix4f> bottle_positions_esti)
{
	// Bottle poses
    std::vector<Vector3D<> > vectors;
    vectors.push_back(Vector3D<>(0.3, 0.473, 0.110));
    vectors.push_back(Vector3D<>(0.0, 0.473, 0.110));
    vectors.push_back(Vector3D<>(-0.15, 0.473, 0.110));
    // vectors.push_back(Vector3D<>(-0.30, 0.50, 0.21));
    // vectors.push_back(Vector3D<>(-0.10, 0.40, 0.21));
    // vectors.push_back(Vector3D<>(-0.10, 0.50, 0.21));
    // vectors.push_back(Vector3D<>( 0.10, 0.40, 0.21));
    // vectors.push_back(Vector3D<>( 0.10, 0.50, 0.21));
    // vectors.push_back(Vector3D<>( 0.30, 0.40, 0.21));
    // vectors.push_back(Vector3D<>( 0.30, 0.50, 0.21));

    RPY<> R1 = RPY<>(-1.574, 0, 1.571);

    std::vector<rw::math::Transform3D<> > bottle_placements;
    rw::math::Transform3D<> temp = Transform3D<>(vectors[0], R1.toRotation3D());

    // for (size_t i = 0; i < vectors.size(); i++)
    // {
    //     temp = Transform3D<>(vectors[i], R1.toRotation3D());
    //     bottle_placements.push_back(temp);
    // }    
    for (size_t i = 0; i < bottle_positions_esti.size(); i++)
    {
        temp = Transform3D<>(bottle_positions_esti[i]);
        bottle_placements.push_back(temp);
    }

	std::vector<double> placement1_total_times;
	std::vector<double> placement2_total_times;
	std::vector<double> placement3_total_times;
	std::vector<double> placement1_total_length;
	std::vector<double> placement2_total_length;
	std::vector<double> placement3_total_length;
	int num_experiments = 50;
	for (int k = 0; k < num_experiments; k++)
	{
		std::cout << "This is K-iteration " << k + 1 << " / " << num_experiments << std::endl;
		for (size_t i = 0; i < bottle_placements.size(); i++)
		{

			std::cout << i + 1 << "'st Bottle position!!" << std::endl
					  << std::endl;

			//load workcell
			rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../Scene.wc.xml");
			//printDeviceNames(*wc);

			if (NULL == wc)
			{
				RW_THROW("COULD NOT LOAD scene... check path!");
				return -1;
			}

			// find relevant frames
			rw::kinematics::MovableFrame::Ptr bottleFrame = wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
			if (NULL == bottleFrame)
			{
				RW_THROW("COULD not find movable frame Bottle ... check model");
				return -1;
			}

			rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
			if (NULL == robotUR5)
			{
				RW_THROW("COULD not find device UR5 ... check model");
				return -1;
			}

			rw::models::TreeDevice::Ptr Gripper = wc->findDevice<rw::models::TreeDevice>("WSG50");
			if (NULL == Gripper)
			{
				RW_THROW("COULD not find device Gripper ... check model");
				return -1;
			}
			rw::kinematics::Frame *frameTcp = wc->findFrame("GraspTCP");

			State state = wc->getDefaultState();
			bottleFrame->setTransform(bottle_placements[i], state);
			bottleFrame->moveTo(bottle_placements[i], state);

			Gripper->setQ(rw::math::Q(0.055), state);
			double total_length = 0.0;
			double total_time = 0.0;

			rw::math::Q start_q = robotUR5->getQ(state);
			rw::math::Transform3D<> end_transform = rw::math::Transform3D<>(rw::math::Vector3D(0.3, -0.502, 0.21), rw::math::Rotation3D(-0.00320367, -0.000203672, -0.999995, -0.999995, 6.52501e-07, 0.00320367, -0.0, 1.0, -0.000203673));

			std::vector<rw::math::Q> solutions = getQConfigs(bottleFrame, robotUR5, wc, state);

			std::vector<std::string> strategies = rwlibs::proximitystrategies::ProximityStrategyFactory::getDistanceStrategyIDs();
			// for (size_t i = 0; i < strategies.size(); i++)
			// {
			// 	std::cout << strategies[i] << std::endl;
			// }
			rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
			//rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP")));

			std::vector<rw::math::Q> collisionFreeSolutions;

			for (unsigned int j = 0; j < solutions.size() - 1; j++)
			{
				// set the robot in that configuration and check if it is in collision
				robotUR5->setQ(solutions[j], state);
				if (!detector->inCollision(state, NULL, true))
				{
					collisionFreeSolutions.push_back(solutions[j]); // save it
																	// we only need one
				}
			}

			std::cout << "Pickup position of the robot vs object to be grasped has: "
					  << solutions.size()
					  << " Q-pose solutions of which: "
					  << collisionFreeSolutions.size()
					  << " are collision-free inverse kinematics solutions!" << std::endl;

			if (!collisionFreeSolutions.empty())
			{

				rw::trajectory::TimedStatePath tStatePath;
				double time = 0;

				RRTp2p(start_q, collisionFreeSolutions[0], total_length, total_time, robotUR5, detector, tStatePath, time, state, frameTcp);
				rw::math::Transform3D<> temp_transform = bottleFrame->getTransform(state);
				bottleFrame->setTransform(end_transform, state);

				std::vector<rw::math::Q> solutions_end = getQConfigs(bottleFrame, robotUR5, wc, state);

				bottleFrame->setTransform(temp_transform, state);

				rw::kinematics::Frame *frameRobotTcp = wc->findFrame(robotUR5->getName() + ".TCP");
				if (bottleFrame == NULL || frameRobotTcp == NULL)
				{
					std::cout << "One of the frames not found!!!" << std::endl;
				}
				if (Kinematics::isDAF(bottleFrame))
				{
					// attach mframe to end of serial device
					Kinematics::gripFrame(bottleFrame, robotUR5->getEnd(), state);
				}

				std::vector<rw::math::Q> collisionFreeSolutionsEnd;
				for (unsigned int j = 0; j < solutions_end.size() - 1; j++)
				{
					// set the robot in that configuration and check if it is in collision
					robotUR5->setQ(solutions_end[j], state);
					if (!detector->inCollision(state, NULL, true))
					{

						collisionFreeSolutionsEnd.push_back(solutions_end[j]); // save it											// we only need one
					}
				}

				std::cout << "Placement position of the robot vs object to be grasped has: "
						  << solutions_end.size()
						  << " Q-pose solutions of which: "
						  << collisionFreeSolutionsEnd.size()
						  << " are collision-free inverse kinematics solutions!" << std::endl;

				if (!collisionFreeSolutionsEnd.empty())
				{
					RRTp2p(collisionFreeSolutions[0], collisionFreeSolutionsEnd[0], total_length, total_time, robotUR5, detector, tStatePath, time, state, frameTcp);
				}

				rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "../replays/visu" + std::to_string(i) + ".rwplay");
				std::cout << "Saved a replay with size: "
						  << tStatePath.size()
						  << " in folder: /replays" << std::endl;
				std::cout << std::endl
						  << std::endl
						  << std::endl;
				if (i == 0)
				{
					placement1_total_length.push_back(total_length);
					placement1_total_times.push_back(total_time);
				}
				else if (i == 1)	
				{
					placement2_total_length.push_back(total_length);
					placement2_total_times.push_back(total_time);					
				}
				else
				{
					placement3_total_length.push_back(total_length);
					placement3_total_times.push_back(total_time);					
				}
			}
		}
	}

	saveDataToCSV(placement1_total_times, "../csv_data/P1_RRT_time.csv");
	saveDataToCSV(placement2_total_times, "../csv_data/P2_RRT_time.csv");
	saveDataToCSV(placement3_total_times, "../csv_data/P3_RRT_time.csv");
	saveDataToCSV(placement1_total_length, "../csv_data/P1_RRT_length.csv");
	saveDataToCSV(placement2_total_length, "../csv_data/P2_RRT_length.csv");
	saveDataToCSV(placement3_total_length, "../csv_data/P3_RRT_length.csv");

	return 0;
}



