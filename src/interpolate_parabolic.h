
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
#include <rw/trajectory/Trajectory.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rwlibs/pathoptimization/PathLengthOptimizer.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/RampInterpolator.hpp>


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

	rw::kinematics::Frame *frameTcp = wc->findFrame("GraspTCP");

	rw::kinematics::Frame *frameRobotBase = wc->findFrame(nameRobotBase);
	rw::kinematics::Frame *frameRobotTcp = wc->findFrame(nameRobotTcp);
	if (frameGoal == NULL || frameTcp == NULL || frameRobotBase == NULL || frameRobotTcp == NULL)
	{
		std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
		std::cout << " Found \"" << frameGoal->getName() << "\": " << (frameGoal == NULL ? "NO!" : "YES!") << std::endl;
		std::cout << " Found \"" << frameTcp->getName() << "\": " << (frameTcp == NULL ? "NO!" : "YES!") << std::endl;
		std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase == NULL ? "NO!" : "YES!") << std::endl;
		std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp == NULL ? "NO!" : "YES!") << std::endl;
	}

	// Make "helper" transformations
	rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
	rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

	// get grasp frame in robot tool frame
	rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

	rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR(robot, state));
	return closedFormSovler->solve(targetAt, state);
}

void interpolateBetweenPoints(rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_points, QPath &qpath_interpolated, rw::models::SerialDevice::Ptr robot, rw::trajectory::TimedStatePath &tStatePath, double &current_time, rw::kinematics::State state)
{

	for (float i = 0.0; i < interpolated_points.duration(); i += 0.01)
	{
		robot->setQ(interpolated_points.x(i), state);
		qpath_interpolated.push_back(interpolated_points.x(i));
		tStatePath.push_back(rw::trajectory::TimedState(current_time, state));
		current_time += 0.01;
	}
}
void interpolateBetweenPointsPara(rw::trajectory::ParabolicBlend<rw::math::Q> interpolated_points, QPath &qpath_interpolated, rw::models::SerialDevice::Ptr robot, rw::trajectory::TimedStatePath &tStatePath, double &current_time, rw::kinematics::State state)
{

	for (float i = 0.0; i < interpolated_points.duration(); i += 0.01)
	{
		robot->setQ(interpolated_points.x(i), state);
		qpath_interpolated.push_back(interpolated_points.x(i));
		tStatePath.push_back(rw::trajectory::TimedState(current_time, state));
		current_time += 0.01;
	}
}

int interpolatePara()
{

	// Bottle poses
	std::vector<Vector3D<>> vectors;
	vectors.push_back(Vector3D<>(0.3, 0.473, 0.110));
	vectors.push_back(Vector3D<>(0.0, 0.473, 0.110));
	vectors.push_back(Vector3D<>(-0.15, 0.473, 0.110));

	RPY<> R1 = RPY<>(-1.574, 0, 1.571);

	std::vector<rw::math::Transform3D<>> bottle_placements;
	rw::math::Transform3D<> temp = Transform3D<>(vectors[0], R1.toRotation3D());

	for (size_t i = 0; i < vectors.size(); i++)
	{
		temp = Transform3D<>(vectors[i], R1.toRotation3D());
		bottle_placements.push_back(temp);
	}

	rw::math::Transform3D<> end_transform = rw::math::Transform3D<>(rw::math::Vector3D(0.3, -0.502, 0.21), rw::math::Rotation3D(-0.00320367, -0.000203672, -0.999995, -0.999995, 6.52501e-07, 0.00320367, -0.0, 1.0, -0.000203673));

	std::vector<double> placement1_total_times;
	std::vector<double> placement2_total_times;
	std::vector<double> placement3_total_times;
	std::vector<double> placement1_total_length;
	std::vector<double> placement2_total_length;
	std::vector<double> placement3_total_length;
	for (int k = 0; k < 50; k++)
	{
		for (size_t i = 0; i < bottle_placements.size(); i++)
		{
			//load workcell
			rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../scene_no_obstacle.wc.xml");

			if (NULL == wc)
			{
				RW_THROW("COULD NOT LOAD scene... check path!");
				return -1;
			}

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

			rw::math::Transform3D<> temp_transform = bottleFrame->getTransform(state);
			bottleFrame->setTransform(end_transform, state);

			std::vector<rw::math::Q> solutions_end = getQConfigs(bottleFrame, robotUR5, wc, state);

			bottleFrame->setTransform(temp_transform, state);

			double total_time = 0;
			double total_length = 0;

			QPath qpath_interpolated;
			Gripper->setQ(rw::math::Q(0.055), state);

			rw::math::Q start_q = robotUR5->getQ(state);
			rw::math::Q interpolation_step1 = rw::math::Q(1.5708, -1.91986, -0.349066, -1.5708, 0, 0);
			rw::math::Q interpolation_step2 = rw::math::Q(1.8, -2.03, -1.998, -2.36, 0.09, -3.139);
			rw::math::Q interpolation_step4 = rw::math::Q(1.5708, -1.91986, -0.349066, -1.5708, 0, 0);
			rw::math::Q interpolation_step5 = start_q;
			std::vector<rw::math::Q> solutions = getQConfigs(bottleFrame, robotUR5, wc, state);

			rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

			std::vector<rw::math::Q> collisionFreeSolutions;

			for (unsigned int i = 0; i < solutions.size(); i++)
			{
				// set the robot in that configuration and check if it is in collision
				robotUR5->setQ(solutions[i], state);
				if (!detector->inCollision(state, NULL, true))
				{
					collisionFreeSolutions.push_back(solutions[i]); // save it
																	// we only need one
				}
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

			std::cout << "Current position of the robot vs object to be grasped has: "
					  << collisionFreeSolutions.size()
					  << " collision-free inverse kinematics solutions!" << std::endl;

			if (!collisionFreeSolutions.empty())
			{
				rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_pointsSt1(start_q, interpolation_step1, 1);
				rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_points1t2(interpolation_step1, interpolation_step2, 1);
				rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_points2tBottle(interpolation_step2, collisionFreeSolutions[0], 1);
				rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_pointsBottlet4(collisionFreeSolutions[0], interpolation_step4, 1);
				rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_points4t5(interpolation_step4, interpolation_step5, 1);
				rw::trajectory::LinearInterpolator<rw::math::Q> interpolated_points5tEnd(interpolation_step5, collisionFreeSolutionsEnd[0], 1);

                rw::trajectory::ParabolicBlend<rw::math::Q> para1(interpolated_pointsSt1, interpolated_points1t2, 1);
                rw::trajectory::ParabolicBlend<rw::math::Q> para2(interpolated_pointsBottlet4, interpolated_points4t5, 1);
                rw::trajectory::ParabolicBlend<rw::math::Q> para3(interpolated_points4t5, interpolated_points5tEnd, 1);

				rw::trajectory::TimedStatePath tStatePath;
				double time = 0;

				interpolateBetweenPointsPara(para1, qpath_interpolated, robotUR5, tStatePath, time, state);
				//interpolateBetweenPoints(interpolated_pointsSt1, qpath_interpolated, robotUR5, tStatePath, time, state);
				//interpolateBetweenPoints(interpolated_points1t2, qpath_interpolated, robotUR5, tStatePath, time, state);
				interpolateBetweenPoints(interpolated_points2tBottle, qpath_interpolated, robotUR5, tStatePath, time, state);

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

				interpolateBetweenPointsPara(para2, qpath_interpolated, robotUR5, tStatePath, time, state);
				//interpolateBetweenPointsPara(para3, qpath_interpolated, robotUR5, tStatePath, time, state);

				// interpolateBetweenPoints(interpolated_pointsBottlet4, qpath_interpolated, robotUR5, tStatePath, time, state);
				// interpolateBetweenPoints(interpolated_points4t5, qpath_interpolated, robotUR5, tStatePath, time, state);
				interpolateBetweenPoints(interpolated_points5tEnd, qpath_interpolated, robotUR5, tStatePath, time, state);

				rw::pathplanning::PathAnalyzer analyzer(robotUR5, state);
				rw::pathplanning::PathAnalyzer::CartesianAnalysis analysis = analyzer.analyzeCartesian(qpath_interpolated, frameTcp);
				rw::pathplanning::PathAnalyzer::TimeAnalysis analysis_time = analyzer.analyzeTime(qpath_interpolated);
				total_length += analysis.length;
				total_time += analysis_time.time1;
				std::cout << "Length of path: " << analysis.length << std::endl;
				std::cout << "Length of total path: " << total_length << std::endl;
				std::cout << "Time for path:" << analysis_time.time1 << std::endl;
				std::cout << "Time for total path: " << total_time << std::endl;

				rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "../replays/visu_linear" + std::to_string(i) + ".rwplay");
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

	saveDataToCSV(placement1_total_times, "../csv_data/P1_para_time.csv");
	saveDataToCSV(placement2_total_times, "../csv_data/P2_para_time.csv");
	saveDataToCSV(placement3_total_times, "../csv_data/P3_para_time.csv");
	saveDataToCSV(placement1_total_length, "../csv_data/P1_para_length.csv");
	saveDataToCSV(placement2_total_length, "../csv_data/P2_para_length.csv");
	saveDataToCSV(placement3_total_length, "../csv_data/P3_para_length.csv");

	return 0;
}