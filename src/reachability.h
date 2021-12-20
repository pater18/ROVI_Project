

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
#include <rw/kinematics/MovableFrame.hpp>
#include <opencv2/opencv.hpp>


#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;

struct bestBasePosition {
	double x;
	double y;
	int	numFreeCollisions = 0;
	int totalSolutions = 0; 
};

struct passedElementsForTest
{
	rw::kinematics::MovableFrame::Ptr base;
	rw::kinematics::MovableFrame::Ptr objectFrame;
	rw::proximity::CollisionDetector::Ptr detector;
	rw::kinematics::Frame* BaseFrame;
	rw::math::Transform3D<> baseTrans;
	rw::math::Vector3D<> testBasePosition;
	rw::math::Vector3D<> testObjectPosition;
	rw::kinematics::Frame* refframe;
};

struct testParam
{
	bool newBasePos = true;
	int numberOfSolutions = 0;
	bool canReachPlaceArea = 0;
	int solutionsForPickArea = 0; 
}; 
	

std::vector<rw::math::Q> getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << std::endl;
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

void testBasePlacement(	rw::models::WorkCell::Ptr wc, rw::models::SerialDevice::Ptr robotUR6, passedElementsForTest vals, 
						bestBasePosition &basePos, std::string object, TimedStatePath &tStatePath, double &time, testParam &test){


	State state = wc->getDefaultState();
	std::vector<rw::math::Q> collisionFreeSolutions;
	vals.base->moveTo(rw::math::Transform3D<>(vals.testBasePosition, vals.baseTrans.R()), vals.BaseFrame, state);
	vals.objectFrame->moveTo(rw::math::Transform3D<>(vals.testObjectPosition, vals.objectFrame->getTransform(state).R() ), vals.refframe, state);
	
	bool validSolution = false;

	//Check for solutions at the pick area
	std::vector<rw::math::Q> solutions = getConfigurations(object, "GraspTCP", robotUR6, wc, state);
	for(unsigned int i=0; i<solutions.size(); i++){
		// set the robot in that configuration and check if it is in collision
		robotUR6->setQ(solutions[i], state);
		if( !vals.detector->inCollision(state,NULL,true) ){
			collisionFreeSolutions.push_back(solutions[i]); // save it
			test.solutionsForPickArea++;
			validSolution = true;
			//break; // we only need one
		}
	}

	if (validSolution){
		test.numberOfSolutions++;
	}

	if (test.newBasePos) {
		//Check for solutions at the place-up area
		solutions = getConfigurations("Square2", "GraspTCP", robotUR6, wc, state);
		for(unsigned int i=0; i<solutions.size(); i++){
			// set the robot in that configuration and check if it is in collision
			robotUR6->setQ(solutions[i], state);
			if( !vals.detector->inCollision(state,NULL,true) ){
				collisionFreeSolutions.push_back(solutions[i]); // save it
				test.canReachPlaceArea = true;
				break; // we only need one
			}
		}
	}

	if (!collisionFreeSolutions.empty()/* && basePos.numFreeCollisions < (int)collisionFreeSolutions.size()*/){
		basePos.x = vals.testBasePosition(0);
		basePos.y = vals.testBasePosition(1);
		basePos.numFreeCollisions = collisionFreeSolutions.size();


		for(unsigned int i=0; i<collisionFreeSolutions.size(); i++){
			robotUR6->setQ(collisionFreeSolutions[i], state);
			tStatePath.push_back(TimedState(time,state));
			time+=0.01;
		}
		rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "../visu.rwplay");
	}

}

void generateHeatmap(std::vector<testParam> test){


	int differentValues = 0;
	for (auto val : test){
		if (val.solutionsForPickArea > differentValues)
			differentValues = val.solutionsForPickArea;
	}

	float divident = 255.0f / (float)(differentValues / 2.0f);
	//std::cout << "Divident = "<< divident << std::endl; 
	
	std::vector<cv::Vec3b> colors1 ={ cv::Vec3b(0,0,255), cv::Vec3b(0,63,255), cv::Vec3b(0,95,255), cv::Vec3b(0,123,255), cv::Vec3b(0,153,255), 
	cv::Vec3b(0,183,255), cv::Vec3b(0,212,255), cv::Vec3b(0,242,255), cv::Vec3b(0,255,238), 
	cv::Vec3b(0,255,212), cv::Vec3b(0,255,182), cv::Vec3b(0,255,153), cv::Vec3b(0,255,123), 
	cv::Vec3b(0,255,93), cv::Vec3b(0,255,63), cv::Vec3b(0,255,30), cv::Vec3b(0,255,0)};
	
	// std::vector<cv::Vec3b> colors;
	// colors.push_back(cv::Vec3b(0,0,255));
	// int G = 0;
	// int R = 255;
	
	//If there are more than 16 different solutions.
	// for(int i = 0; i < differentValues; i++) {
	// 	//int B = 255;
	// 	G += divident;
	// 	if (G > 255 || i > differentValues/2) {
	// 		G = 255;
	// 		R -= divident;
	// 	}
	// 	if (R < 0){
	// 		R = 0;
	// 	}
	// 	//std::cout << R << "  " << G << std::endl;
	// 	colors.push_back(cv::Vec3b(0,G,R));
	// }


	cv::Mat table = cv::imread("../Template/Table.png");
	cv::Mat heatmap; 
	cv::copyTo(table, heatmap, cv::noArray());
	//x start = 145 pixels, xslut 650
	int boxSizeX = (650 - 145) / 13;
	int boxSizeY = table.rows / 13;  
	int countX = 0;
	int countY = 0;
	divident /= 2;
	std::cout << "Box x = " << boxSizeX << " Box y = " << boxSizeY << " test size " << test.size() <<  std::endl;
	
	for (size_t i = 0; i < test.size(); i++) {
		int xstart = countX * boxSizeX + 145;
		int xend = xstart + boxSizeX;

		int ystart = countY * boxSizeY;
		int yend = ystart + boxSizeY;

		if (!test[i].canReachPlaceArea || test[i].numberOfSolutions == 0 )
		{
			for (int x = xstart; x < xend - 1; x ++){
	 			for (int y = ystart; y < yend - 1 ; y++ ){
					heatmap.at<cv::Vec3b>(y, x) = colors1[0]; // If the place area or the pickup area has no solutions color it red
				}
			}
			// Write the percent on the image
			std::string percent = std::to_string(0);
			percent.push_back('%');
			cv::Point pt = cv::Point(xstart + 15, yend - 20);
			cv::putText(heatmap, percent, pt, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(255,0,0), 1);
		} else {
			for (int x = xstart; x < xend - 1; x ++){
	 			for (int y = ystart; y < yend -1; y++ ){
					heatmap.at<cv::Vec3b>(y, x) = colors1[test[i].numberOfSolutions]; 
				}
			};

			// Write the percent on the image
			double usefull1 = (double)((test[i].numberOfSolutions / 16.0) * 100.0);
			std::string percent = std::to_string((int)usefull1) ;
			percent.push_back('%');
			cv::Point pt = cv::Point(xstart + 1, yend - 30);
			cv::putText(heatmap, percent, pt, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(255,0,0), 1);

			// Write the number of solutions for each pick up location
			percent = std::to_string(test[i].solutionsForPickArea) ;
			pt = cv::Point(xstart + 15, yend - 10);
			cv::putText(heatmap, percent, pt, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(255,0,0), 1);
		}

		// Used to draw the boxes in image
		countX++;
		if (countX > 12){
			countY++;
			countX = 0;
		}
	}

	double alpha = 0.5;
	double beta = (1.0 - alpha); 
	double gamma = 0.0;
	cv::Mat dist;
	cv::imshow("Table", heatmap);
	cv::addWeighted(heatmap,alpha, table, beta, gamma, dist);

	// Draw a black border around the image
	for (int i = 0; i < dist.rows; i++) {
		for(int j = 0; j < dist.cols; j++){
			dist.at<cv::Vec3b>(i, 0) = cv::Vec3b(0,0,0); 
			dist.at<cv::Vec3b>(0, j) = cv::Vec3b(0,0,0); 
			dist.at<cv::Vec3b>(dist.rows - 1,j) = cv::Vec3b(0,0,0); 
			dist.at<cv::Vec3b>(i, dist.cols - 1) = cv::Vec3b(0,0,0); 
		}
	}
	
	cv::putText(dist, "Pick Area", cv::Point(680, 300),cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(255,0,0), 2);
	cv::putText(dist, "Place Area", cv::Point(1, 525),cv::FONT_HERSHEY_DUPLEX, .8, cv::Scalar(255,0,0), 2);


	cv::imshow("Heatmap combined", dist);
	cv::imwrite("Heatmap_grip_Top.png", dist);
	cv::waitKey(0);
	cv::destroyAllWindows();
	
}

void testimage(){


	std::vector<cv::Vec3b> colors ={ cv::Vec3b(0,0,255), cv::Vec3b(0,63,255), cv::Vec3b(0,95,255), cv::Vec3b(0,123,255), cv::Vec3b(0,153,255), 
	cv::Vec3b(0,183,255), cv::Vec3b(0,212,255), cv::Vec3b(0,242,255), cv::Vec3b(0,255,238), 
	cv::Vec3b(0,255,212), cv::Vec3b(0,255,182), cv::Vec3b(0,255,153), cv::Vec3b(0,255,123), 
	cv::Vec3b(0,255,93), cv::Vec3b(0,255,63), cv::Vec3b(0,255,30), cv::Vec3b(0,255,0)};


	cv::Mat table = cv::imread("../Template/Table.png");
	cv::Mat heatmap; 
	cv::copyTo(table, heatmap, cv::noArray());
	int boxSizeY = table.rows / 13;  
	for (int count = 0; count < 13; count++) {
		int ystart = boxSizeY * count;
		int yend = ystart + boxSizeY;
		for (int y = ystart; y < yend ; y++)
			heatmap.at<cv::Vec3b>(y, 145) = colors[0];



		cv::imshow("Heatmap", heatmap);
		cv::waitKey(0);
	}
	cv::destroyAllWindows();

}

void reach_object()
{

	//load workcell
	rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../Scene.wc.xml");
	if(NULL==wc){
		RW_THROW("COULD NOT LOAD scene... check path!");
		return ;
	} else {
        std::cout << "Workcell is loaded" << std::endl;
    }


	rw::models::SerialDevice::Ptr robotUR6 = wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
	if(NULL==robotUR6){
		RW_THROW("COULD not find device UR5 ... check model");
		return ;
	}


	// get the default state
	State state = wc->getDefaultState();

	std::string bottle = "Bottle";
	std::string square = "Square";
	std::string cylinder = "Cylinder";
	std::string object = square;


	bestBasePosition basePos; 
	TimedStatePath tStatePath;
	double time = 0;

	rw::kinematics::Frame* BaseFrame = robotUR6->getBase();
	rw::kinematics::Frame* refframe = wc->findFrame<rw::kinematics::FixedFrame>("Table");
	rw::math::Transform3D<> baseTrans = BaseFrame -> getTransform(state);
	rw::kinematics::MovableFrame::Ptr base = wc->findFrame<rw::kinematics::MovableFrame>("UR-6-85-5-A.Base"); 
	rw::kinematics::MovableFrame::Ptr objectFrame = wc->findFrame<rw::kinematics::MovableFrame>(object);
	rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));


	passedElementsForTest testVariables;
	testVariables.base = base;
	testVariables.BaseFrame = BaseFrame;
	testVariables.baseTrans = baseTrans;
	testVariables.detector = detector;
	testVariables.objectFrame = objectFrame;
	testVariables.refframe = refframe;


	testParam test;

	std::vector<testParam> testValues;
	std::ofstream heatmap;
	//heatmap.open("heatmap.csv");
	//heatmap << "Base x, Base y, Solutions for Pickarea, Can reach place area\n";
	float count = 1;
	for (double x = -0.3; x <= 0.3; x += 0.05){ // 12
		for (double y = -0.3; y <= 0.3; y += 0.05){ // 12
			testVariables.testBasePosition = rw::math::Vector3D<>(x, y, 0.01);
			
			test.canReachPlaceArea = false;
			test.newBasePos = true;
			test.numberOfSolutions = 0;
			test.solutionsForPickArea = 0;

			for (double obj_y = 0.358; obj_y <= 0.527; obj_y += 0.169 ){ // 2
				for (double obj_x = -0.35; obj_x <= 0.36; obj_x += 0.1 ){ // 8
					testVariables.testObjectPosition = rw::math::Vector3D<>(obj_x, obj_y, 0.21);
					
					// GraspTCP <RPY>0 0 -90</RPY> takes the object from the top
					testBasePlacement(wc, robotUR6, testVariables, basePos, object, tStatePath, time, test);
					test.newBasePos = false;
					float finished = (count++ / 2704.0f) * 100.0f; 
					if ((int)finished % 10 == 0 || finished > 95.0)
						std::cout << "Caluculation is " << finished << "% done" << std::endl;
					
					// std::cout << "[ x = " << x << " y = " << y << " ]" << std::endl;
					// std::cout << "[ obj_x = " << obj_x << " obj_y = " << obj_y << " ]\n" << std::endl;
				}
			}

			testValues.push_back(test);

			// heatmap << x << "," << y << "," << test.solutionsForPickArea << "," << test.canReachPlaceArea << std::endl;
		}
	}
	generateHeatmap(testValues);
	//heatmap.close();
	std::cout << "Base position\n" << "[" << basePos.x << " " << basePos.y << "]" << " with collision free solutions : " << basePos.numFreeCollisions << std::endl;

}

