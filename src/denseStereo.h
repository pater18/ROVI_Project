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
#include <rw/sensor/Camera.hpp>
#include <rw/sensor/Scanner.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwlibs/simulation/SimulatedScanner25D.hpp>

#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>
#include "Camera.h"
#include "PCL.h"
//#include "poseEstimate.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>




#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rwlibs::simulation;
using namespace rws;
using namespace rw::math;



int getPoseWithDenseStereo()
{

    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../Scene.wc.xml");
	printDeviceNames(*wc);

	if(NULL==wc)
    {
		RW_THROW("COULD NOT LOAD scene... check path!");
		return -1;
	}

    // MyCamera camera_left("Camera_Left", wc);
    // MyCamera camera_right("Camera_Right", wc);


    Frame* const scanner25d = wc->findFrame ("Scanner25D");
    if (scanner25d == nullptr)
        RW_THROW ("Scanner frame could not be found.");
    const PropertyMap& properties = scanner25d->getPropertyMap ();
    if (!properties.has ("Scanner25D"))
        RW_THROW ("Scanner25D frame does not have Scanner25D property.");
    const std::string parameters = properties.get< std::string > ("Scanner25D");
    std::istringstream iss (parameters, std::istringstream::in);
    double fovy;
    int width;
    int height;
    iss >> fovy >> width >> height;
    std::cout << "Scanner25D properties: fov " << fovy << " width " << width << " height " << height
              << std::endl;


    rw::kinematics::MovableFrame::Ptr bottleFrame = wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
	if(NULL==bottleFrame)
    {
		RW_THROW("COULD not find movable frame bottle ... check model");
		return -1;
	}



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

    State state = wc->getDefaultState ();

    std::cout << bottleFrame->getTransform(state) << std::endl;

    RobWorkStudioApp app ("");
    RWS_START (app)
    {

            RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
            rwstudio->postOpenWorkCell ("../Scene.wc.xml");
            TimerUtil::sleepMs (5000);
            State state_studio = rwstudio->getState();




            const SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
            // const GLFrameGrabber::Ptr framegrabber_left = ownedPtr (new GLFrameGrabber (camera_left.getwidth(), camera_left.getheight(), camera_left.getfovy()));
            // const GLFrameGrabber::Ptr framegrabber_right = ownedPtr (new GLFrameGrabber (camera_right.getwidth(), camera_right.getheight(), camera_right.getfovy()));
            const GLFrameGrabber25D::Ptr framegrabber_depth = ownedPtr (new GLFrameGrabber25D (fovy, width, height));
            // framegrabber_left->init (gldrawer);
            // framegrabber_right->init (gldrawer);
            framegrabber_depth->init (gldrawer);
            // SimulatedCamera::Ptr simcam_left = ownedPtr (new SimulatedCamera ("SimulatedCamera", camera_left.getfovy(), camera_left.getcamera_frame(), framegrabber_left));
            // simcam_left->setFrameRate (100);
            // simcam_left->initialize ();
            // simcam_left->start ();
            // simcam_left->acquire ();
            // SimulatedCamera::Ptr simcam_right = ownedPtr (new SimulatedCamera ("SimulatedCamera", camera_right.getfovy(), camera_right.getcamera_frame(), framegrabber_right));
            // simcam_right->setFrameRate (100);
            // simcam_right->initialize ();
            // simcam_right->start ();
            // simcam_right->acquire ();

            SimulatedScanner25D::Ptr simscanner25d = ownedPtr (new SimulatedScanner25D ("SimulatedScanner25D", scanner25d, framegrabber_depth));
            simscanner25d->setFrameRate(100);
            simscanner25d->open();
            simscanner25d->acquire();
            for (int i = 0; i < 8; i++)
            {
            static const double DT = 0.001;
            const Simulator::UpdateInfo info (DT);
            int cnt     = 0;
            // const Image* img_left;
            // const Image* img_right;

            // while (!simcam_left->isImageReady ()) {
            //     std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
            //     simcam_left->update (info, state);
            //     cnt++;
            // }
            // while (!simcam_right->isImageReady ()) {
            // std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
            // simcam_right->update (info, state);
            // cnt++;
            // }
        
            bottleFrame->moveTo(bottle_transformations[i], state_studio);

            bottleFrame->setTransform(bottle_transformations[i], state_studio);
            std::cout << bottleFrame->getTransform(state_studio) << std::endl;
            TimerUtil::sleepMs (1000);

            rwstudio->setState(state_studio);

            while (!simscanner25d->isScanReady ()) {
            std::cout << "Pointcloud is not ready yet. Iteration " << cnt << std::endl;
            simscanner25d->update (info, state_studio);
            cnt++;
            }

            const rw::geometry::PointCloud pointcloud = simscanner25d->getScan();
            pointcloud.savePCD(pointcloud, "scene_clouds/cloud_scene" + std::to_string(i) + ".pcd");
            simscanner25d->acquire();
    
        }

        
            
        
        // img_left = simcam_left->getImage ();
        // img_left->saveAsPPM ("image_left.ppm");
        // simcam_left->acquire ();
        // img_right = simcam_right->getImage ();
        // img_right->saveAsPPM ("image_right.ppm");
        // simcam_right->acquire ();


        
        

        // std::cout << "Took " << cnt << " steps" << std::endl;
        // img_left = simcam_left->getImage ();
        // std::cout << "Image: " << img_left->getWidth () << "x" << img_left->getHeight () << " bits "
        //           << img_left->getBitsPerPixel () << " channels " << img_left->getNrOfChannels ()
        //           << std::endl;
        
        

        // simcam_left->stop ();
        app.close ();       

    }
    
    RWS_END ()
    

    
    // cv::Mat img_left = cv::imread("../build/image_left.ppm");
    // cv::Mat img_right = cv::imread("../build/image_right.ppm");

    // cv::cvtColor(img_left, img_left, cv::COLOR_BGR2GRAY);
    // cv::cvtColor(img_right, img_right, cv::COLOR_BGR2GRAY);

    // cv::namedWindow("Car",cv::WINDOW_AUTOSIZE);

    // cv::imshow("Car",img_left);

    // cv::waitKey(0);

    // cv::destroyWindow("Car");
    // cv::namedWindow("Car",cv::WINDOW_AUTOSIZE);

    // cv::imshow("Car",img_right);
    // cv::waitKey(0);
    // cv::destroyWindow("Car");

    // cv::Mat colors = img_left;
    // cv::Mat disp;
    // while (true) {
    //     int nDisparities, BlockSize;

    //     std::cout << "Choose nDisparities: ";
    //     std::cin >> nDisparities;
    //     std::cout << "Choose SADWindowSize: ";
    //     std::cin >> BlockSize;

        
    //     disp = DisparitySGBM(img_left, img_right, nDisparities, BlockSize);

    //     auto dispNorm = normDisparity(disp);
    //     cv::imshow("Stereo", dispNorm);
    //     // Press esc to choose settings
    //     if ( (char) 27 == (char) cv::waitKey(0) ) {
    //         cv::imwrite("disparity.png", dispNorm);
    //         break;
    //     }
    // }




	// auto qMat = defineQ(img_left.cols, img_left.rows);
    // cv::Mat points = reproject3D(disp, qMat);
    // double z_threshold = 500;
    // savePointCloud("scene.pcd", points, colors, z_threshold);
	// std::cout << "Pointcloud saved" << std::endl;

    //poseEstimatePCL("bottle.ply", "scene.pcd");







    return 0;
}
