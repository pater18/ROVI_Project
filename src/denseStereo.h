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
// #include <opencv2/opencv.hpp>
// #include <opencv2/features2d/features2d.hpp>

#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rwlibs::simulation;
using namespace rws;
using namespace rw::math;

std::vector<rw::math::Transform3D<>> makePointCloudFromScene(std::vector<rw::math::Transform3D<> > bottle_transformations)
{

    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../Scene.wc.xml");
    printDeviceNames(*wc);

    if (NULL == wc)
    {
        RW_THROW("COULD NOT LOAD scene... check path!");
    }

    // MyCamera camera_left("Camera_Left", wc);
    // MyCamera camera_right("Camera_Right", wc);

    Frame *const scanner25d = wc->findFrame("Scanner25D");
    if (scanner25d == nullptr)
        RW_THROW("Scanner frame could not be found.");
    const PropertyMap &properties = scanner25d->getPropertyMap();
    if (!properties.has("Scanner25D"))
        RW_THROW("Scanner25D frame does not have Scanner25D property.");
    const std::string parameters = properties.get<std::string>("Scanner25D");
    std::istringstream iss(parameters, std::istringstream::in);
    double fovy;
    int width;
    int height;
    iss >> fovy >> width >> height;
    std::cout << "Scanner25D properties: fov " << fovy << " width " << width << " height " << height
              << std::endl;

    rw::kinematics::MovableFrame::Ptr bottleFrame = wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    if (NULL == bottleFrame)
    {
        RW_THROW("COULD not find movable frame bottle ... check model");
    }

    // // Bottle poses
    // std::vector<Vector3D<>> vectors;
    // vectors.push_back(Vector3D<>(-0.30, 0.40, 0.21));
    // vectors.push_back(Vector3D<>(-0.30, 0.50, 0.21));
    // vectors.push_back(Vector3D<>(-0.10, 0.40, 0.21));
    // vectors.push_back(Vector3D<>(-0.10, 0.50, 0.21));
    // vectors.push_back(Vector3D<>(0.10, 0.40, 0.21));
    // vectors.push_back(Vector3D<>(0.10, 0.50, 0.21));
    // vectors.push_back(Vector3D<>(0.30, 0.40, 0.21));
    // vectors.push_back(Vector3D<>(0.30, 0.50, 0.21));

    // RPY<> R1 = RPY<>(-1.571, 0, 1.571);

    // std::vector<rw::math::Transform3D<>> bottle_transformations;
    // rw::math::Transform3D<> temp = Transform3D<>(vectors[0], R1.toRotation3D());

    // for (size_t i = 0; i < vectors.size(); i++)
    // {
    //     temp = Transform3D<>(vectors[i], R1.toRotation3D());
    //     bottle_transformations.push_back(temp);
    // }

    State state = wc->getDefaultState();

    std::cout << bottleFrame->getTransform(state) << std::endl;

    RobWorkStudioApp app("");
    RWS_START(app)
    {

        RobWorkStudio *const rwstudio = app.getRobWorkStudio();
        rwstudio->postOpenWorkCell("../Scene.wc.xml");
        TimerUtil::sleepMs(5000);
        State state_studio = rwstudio->getState();

        const SceneViewer::Ptr gldrawer = rwstudio->getView()->getSceneViewer();
        // const GLFrameGrabber::Ptr framegrabber_left = ownedPtr (new GLFrameGrabber (camera_left.getwidth(), camera_left.getheight(), camera_left.getfovy()));
        // const GLFrameGrabber::Ptr framegrabber_right = ownedPtr (new GLFrameGrabber (camera_right.getwidth(), camera_right.getheight(), camera_right.getfovy()));
        const GLFrameGrabber25D::Ptr framegrabber_depth = ownedPtr(new GLFrameGrabber25D(width, height, fovy));
        // framegrabber_left->init (gldrawer);
        // framegrabber_right->init (gldrawer);
        framegrabber_depth->init(gldrawer);

        SimulatedScanner25D::Ptr simscanner25d = ownedPtr(new SimulatedScanner25D("SimulatedScanner25D", scanner25d, framegrabber_depth));
        simscanner25d->setFrameRate(100);
        simscanner25d->open();
        simscanner25d->acquire();
        for (size_t i = 0; i < bottle_transformations.size(); i++)
        {
            static const double DT = 0.001;
            const Simulator::UpdateInfo info(DT);
            int cnt = 0;

            bottleFrame->setTransform(bottle_transformations[i], state_studio);
            std::cout << bottleFrame->getTransform(state_studio) << std::endl;

            rwstudio->setState(state_studio);
            simscanner25d->acquire();


            while (!simscanner25d->isScanReady())
            {
                //std::cout << "Pointcloud is not ready yet. Iteration " << cnt << std::endl;
                simscanner25d->update(info, state_studio);
                cnt++;
            }

            const rw::geometry::PointCloud pointcloud = simscanner25d->getScan();
            pointcloud.savePCD(pointcloud, "scene_clouds/cloud_scene" + std::to_string(i) + ".pcd");
        }


        app.close();
    }

    RWS_END()

 

    return bottle_transformations;
}
