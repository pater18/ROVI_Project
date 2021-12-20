#pragma once

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

    if (NULL == wc)
    {
        RW_THROW("COULD NOT LOAD scene... check path!");
    }

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
        const GLFrameGrabber25D::Ptr framegrabber_depth = ownedPtr(new GLFrameGrabber25D(width, height, fovy));

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
