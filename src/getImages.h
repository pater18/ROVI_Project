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
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>
#include "Camera.h"
#include "PCL.h"
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
 

int saveCameraImages(rw::models::WorkCell::Ptr wc )
{

    RobWorkStudioApp app ("");
        RWS_START (app)
        {
            RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
            rwstudio->postOpenWorkCell ("../Scene.wc.xml");
            TimerUtil::sleepMs (5000);
            const SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
            const GLFrameGrabber::Ptr framegrabber_left = ownedPtr (new GLFrameGrabber (camera_left.getwidth(), camera_left.getheight(), camera_left.getfovy()));
            const GLFrameGrabber::Ptr framegrabber_right = ownedPtr (new GLFrameGrabber (camera_right.getwidth(), camera_right.getheight(), camera_right.getfovy()));
            framegrabber_left->init (gldrawer);
            framegrabber_right->init (gldrawer);
            SimulatedCamera::Ptr simcam_left = ownedPtr (new SimulatedCamera ("SimulatedCamera", camera_left.getfovy(), camera_left.getcamera_frame(), framegrabber_left));
            simcam_left->setFrameRate (100);
            simcam_left->initialize ();
            simcam_left->start ();
            simcam_left->acquire ();
            SimulatedCamera::Ptr simcam_right = ownedPtr (new SimulatedCamera ("SimulatedCamera", camera_right.getfovy(), camera_right.getcamera_frame(), framegrabber_right));
            simcam_right->setFrameRate (100);
            simcam_right->initialize ();
            simcam_right->start ();
            simcam_right->acquire ();
            
            static const double DT = 0.001;
            const Simulator::UpdateInfo info (DT);
            State state = wc->getDefaultState ();
            int cnt     = 0;
            const Image* img_left;
            const Image* img_right;

            while (!simcam_left->isImageReady ()) {
                std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
                simcam_left->update (info, state);
                cnt++;
            }
            while (!simcam_right->isImageReady ()) {
                std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
                simcam_right->update (info, state);
                cnt++;
            }
            img_left = simcam_left->getImage ();
            img_left->saveAsPPM ("image_left.ppm");
            simcam_left->acquire ();
            img_right = simcam_right->getImage ();
            img_right->saveAsPPM ("image_right.ppm");
            simcam_right->acquire ();
            

            std::cout << "Took " << cnt << " steps" << std::endl;
            img_left = simcam_left->getImage ();
            std::cout << "Image: " << img_left->getWidth () << "x" << img_left->getHeight () << " bits "
                    << img_left->getBitsPerPixel () << " channels " << img_left->getNrOfChannels ()
                    << std::endl;
            

            simcam_left->stop ();
            app.close ();        
        }
        RWS_END ()
}