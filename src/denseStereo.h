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


#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rwlibs::simulation;
using namespace rws;



int getPoseWithDenseStereo()
{
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../Scene.wc.xml");
	printDeviceNames(*wc);

	if(NULL==wc)
    {
		RW_THROW("COULD NOT LOAD scene... check path!");
		return -1;
	}

    Frame* const camera_left = wc->findFrame ("Camera_Right");
    if (camera_left == nullptr)
        RW_THROW ("Camera frame could not be found.");
    const PropertyMap& properties = camera_left->getPropertyMap ();
    if (!properties.has ("Camera"))
        RW_THROW ("Camera frame does not have Camera property.");

    const std::string parameters = properties.get< std::string > ("Camera");
    std::istringstream iss (parameters, std::istringstream::in);
    double fovy;
    int width;
    int height;
    iss >> fovy >> width >> height;
    std::cout << "Camera properties: fov " << fovy << " width " << width << " height " << height
              << std::endl;


    RobWorkStudioApp app ("");
    RWS_START (app)
    {
        RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
        rwstudio->postOpenWorkCell ("../Scene.wc.xml");
        TimerUtil::sleepMs (5000);

        const SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();

        const GLFrameGrabber::Ptr framegrabber = ownedPtr (new GLFrameGrabber (width, height, fovy));
        framegrabber->init (gldrawer);
        SimulatedCamera::Ptr simcam =
            ownedPtr (new SimulatedCamera ("SimulatedCamera", fovy, camera_left, framegrabber));
        simcam->setFrameRate (100);
        simcam->initialize ();
        simcam->start ();
        simcam->acquire ();
        
        static const double DT = 0.001;
        const Simulator::UpdateInfo info (DT);
        State state = wc->getDefaultState ();
        int cnt     = 0;
        const Image* img;
        while (!simcam->isImageReady ()) {
            std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
            simcam->update (info, state);
            cnt++;
        }
        img = simcam->getImage ();
        img->saveAsPPM ("Image1.ppm");
        simcam->acquire ();
        while (!simcam->isImageReady ()) {
            std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
            simcam->update (info, state);
            cnt++;
        }
        std::cout << "Took " << cnt << " steps" << std::endl;
        img = simcam->getImage ();
        std::cout << "Image: " << img->getWidth () << "x" << img->getHeight () << " bits "
                  << img->getBitsPerPixel () << " channels " << img->getNrOfChannels ()
                  << std::endl;
        img->saveAsPPM ("Image2.ppm");

        simcam->stop ();
        app.close ();
    }
    RWS_END ()
 



    return 0;
}
