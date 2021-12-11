#include <rw/invkin.hpp>
#include <rw/core/Log.hpp>
#include <rw/core.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics.hpp>
#include <rw/loaders/image/ImageLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>

using namespace Eigen;

USE_ROBWORK_NAMESPACE
using namespace robwork;

int calcErrorOnPose()
{
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../Scene.wc.xml");
    if(NULL==wc)
    {
        RW_THROW("COULD NOT LOAD scene... check path!");
        return -1;
	}

    rw::kinematics::Frame* bottleFrame = wc->findFrame("Bottle");
	if(NULL==bottleFrame)
    {
		RW_THROW("COULD not find movable frame bottle ... check model");
		return -1;
	}	

    rw::kinematics::Frame* tableFrame = wc->findFrame("Table");
	if(NULL==tableFrame)
    {
		RW_THROW("COULD not find movable frame table ... check model");
		return -1;
	}	

    rw::kinematics::Frame* scannerFrame = wc->findFrame("Scanner25D");
	if(NULL==scannerFrame)
    {
		RW_THROW("COULD not find movable frame table ... check model");
		return -1;
	}	

    



    State state = wc->getDefaultState();


    rw::math::Transform3D<> frameWorldTScanner = rw::kinematics::Kinematics::frameTframe(tableFrame, scannerFrame, state);
    rw::math::Transform3D<> frameScannerTBottle = rw::kinematics::Kinematics::frameTframe(scannerFrame, bottleFrame, state);
    std::cout << frameWorldTScanner * frameScannerTBottle  << std::endl;
    

    std::cout << frameScannerTBottle << std::endl;
    std::cout << frameWorldTScanner << std::endl;
    
    rw::kinematics::Frame* parent = bottleFrame->getParent(state);

    std::cout << parent->getName() << std::endl;
    frameScannerTBottle = rw::kinematics::Kinematics::frameTframe(parent, bottleFrame, state);
    std::cout << frameScannerTBottle << std::endl;



    return 0;


}