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

int calcErrorOnPose(rw::math::Transform3D<> pose, Matrix4f pose_esti)
{
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../Scene.wc.xml");
    if(NULL==wc)
    {
        RW_THROW("COULD NOT LOAD scene... check path!");
        return -1;
	}

    rw::kinematics::MovableFrame::Ptr bottleFrame = wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
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
    
    rw::kinematics::Frame* worldFrame = wc->findFrame("WORLD");
	if(NULL==worldFrame)
    {
		RW_THROW("COULD not find movable frame WORLD ... check model");
		return -1;
	}	

    rw::kinematics::Frame* scannerFrame = wc->findFrame("Scanner25D");
	if(NULL==scannerFrame)
    {
		RW_THROW("COULD not find movable frame table ... check model");
		return -1;
	}	

    



    State state = wc->getDefaultState();

    bottleFrame->moveTo(pose, state);

    rw::math::Transform3D<> frameWorldTTable = rw::kinematics::Kinematics::frameTframe(worldFrame, tableFrame, state);
    rw::math::Transform3D<> frameTableTScanner = rw::kinematics::Kinematics::frameTframe(tableFrame, scannerFrame, state);
    rw::math::Transform3D<> frameScannerTBottle = rw::kinematics::Kinematics::frameTframe(scannerFrame, bottleFrame, state);
    rw::math::Transform3D<> frameScannerTTable = rw::kinematics::Kinematics::frameTframe(scannerFrame, tableFrame, state);
    rw::math::Transform3D<> frameTableTWorld = rw::kinematics::Kinematics::frameTframe(tableFrame, worldFrame, state);

    // Matrix4f frameScannerTTable_e = frameScannerTTable.e();
    // Matrix4f frameTableTWorld_e = frameTableTWorld.e();

    //std::cout << frameWorldTTable << std::endl;
    //std::cout << frameTableTScanner << std::endl;
    std::cout << frameScannerTBottle << std::endl;
    
    //std::cout << pose_esti.inverse() * frameScannerTTable_e * frameTableTWorld_e  << std::endl;

    //std::cout << frameWorldTTable * frameTableTScanner * frameScannerTBottle << std::endl;

    






    return 0;


}