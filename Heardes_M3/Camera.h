#include <rw/core.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>

#include <string>
#include <iostream>

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rwlibs::simulation;
//using namespace rws;

class MyCamera
{
public: 
    MyCamera();
    MyCamera(const std::string name, rw::models::WorkCell::Ptr wc);

    int getwidth();
    double getfovy();
    int getheight();
    std::string getname();
    Frame* getcamera_frame();

private:

    double fovy;
    int width;
    int height;
    std::string name;
    Frame* camera_frame;



};