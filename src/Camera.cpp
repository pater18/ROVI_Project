#include "Camera.h"

MyCamera::MyCamera (const std::string name, rw::models::WorkCell::Ptr wc)
{
    MyCamera::name = name;
    MyCamera::camera_frame = wc->findFrame(name);
    if (MyCamera::camera_frame == nullptr)
        RW_THROW ("Camera frame could not be found.");
    const PropertyMap& properties = MyCamera::camera_frame->getPropertyMap();
    if (!properties.has ("Camera"))
        RW_THROW ("Camera frame does not have Camera property.");
    
    //const std::string* ptr = MyCamera::camera_frame->getPropertyMap().getPtr<std::string>("Camera");
    // if (ptr) 
    // {
    //     std::cout << "Property 'Camera' has value " << *ptr << "\n";
    // }
    const std::string parameters = properties.get< std::string > ("Camera");
    std::istringstream iss (parameters, std::istringstream::in);

    iss >> MyCamera::fovy >> MyCamera::width >> MyCamera::height;
    // std::cout << "Camera properties: fov " << MyCamera::fovy << " width " 
    //                 << MyCamera::width << " height " << MyCamera::height << std::endl;

}

Frame* MyCamera::getcamera_frame()
{
    return MyCamera::camera_frame;
}

int MyCamera::getwidth()
{
    return MyCamera::width;
}
double MyCamera::getfovy()
{
    return MyCamera::fovy;
}

int MyCamera::getheight()
{
    return MyCamera::height;
}
std::string MyCamera::getname()
{
    return MyCamera::name;
}