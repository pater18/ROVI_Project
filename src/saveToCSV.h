

#include <fstream>
#include <vector>
#include <string>
#include <rw/math/Q.hpp>
#include <rw/core.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/kinematics.hpp>



void saveVelAcc(std::vector<std::vector<double > > data, std::string file_name)
{
    std::ofstream myFile(file_name);


    for (size_t i = 0; i < data.size(); ++i)
    {
        for (size_t j = 0; j < data[i].size(); j++)
        {
            myFile << data[i][j] << "\t";
            myFile << ",";
        }
        myFile << std::endl;
    }
}


void saveVelAccRRT(std::vector<rw::math::Q> data, std::string file_name)
{
    std::ofstream myFile(file_name);
    std::vector<double> temp;
    for (size_t i = 0; i < data.size(); ++i)
    {
        data[i].toStdVector(temp);

        for (size_t j = 0; j < temp.size(); j++)
        {
            myFile << temp[j] << "\t";
            myFile << ",";
        }
        myFile << std::endl;
    }
}
