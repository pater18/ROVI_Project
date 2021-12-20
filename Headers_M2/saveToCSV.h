#pragma once

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


void saveDataToCSV(std::vector<double> data, std::string file_name)
{

    std::ofstream myFile(file_name);

    // Send the column name to the stream

    // Send data to the stream
    for (size_t i = 0; i < data.size(); ++i)
    {
        myFile << data.at(i) << "\n";
    }

    // Close the file
    myFile.close();
}