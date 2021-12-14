

#include <fstream>
#include <vector>
#include <string>

void saveDataToCSV(std::vector<double> data, std::string file_name)
{

    std::ofstream myFile(file_name);
    
    // Send the column name to the stream
    
    // Send data to the stream
    for(size_t i = 0; i < data.size(); ++i)
    {
        myFile << data.at(i) << "\n";
    }
    
    // Close the file
    myFile.close();
}

