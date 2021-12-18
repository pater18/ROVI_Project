void saveVelAcc(std::vector<rw::math::Q> data, std::string file_name)
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