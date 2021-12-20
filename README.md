# How to build and run the executable
    To build the project navigate to the build folder and execute the following command:
        cmake ..
    To make the project stay in the same folder and run the command:
        make
    To run the executable execute the following command:
        ./../bin/main


# How to run the code for the different tasks
    To run the code for the different tasks, simply go into main.cpp and uncomment the line that handles that function. The names of the functions indicates which task they refer to.

# References
    - getQConfigs:
        This function appears multiple times throughout the code under different, but similar names (which is not well designed of course, but is a result of combining code from multiple branches to get all the code in one place). This code has been copied from the Robotics solution from lecture 3.
    - makePointCloudFromScene:
        This function is heavily inspired by the code example from robworks website. The original code can be found on this link: https://www.robwork.dk/manual/simulated_sensors/.
    - poseEstimatePCL:
        The part of this funtion that deals with global pose estimation is inspired by the code example from PCL's website. The original code can be found on this link: https://pcl.readthedocs.io/projects/tutorials/en/latest/alignment_prerejective.html?highlight=fpfh.
        The part of the function that deals with ICP is copied from the vision solution from lecture 6.


