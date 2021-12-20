#include "Headers_M2/poseEstimatePipeline.h"
#include "Headers_MotionPlanning/interpolate_parabolic.h"
#include "Headers_MotionPlanning/interpolate_linear.h"
#include "Headers_MotionPlanning/RRT_planning.h"
#include "Headers_Reachability/reachability.h"
#include "Headers_M3/sparceStereo.h"




int main(int argc, char** argv)
{


    //poseEstimatePipelineForMain();
    interpolateLinear();
    //interpolatePara();
    //rrtPlanning();'
    //reach_object();
    //getPoseWSparceStereo();

	return 0;
}



