
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/core/Log.hpp>
#include <rw/core.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/kinematics.hpp>
#include <rw/proximity.hpp>
#include <rw/trajectory.hpp>
#include <rw/loaders/image/ImageLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/sensor/Camera.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <vector>
#include <iostream>
#include <string>
#include <array>
#include "Headers_M3/Camera.h"


USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rwlibs::simulation;
using namespace rws;





struct boundingBox
{
    cv::Point topLeft;
    cv::Point downRight;
};

void getCameraParam(rw::models::WorkCell::Ptr wc, std::string frameName, cv::Mat &_KA, cv::Mat &_H)
{
    rw::kinematics::State state = wc->getDefaultState();
    rw::kinematics::Frame* cameraFrame = wc->findFrame(frameName);
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap().has("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;

                double fovy_pixel = height / 2 / tan(fovy * (2*M_PI) / 360.0 / 2.0 );

                Eigen::Matrix<double, 3, 4> KA;
                KA << fovy_pixel, 0, width / 2.0, 0,
                    0, fovy_pixel, height / 2.0, 0,
                    0, 0, 1, 0;

                // std::cout << "Intrinsic parameters:" << std::endl;
                // std::cout << KA << std::endl;

                Transform3D<> camPosOGL = cameraFrame->wTf(state);
                Transform3D<> openGLToVis = Transform3D<>(RPY<>(-Pi, 0, Pi).toRotation3D());
                Transform3D<> H = inverse(camPosOGL * inverse(openGLToVis));
                
                // std::cout << "Extrinsic parameters:" << std::endl;
                // std::cout << H.e() << std::endl;
                
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++){
                        if (i != 3)
                            _KA.at<double>(i, j) = KA(i,j);
                        _H.at<double>(i,j) = H.e()(i,j);
                    }
                }

            }
        }
}

void erodeImage (cv::Mat &image, int type = 0, int erosion_size = 3){
    int erosion_type = 0;

    if( type == 0 ){ erosion_type = cv::MORPH_RECT; }
    else if( type == 1 ){ erosion_type = cv::MORPH_CROSS; }
    else if( type == 2) { erosion_type = cv::MORPH_ELLIPSE; }

    cv::Mat element = cv::getStructuringElement( erosion_type,
                        cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                        cv::Point( erosion_size, erosion_size ) );
    
    cv::erode( image, image, element );
}

void dilateImage(cv::Mat &image, int type = 0, int dilation_size = 3 ) {
    int dilation_type = 0;

    if( type == 0 ){ dilation_type = cv::MORPH_RECT; }
    else if( type == 1 ){ dilation_type = cv::MORPH_CROSS; }
    else if( type == 2) { dilation_type = cv::MORPH_ELLIPSE; }

    cv::Mat element = cv::getStructuringElement( dilation_type,
                        cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                        cv::Point( dilation_size, dilation_size ) );
    cv::dilate( image, image, element );
}

void addNoise(cv::Mat &image, double mean = 0.0, double stdDev = 5.0) {
    mean = 0.0;
    stdDev = 5.0;

    cv::Mat noise = cv::Mat(image.size(), image.type());
    cv::randn(noise, cv::Scalar::all(mean), cv::Scalar::all(stdDev));
    image = image + noise;
    cv::imshow("Image with noise", image);

}

void readImage(cv::Mat &leftImage, cv::Mat &rightImage)
{
    // Read the images from robworkstudio
    leftImage = cv::imread("../build/image_left.ppm");
    rightImage = cv::imread("../build/image_right.ppm");
    if (leftImage.empty() || rightImage.empty()) {
        std::cout << "Could not read images in function readImage " << std::endl;
        return;
    }
    
    cv::flip(rightImage, rightImage, 1);
    cv::flip(leftImage, leftImage, 1);

    //cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2GRAY);
    //cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2GRAY);
 

    // erodeImage(leftImage);
    // cv::imshow("left image", leftImage);
    // dilateImage(leftImage);
    // cv::imshow("Dilated left image", leftImage);

    // erodeImage(rightImage);
    // cv::imshow("eroded right image", rightImage );
    // dilateImage(rightImage);
    // cv::imshow("Dilated right image", rightImage);

    // addNoise(leftImage);
    // addNoise(rightImage);

    // cv::waitKey(0);
    // cv::destroyAllWindows();
}

cv::Mat constructProjectionMat(cv::Mat KA, cv::Mat H) {
    return KA * H;
}

std::vector<cv::Point3f> find3dpoint(std::vector<cv::DMatch> goodMatches, std::vector<cv::KeyPoint> keypoints_left, 
                std::vector<cv::KeyPoint> keypoints_right,
                cv::Mat proj_right, cv::Mat proj_left){

    //############################################################
    // This function only finds one 3D point at the moment
    //############################################################
    cv::Mat worldCoordinate (1, 1, CV_64FC4);
    cv::Mat pointLeftImage (1, 1, CV_64FC2);    
    cv::Mat pointRightImage (1, 1, CV_64FC2);
    std::vector<cv::Point3f> image_coordinates;

    for (auto match : goodMatches ){
        cv::Point3d tmp;
        pointLeftImage.at<cv::Vec2d>(0)[0] = keypoints_left[match.queryIdx].pt.x;
        pointLeftImage.at<cv::Vec2d>(0)[1] = keypoints_left[match.queryIdx].pt.y;
        pointRightImage.at<cv::Vec2d> (0)[0] = keypoints_right[match.trainIdx].pt.x;
        pointRightImage.at<cv::Vec2d> (0)[1] = keypoints_right[match.trainIdx].pt.y;
        cv::triangulatePoints(proj_left, proj_right, pointLeftImage, pointRightImage, worldCoordinate);

        worldCoordinate = worldCoordinate /  worldCoordinate.at<double>(3, 0);
        tmp.x = worldCoordinate.at<double>(0, 0) ;
        tmp.y = worldCoordinate.at<double>(1, 0) ;
        tmp.z = worldCoordinate.at<double>(2, 0) ;

        image_coordinates.push_back(tmp);    
    }

    //std::cout << "Image points: " << pointLeftImage << "\t" << pointRightImage << std::endl << std::endl;
    //std::cout << "Triangulated point : " << std::endl << worldCoordinate  << std::endl << std::endl;
    //std::cout << "Triangulated point (normalized): " << std::endl << image_coordinates[0] << std::endl << std::endl;

    return image_coordinates;
}


boundingBox templateMatch(const cv::Mat image){

    cv::Mat img_display, result;
    boundingBox resultBox; 

    
    cv::Mat templateImage = cv::imread("../Templates/bottle.png");
    if (templateImage.empty()){
        std::cout << "Could not read template image in function templateMatch" << std::endl << std::endl; 
    }


    image.copyTo(img_display);
    
    int result_cols = image.cols - templateImage.cols + 1;
    int result_rows = image.rows - templateImage.rows + 1;

    result.create( result_rows, result_cols, CV_32FC1);
    
    cv::matchTemplate(image, templateImage, result, cv::TM_CCOEFF_NORMED);

    cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    double minVal; 
    double maxVal; 
    cv::Point minLoc; 
    cv::Point maxLoc;
    cv::Point matchLoc;

    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
    matchLoc = maxLoc;

    resultBox.topLeft = matchLoc;
    resultBox.downRight.x = matchLoc.x + templateImage.cols;
    resultBox.downRight.y = matchLoc.y + templateImage.rows;
    

    //The following code is to show where the template is matced to
    cv::rectangle(img_display, matchLoc, cv::Point(matchLoc.x + templateImage.cols, matchLoc.y + templateImage.rows), cv::Scalar::all(0), 2, 8, 0);
    cv::rectangle( result, matchLoc, cv::Point( matchLoc.x + templateImage.cols , matchLoc.y + templateImage.rows ), cv::Scalar::all(0), 2, 8, 0 );
    cv::imshow( "Template Mask", img_display );
    // // cv::imshow( "result_window", result );
    // cv::waitKey(0);
    // cv::destroyAllWindows();
    
    return resultBox;
}

void getKeyPressAction(int key, int &nfeatures , int &nOctaveLayers ,
                    double &contrastThreshold ,  double &edgeThreshold, double &sigma)
{


    //std::cout << "Key that is pressed is char: " << char(key) << " and integer: " << key << std::endl << std::endl;  

    switch (key)
    {
    case 110: //n
        nfeatures += 1;
        break;
    case 109: //m
        nfeatures -= 1;
        if(nfeatures < 0)
            nfeatures = 0;
        break;
    case 111: // o
        nOctaveLayers += 1;
        break;
    case 112: // p
        nOctaveLayers -= 1;
        if(nOctaveLayers < 0)
            nOctaveLayers = 0;
        break;
    case 99: // c
        contrastThreshold += 0.02;
        break;
    case 118: // v
        contrastThreshold -= 0.02;
        if(contrastThreshold < 0.0)
            contrastThreshold = 0.0;
        break;
    case 101: // e
        edgeThreshold += 5;
        break;
    case 114: //r
        edgeThreshold -= 5;
        if(edgeThreshold < 0)
            edgeThreshold = 0;
        break;
    case 115: //s
        sigma += 0.1;
        break;
    case 100: //d
        sigma -= 0.1;
        if(sigma < 0.0)
            sigma = 0.0;
        break;
    default:
        break;
    }

    // std::cout << "nfeatures: " << nfeatures << "\nnOctaveLayers: " << nOctaveLayers << "\ncontrastThreshold: " << contrastThreshold
    //         << "\nedgeThreshold: " << edgeThreshold << "\nsigma: " << sigma << std::endl<< std::endl;
 


}

void findPose(  rw::models::WorkCell::Ptr wc, std::vector<cv::Point3f> objectCoordinates, 
                std::vector<cv::KeyPoint> keypoints_left, std::vector<cv::DMatch> goodMatches, 
                cv::Mat leftImage, cv::Mat proj_left){
    
    std::cout << "Points in the real world:" << std::endl;
    for (cv::Point3f coord : objectCoordinates){
        std::cout << coord << std::endl; 
    }
    std::cout << std::endl;

    // std::cout << "Image points:\n";
    std::vector<cv::Point2f> image_points;
    for (auto match : goodMatches){
        cv::Point2f tmp; 
        tmp.x = (int)keypoints_left[match.queryIdx].pt.x;
        tmp.y = (int)keypoints_left[match.queryIdx].pt.y;
        
        image_points.push_back(tmp);
        // std::cout << "[ " << tmp << " ]\n" ; 
    }
    // std::cout << std::endl;

    // cv::Mat newImage;
    // leftImage.copyTo(newImage);
    // for (auto key : image_points)
    //         cv::circle(newImage, key, 2, cv::Scalar(0,0,255));

    // cv::imshow("Image points", newImage);
    
    cv::Mat cameraMatrix(3,3, CV_64F);
    cameraMatrix.at<double> (0,0) = 514.682;
    cameraMatrix.at<double> (1,0) = 0;
    cameraMatrix.at<double> (2,0) = 0;
    cameraMatrix.at<double> (0,1) = 0;
    cameraMatrix.at<double> (1,1) = 514.682;
    cameraMatrix.at<double> (2,1) = 0;
    cameraMatrix.at<double> (0,2) = 320;
    cameraMatrix.at<double> (1,2) = 240;
    cameraMatrix.at<double> (2,2) = 1;

    
    rw::kinematics::State state = wc->getDefaultState();        
    rw::kinematics::Frame* cameraFrame = wc->findFrame("Camera_Left");
    rw::math::Transform3D<> worldTCamera = rw::kinematics::Kinematics::worldTframe(cameraFrame, state);

    cv::Mat frameCameraObject ;
    cv::Mat worldTCameraCV(4,4, CV_64F);

    for (int i = 0; i < 4; i++) 
    {
        worldTCameraCV.at<double>(0,i) = worldTCamera(0,i);
        worldTCameraCV.at<double>(1,i) = worldTCamera(1,i);
        worldTCameraCV.at<double>(2,i) = worldTCamera(2,i);
        worldTCameraCV.at<double>(3,i) = worldTCamera.e()(3,i);
    }


    // cv::Mat rvec(3,1, CV_64F);
    // cv::Mat tvec(3,1, CV_64F);
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat rotationMat (3,3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(4,1, CV_64F);

 
    cv::solvePnP(objectCoordinates, image_points, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    std::cout << "Rotation vector:\n" << rvec << std::endl;
    cv::Rodrigues(rvec, rotationMat);
    frameCameraObject = rotationMat;

    std::cout << "Translation vector:\n" << tvec << std::endl << std::endl;
    cv::hconcat(frameCameraObject, tvec, frameCameraObject);
    cv::Mat lastRow = cv::Mat::zeros(1,4, CV_64F);
    lastRow.at<double> (0,3) = 1.0; 
    cv::vconcat(frameCameraObject, lastRow, frameCameraObject);

    std::cout << "Camera to object transform\n" << frameCameraObject << "\n" << std::endl;
    std::cout << "World to camera transform\n" << worldTCameraCV << "\n" << std::endl;

    cv::Mat objectPose = frameCameraObject * worldTCameraCV;
    std::cout << "Object pose: \n" << objectPose << std::endl;
    std::cout << "Object pose times proj\n" << proj_left * worldTCameraCV << std::endl;
    //cv::invert(objectPose, objectPose);
    cv::Mat result = objectPose.inv();
    
    std::cout << "Result pose: \n" << result << std::endl;


}

int getPoseWSparceStereo(){

    //Get the workcell pointer
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../Scene.wc.xml");
	//printDeviceNames(*wc);
	if(NULL==wc)
    {
		RW_THROW("COULD NOT LOAD scene... check path!");
		return -1;
	}

    
    // Get the intrinsic and extrinsic parameters from the cameras
    cv::Mat KA_right = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat H_right = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat KA_left = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat H_left = cv::Mat::zeros(4, 4, CV_64F);
    // std::cout << "Camera_Right" << std::endl;
    getCameraParam(wc, "Camera_Right", KA_right, H_right);
    // std::cout << "Camera_Left" << std::endl;
    getCameraParam(wc, "Camera_Left", KA_left, H_left);
    
    // Create projection matrix
    cv::Mat proj_right = constructProjectionMat( KA_right , H_right);
    cv::Mat proj_left = constructProjectionMat (KA_left , H_left);
    // std::cout << "Camera 2 (right) projection matrix" << std::endl << proj_right << std::endl << std::endl;
    // std::cout << "Camera 1 (left) projection matrix" << std::endl << proj_left << std::endl << std::endl;


    // Get images from robworkstudio
    MyCamera camera_left("Camera_Left", wc);
    MyCamera camera_right("Camera_Right", wc);

    RobWorkStudioApp app ("");
    RWS_START (app)
    {
        RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
        rwstudio->postOpenWorkCell ("../Scene.wc.xml");
        TimerUtil::sleepMs (5000);
        const SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
        const GLFrameGrabber::Ptr framegrabber_left = ownedPtr (new GLFrameGrabber (camera_left.getwidth(), camera_left.getheight(), camera_left.getfovy()));
        const GLFrameGrabber::Ptr framegrabber_right = ownedPtr (new GLFrameGrabber (camera_right.getwidth(), camera_right.getheight(), camera_right.getfovy()));
        framegrabber_left->init (gldrawer);
        framegrabber_right->init (gldrawer);
        SimulatedCamera::Ptr simcam_left = ownedPtr (new SimulatedCamera ("SimulatedCamera", camera_left.getfovy(), camera_left.getcamera_frame(), framegrabber_left));
        simcam_left->setFrameRate (100);
        simcam_left->initialize ();
        simcam_left->start ();
        simcam_left->acquire ();
        SimulatedCamera::Ptr simcam_right = ownedPtr (new SimulatedCamera ("SimulatedCamera", camera_right.getfovy(), camera_right.getcamera_frame(), framegrabber_right));
        simcam_right->setFrameRate (100);
        simcam_right->initialize ();
        simcam_right->start ();
        simcam_right->acquire ();
        
        
        static const double DT = 0.001;
        const Simulator::UpdateInfo info (DT);
        State state = wc->getDefaultState ();
        int cnt     = 0;
        const Image* img_left;
        const Image* img_right;

        while (!simcam_left->isImageReady ()) {
            //std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
            simcam_left->update (info, state);
            cnt++;
        }
            while (!simcam_right->isImageReady ()) {
            //std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
            simcam_right->update (info, state);
            cnt++;
        }
        img_left = simcam_left->getImage ();
        img_left->saveAsPPM ("image_left.ppm");
        simcam_left->acquire ();
        img_right = simcam_right->getImage ();
        img_right->saveAsPPM ("image_right.ppm");
        simcam_right->acquire ();
        

        //std::cout << "Took " << cnt << " steps" << std::endl;
        img_left = simcam_left->getImage ();
        // std::cout << "Image: " << img_left->getWidth () << "x" << img_left->getHeight () << " bits "
        //           << img_left->getBitsPerPixel () << " channels " << img_left->getNrOfChannels ()
        //           << std::endl;
        

        simcam_left->stop ();
        app.close ();        
    }
    RWS_END ()

    // Variable for keyboard press
    int key = 0;

    // Variables for the SIFT feature detector
    int nfeatures = 0, nOctaveLayers = 11;
    double contrastThreshold = 0.04, edgeThreshold = 0.0, sigma = 1.1;

    while(key != 113){

        getKeyPressAction(key, nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
        
        cv::Mat img_left;
        cv::Mat img_right;
        cv::Mat dstL;
        cv::Mat dstR;
        
        readImage(img_left, img_right);

        // cv::Canny(dstL,img_left, 10.0, 100.0);
        // cv::Canny(dstR,img_right, 10.0, 100.0);
        // cv::imshow("cannyR" ,img_left);
        // cv::imshow("cannyL" ,img_right);
        // cv::waitKey(0);
        // cv::destroyAllWindows();
  

        boundingBox boxLeftImage = templateMatch(img_left);
        boundingBox boxRightImage = templateMatch(img_right);

        
        // Find features on left and right image within a mask
        //cv::Ptr<cv::Feature2D> sift = cv::SIFT::create();
        cv::Ptr<cv::Feature2D> sift = cv::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
        std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
        cv::DescriptorExtractor extractor = cv::DescriptorExtractor();
        cv::Mat descriptor_left, descriptor_right;
        cv::Mat maskLeftImage = cv::Mat(img_left.rows, img_left.cols, CV_8UC1, cv::Scalar(0));
        cv::Mat maskRightImage = cv::Mat(img_right.rows, img_right.cols, CV_8UC1, cv::Scalar(0));

        // Create an image mask based on the template maching box found earlier.  
        for (int x = boxLeftImage.topLeft.x; x < boxLeftImage.downRight.x; x++){
            for (int y = boxLeftImage.topLeft.y; y < boxLeftImage.downRight.y; y++){
                maskLeftImage.at<uchar>(y,x) = 255;
            }
        }
        for (int x = boxRightImage.topLeft.x; x < boxRightImage.downRight.x; x++){
            for (int y = boxRightImage.topLeft.y; y < boxRightImage.downRight.y; y++){
                maskRightImage.at<uchar>(y,x) = 255;
            }
        }
        // Show the area that the SIFT detector should focus on
        // cv::imshow("Mask left", maskLeftImage);
        // cv::imshow("Mask right image", maskRightImage);

        sift->detectAndCompute(img_left, maskLeftImage, keypoints_left, descriptor_left);
        sift->detectAndCompute(img_right, maskRightImage,keypoints_right, descriptor_right);
        
        if (descriptor_left.empty())
            std::cout << "Descripter empty\n" << std::endl;
        // else
        //     std::cout << "Descripter size: " << descriptor_left.size << std::endl;
        

        // Draw the found features on both images
        for (cv::KeyPoint key : keypoints_left)
            cv::circle(img_left, key.pt, 2, cv::Scalar(255,255,0));
        for (cv::KeyPoint key : keypoints_right)
            cv::circle(img_right, key.pt, 2, cv::Scalar(255,255,0));
        
        // cv::imshow("First detected points left image" , img_left);
        // cv::imshow("First detected points right image" , img_right);

        // Match the features found on both images
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
        std::vector< std::vector<cv::DMatch> > matches;
        matcher->knnMatch(descriptor_left, descriptor_right, matches, 2);

        // Only use maches where the points are close to each other. 
        const float threshold = 0.7f;
        std::vector<cv::DMatch> goodMatches;         
        for (size_t i = 0; i < matches.size(); i ++ ){
            if (matches[i][0].distance < threshold * matches[i][1].distance){
                goodMatches.push_back(matches[i][0]);    
            }
        }

        // Delete matches with different y values. 
        for (int i = goodMatches.size() - 1; i >= 0; i--){
            cv::DMatch tmp = goodMatches[i];
            int y_val_left_img = (int)keypoints_left[tmp.queryIdx].pt.y;
            int y_val_right_img = (int)keypoints_right[tmp.trainIdx].pt.y;
            if (y_val_left_img != y_val_right_img) 
            {
                //std::cout << "element to erase: " << keypoints_left[tmp.queryIdx].pt << keypoints_right[tmp.trainIdx].pt <<  std::endl;
                goodMatches.erase(goodMatches.begin() + i); 
            }
        }

        // // Print out the points that is matched together
        // for (auto val : goodMatches){
        //     std::cout << keypoints_left[val.queryIdx].pt << std::endl;
        //     std::cout << keypoints_right[val.trainIdx].pt << std::endl << std::endl;
        // }

        //-- Draw matches
        cv::Mat img_matches;
        std::vector<cv::Point3f> objectCoordinates;
        
        if (!goodMatches.empty()){
            drawMatches( img_left, keypoints_left, img_right, keypoints_right, 
                        goodMatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            objectCoordinates = find3dpoint(goodMatches, keypoints_left, keypoints_right, proj_right, proj_left);
            findPose(wc, objectCoordinates, keypoints_left, goodMatches, img_left, proj_left); 

        } else {
            std::cout << "No good macthes was found\n" << std::endl;
        }
        

        
        

        cv::namedWindow("Left image sift",cv::WINDOW_AUTOSIZE);
        cv::imshow("Left image sift",img_left);

        cv::namedWindow("Right image sift",cv::WINDOW_AUTOSIZE);
        cv::imshow("Right image sift",img_right);


        if (!img_matches.empty()){
            cv::namedWindow("Image matches",cv::WINDOW_AUTOSIZE);
            cv::imshow("Image matches",img_matches);
        }

        key = cv::waitKey(0);
    }
    cv::destroyAllWindows();




    return 0; 

}