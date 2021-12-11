#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>



cv::Mat reproject3D(cv::Mat disp, cv::Mat Q)
{
    cv::Mat points;
    cv::reprojectImageTo3D(disp, points, Q, true);
    return points;
}
cv::Mat normDisparity(cv::Mat disp)
{
    cv::Mat dispNorm;
    cv::normalize(disp, dispNorm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    return dispNorm;
}
cv::Mat DisparitySGBM(cv::Mat &imgL, cv::Mat &imgR, int nDisparities, int BlockSize)
{
    auto sgbm = cv::StereoSGBM::create(0, nDisparities, BlockSize);
    cv::Mat disp;
    sgbm->setMode(cv::StereoSGBM::MODE_HH);
    sgbm->compute(imgL, imgR, disp);
    return disp;
}

cv::Mat DisparityBM(cv::Mat &imgL, cv::Mat &imgR, int nDisparities, int BlockSize)
{
    auto bm = cv::StereoBM::create(nDisparities, BlockSize);
    cv::Mat disp;
    bm->compute(imgL, imgR, disp);
    return disp;
}

cv::Mat defineQ(int img_width, int img_height)
{
    double cx = -img_width / 2, cy = -img_height / 2;
    double f = 50,	Tx = 100;
	cv::Mat Q = cv::Mat::zeros(4, 4, CV_64F);
	Q.at<double>(0, 0) = 1;
	Q.at<double>(1, 1) = 1;
	Q.at<double>(0, 3) = cx;
	Q.at<double>(1, 3) = cy;
	Q.at<double>(2, 3) = f;
	Q.at<double>(3, 2) = -1 / Tx;
    return Q;
}

using pclPoint = pcl::PointXYZRGB;
using pclCloud = pcl::PointCloud<pclPoint>;

void savePointCloud(std::string filename, cv::Mat points, cv::Mat colors, double max_z) {
    pclPoint p_default;
    pclCloud::Ptr dst(new pclCloud(points.rows, points.cols, p_default));
    for (int i = 0; i < points.rows; i++) {
        for (int j = 0; j < points.cols; j++) {
            cv::Vec3f xyz = points.at<cv::Vec3f>(i, j);
            cv::Vec3b bgr = colors.at<cv::Vec3b>(i, j);
            // Check if points are too far away, if not take them into account
            if (fabs(xyz[2]) < max_z) {
                pclPoint pn;
                pn.x = xyz[0];
                pn.y = xyz[1];
                pn.z = xyz[2];
                pn.r = bgr[2];
                pn.g = bgr[1];
                pn.b = bgr[0];
                dst->at(i, j) = pn;
            }
        }
    }
    pcl::io::savePCDFileASCII(filename, *dst);
}