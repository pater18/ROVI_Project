#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/point_types.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include "preprocessing.h"



using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;

typedef PointNormal PointT;
typedef Histogram<153> FeatureT;

void nearest_feature(const FeatureT& query, const pcl::PointCloud<FeatureT>& target, int &idx, float &distsq);



void spatialFilter( pcl::PointCloud<PointT>::Ptr input_cloud, pcl::PointCloud<PointT>::Ptr &output_cloud )
{
	
  // Create the filtering object
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1.5, -1);
  pass.filter (*output_cloud);

  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.371274, 0.197482);
  pass.filter (*output_cloud);

}

Matrix4f poseEstimatePCLAnders(const std::string object_name, const std::string scene_name )
{

    // Load
    pcl::PointCloud<PointT>::Ptr object(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
    pcl::io::loadPLYFile<PointT> (object_name, *object);
    pcl::io::loadPCDFile<PointT> (scene_name, *scene);

    {
        PCLVisualizer v("Before global alignment");
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.spin();
        v.close ();
    }
    
    
    // Compute surface normals
    {
        ScopeTime t("Surface normals");
        NormalEstimation<PointT,PointT> ne;
        ne.setKSearch(10);
        
        ne.setInputCloud(object);
        ne.compute(*object);
        
        ne.setInputCloud(scene);
        ne.compute(*scene);
    }

    spatialFilter(scene, scene);
    preprocess(scene, scene);
    preprocess(object, object);


    pcl::PointNormal minPt, maxPt;
    pcl::getMinMax3D (*scene, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;

    {
        PCLVisualizer v("Before global alignment");
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.spin();
    }

    
    // Compute shape features
    pcl::PointCloud<FeatureT>::Ptr object_features(new pcl::PointCloud<FeatureT>);
    pcl::PointCloud<FeatureT>::Ptr scene_features(new pcl::PointCloud<FeatureT>);
    {
        ScopeTime t("Shape features");
        
        SpinImageEstimation<PointT,PointT,FeatureT> spin;
        spin.setRadiusSearch(0.15);
        
        spin.setInputCloud(object);
        spin.setInputNormals(object);
        spin.compute(*object_features);
        
        spin.setInputCloud(scene);
        spin.setInputNormals(scene);
        spin.compute(*scene_features);
    }
    
    // Find feature matches
    Correspondences corr(object_features->size());
    {
        ScopeTime t("Feature matches");
        for(size_t i = 0; i < object_features->size(); ++i) {
            corr[i].index_query = i;
            nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
        }
    }
    
    // Show matches
    {
        PCLVisualizer v("Matches");
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.addCorrespondences<PointT>(object, scene, corr, 1);
        v.spin();
    }
    
    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);
    
    // Set RANSAC parameters
    const size_t iter = 2000;
    const float thressq = 0.01 * 0.01;
    
    // Start RANSAC
    Matrix4f pose = Matrix4f::Identity();
    pcl::PointCloud<PointNormal>::Ptr object_aligned(new pcl::PointCloud<PointNormal>);
    float penalty = FLT_MAX;
    {
        ScopeTime t("RANSAC");
        cout << "Starting RANSAC..." << endl;
        UniformGenerator<int> gen(0, corr.size() - 1);
        for(size_t i = 0; i < iter; ++i) {
            if((i + 1) % 100 == 0)
                cout << "\t" << i+1 << endl;
            // Sample 3 random correspondences
            vector<int> idxobj(3);
            vector<int> idxscn(3);
            for(int j = 0; j < 3; ++j) {
                const int idx = gen.run();
                idxobj[j] = corr[idx].index_query;
                idxscn[j] = corr[idx].index_match;
            }
            
            // Estimate transformation
            Matrix4f T;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T);
            
            // Apply pose
            transformPointCloud(*object, *object_aligned, T);
            
            // Validate
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
            
            // Compute inliers and RMSE
            size_t inliers = 0;
            float rmse = 0;
            for(size_t j = 0; j < distsq.size(); ++j)
                if(distsq[j][0] <= thressq)
                    ++inliers, rmse += distsq[j][0];
            rmse = sqrtf(rmse / inliers);
            
            // Evaluate a penalty function
            const float outlier_rate = 1.0f - float(inliers) / object->size();
            //const float penaltyi = rmse;
            const float penaltyi = outlier_rate;
            
            // Update result
            if(penaltyi < penalty) {
                cout << "\t--> Got a new model with " << inliers << " inliers!" << endl;
                penalty = penaltyi;
                pose = T;
                
            }
        }
        
        transformPointCloud(*object, *object_aligned, pose);
        
        // Compute inliers and RMSE
        vector<vector<int> > idx;
        vector<vector<float> > distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < distsq.size(); ++i)
            if(distsq[i][0] <= thressq)
                ++inliers, rmse += distsq[i][0];
        rmse = sqrtf(rmse / inliers);
    
        // Print pose
        cout << "Got the following pose:" << endl << pose << endl;
        cout << "Inliers: " << inliers << "/" << object->size() << endl;
        cout << "RMSE: " << rmse << endl;
    } // End timing
    
    // Show result
    {
        PCLVisualizer v("After global alignment");
        v.addPointCloud<PointT>(object_aligned, PointCloudColorHandlerCustom<PointT>(object_aligned, 0, 255, 0), "object_aligned");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.spin();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree2;
    tree2.setInputCloud(scene);
    
    // Set ICP parameters
    const size_t iter2 = 100;
    const float thressq2 = 0.01 * 0.01;
    
    // Start ICP
    Matrix4f pose2 = Matrix4f::Identity();
    //Matrix4f pose2 = pose;
    pcl::PointCloud<PointNormal>::Ptr object_aligned2(new pcl::PointCloud<PointNormal>(*object_aligned));

    {
        ScopeTime t("ICP");
        cout << "Starting ICP..." << endl;
        for(size_t i = 0; i < iter2; ++i) {
            // 1) Find closest points
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree2.nearestKSearch(*object_aligned2, std::vector<int>(), 1, idx, distsq);
            
            // Threshold and create indices for object/scene and compute RMSE
            vector<int> idxobj;
            vector<int> idxscn;
            for(size_t j = 0; j < idx.size(); ++j) {
                if(distsq[j][0] <= thressq2) {
                    idxobj.push_back(j);
                    idxscn.push_back(idx[j][0]);
                }
            }
            
            // 2) Estimate transformation
            Matrix4f T;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object_aligned2, idxobj, *scene, idxscn, T);
            
            // 3) Apply pose2
            transformPointCloud(*object_aligned2, *object_aligned2, T);
            
            // 4) Update result
            pose2 = T * pose2;
        }
        
        // Compute inliers and RMSE
        vector<vector<int> > idx;
        vector<vector<float> > distsq;
        tree2.nearestKSearch(*object_aligned2, std::vector<int>(), 1, idx, distsq);
        size_t inliers = 0;
        float rmse = 0;
        for(size_t i = 0; i < distsq.size(); ++i)
            if(distsq[i][0] <= thressq2)
                ++inliers, rmse += distsq[i][0];
        rmse = sqrtf(rmse / inliers);
    
        // Print pose2
        cout << "Got the following pose2:" << endl << pose2 << endl;
        cout << "Inliers: " << inliers << "/" << object->size() << endl;
        cout << "RMSE: " << rmse << endl;
    } // End timing
    
    // Show result
    {
        PCLVisualizer v("After local alignment");
        v.addPointCloud<PointNormal>(object_aligned2, PointCloudColorHandlerCustom<PointNormal>(object_aligned2, 0, 255, 0), "object_aligned");
        v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
        v.spin();
    }

    //Matrix4f final_inverse = pose * pose2;
    //final_inverse = final_inverse.inverse();

    std::cout << "Final pose :" << std::endl << pose * pose2 << std::endl;
    std::cout << "Final pose inverse :" << std::endl << pose2 * pose << std::endl;
    //std::cout << "Final pose inverse :" << std::endl << final_inverse << std::endl;

    
    return pose2;
}

inline float dist_sq(const FeatureT& query, const FeatureT& target) {
    float result = 0.0;
    for(int i = 0; i < FeatureT::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }
    
    return result;
}

void nearest_feature(const FeatureT& query, const pcl::PointCloud<FeatureT>& target, int &idx, float &distsq) {
    idx = 0;
    distsq = dist_sq(query, target[0]);
    for(size_t i = 1; i < target.size(); ++i) {
        const float disti = dist_sq(query, target[i]);
        if(disti < distsq) {
            idx = i;
            distsq = disti;
        }
    }
}





