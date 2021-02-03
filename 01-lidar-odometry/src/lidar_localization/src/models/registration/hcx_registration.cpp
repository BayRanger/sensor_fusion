/*
 * @Reference: https://github.com/zjudmd1015/icp/blob/master/test.cpp
 * https://libpointmatcher.readthedocs.io/en/latest/BasicRegistration/
 * https://www.programmersought.com/article/42165950844/
 * @Description: ICP 匹配模块
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */
#include "lidar_localization/models/registration/hcx_registration.hpp"
#include "lidar_localization/models/registration/ICP.h"

#include "glog/logging.h"

namespace lidar_localization {

HCXRegistration::HCXRegistration(
    const YAML::Node& node
) : hcx_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {
    
    float max_corr_dist = node["max_corr_dist"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

HCXRegistration::HCXRegistration(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) : hcx_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

bool HCXRegistration::SetRegistrationParam(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) {
    hcx_ptr_->setMaxCorrespondenceDistance(max_corr_dist);
    hcx_ptr_->setTransformationEpsilon(trans_eps);
    hcx_ptr_->setEuclideanFitnessEpsilon(euc_fitness_eps);
    hcx_ptr_->setMaximumIterations(max_iter);

    LOG(INFO) << "HCX params:" << std::endl
              << "max_corr_dist: " << max_corr_dist << ", "
              << "trans_eps: " << trans_eps << ", "
              << "euc_fitness_eps: " << euc_fitness_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool HCXRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    hcx_ptr_->setInputTarget(input_target);

    return true;
}
/*
@predict: the estimated pose based on motion model


*/
bool HCXRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, //reference
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,  //align
                                Eigen::Matrix4f& result_pose) {
    //设置当前帧
    //pcl::PointCloud<pcl::PointXYZ>::Ptr in_source_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //*in_source_ptr = *input_source;
    //pcl::copyPointCloud(input_source, inptr);
  

    //执行匹配 根据predict pose 转换 result_cloud_ptr, 
    Eigen::Affine3f ini_pose =  Eigen::Affine3f::Identity();
    ini_pose.matrix() = predict_pose;
    pcl::transformPointCloud(*result_cloud_ptr,*result_cloud_ptr,ini_pose);
    Eigen::Matrix3Xd X ( 3, input_source->size() ); // source  map
    Eigen::Matrix3Xd Y ( 3, result_cloud_ptr->size() ); // target
    for(int i = 0; i < cloud_source->size(); i++)
    {
      X(0,i) = input_source->points[i].x;
      X(1,i) = input_source->points[i].y;
      X(2,i) = input_source->points[i].z;
    }
    for(int i = 0; i < cloud_target->size(); i++)
    {
      Y(0,i) = result_cloud_ptr->points[i].x;
      Y(1,i) = result_cloud_ptr->points[i].y;
      Y(2,i) = result_cloud_ptr->points[i].z;
    }
    
    Eigen::Affine3d pose_tmp = SICP::point_to_point ( X, Y ); // sparse ICP

    result_pose =  pose_tmp.matrix() + predict_pose;

    return true;
}

}