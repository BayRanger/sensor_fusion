/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic) {
    //std::string pointcloud_topic = "/kitti/velo/pointcloud";
    std::string pointcloud_topic = "/velodyne_points";//"/kitti/velo/pointcloud";
    //std::string imu_topic = "/kitti/oxts/imu";
    std::string imu_topic = "/imu/data";
    //std::string gnss_topic ="/kitti/oxts/gps/fix";
    std::string gnss_topic ="/navsat/fix";
    //std::string imu_tf ="/imu_link";
    std::string lidar_tf ="/velodyne";
    std::string odom_topic = "/navsat/odom";
    std::string vel_topic = "/kitti/oxts/gps/vel";


    // subscriber
    // a. velodyne measurement:
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, pointcloud_topic, 100000);
    // b. OXTS IMU:
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, imu_topic, 1000000);
    // c. OXTS velocity:
    //TODO:
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, odom_topic, 1000000);
    // d. OXTS GNSS:
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, gnss_topic, 1000000);
    //lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, imu_tf, lidar_tf);

    // publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, lidar_tf, 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", lidar_tf, 100);

    // motion compensation for lidar measurement:
    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() {
    if (!ReadData())
        return false;

    if (!InitCalibration()) 
        return false;

    if (!InitGNSS())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        TransformData();
        PublishData();
    }
    //LOG(INFO) << "RUN works" << std::endl;


    return true;
}

bool DataPretreatFlow::ReadData() {
    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_;

    // fetch lidar measurements from buffer:
   // LOG(INFO) << "Start to read data...." << std::endl;

    cloud_sub_ptr_->ParseData(cloud_data_buff_);
 
    imu_sub_ptr_->ParseData(unsynced_imu_);
    //        LOG(INFO) << "Parse gnss...." << std::endl;

    velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if (cloud_data_buff_.size() == 0)
    {
       //LOG(INFO) << "No cloud data...." << std::endl;

        return false;

    }

    // use timestamp of lidar measurement as reference:
    double cloud_time = cloud_data_buff_.front().time;
    // sync IMU, velocity and GNSS with lidar measurement:
    // find the two closest measurement around lidar measurement time
    // then use linear interpolation to generate synced measurement:
    //The data in the cloud_data_buff_ has already been synchronized 
    //LOG(INFO) << "Start to read data works" << std::endl;

    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time, 0.02);
    bool valid_velocity =  VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_ , cloud_time, 2);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time,2);
    //bool valid_velocity = false;
    //if (valid_gnss &&valid_imu)
    //{            //LOG(INFO) << "GNSS is valid....." << std::endl;

    //    valid_velocity =  VelocityData::SyncDataFromGnssIMU(gnss_data_buff_,imu_data_buff_, velocity_data_buff_,cloud_time);
    //}
    // only mark lidar as 'inited' when all the three sensors are synced:
    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu || !valid_velocity || !valid_gnss) {
            if (!valid_imu)
            {
            LOG(INFO) << "imu is invalid....." << std::endl;
             }
            if (!valid_velocity)
            {
            LOG(INFO) << "velocity is invalid....." << std::endl;
             }            
            if (!valid_gnss)
            {
            LOG(INFO) << "gnss is invalid....." << std::endl;
             }   
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }
            //LOG(INFO) << "Read data works" << std::endl;


    return true;
}

bool DataPretreatFlow::InitCalibration() {
    // lookup imu pose in lidar frame:
    static bool calibration_received = false;
    if (!calibration_received) {
        //if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
            Eigen::Matrix4f imu_to_lidar;
            imu_to_lidar<<  2.67949e-08, -1,  0, 0,
           1,  2.67949e-08,  0, 0,
           0,  0,  1, -0.28, 
           0., 0., 0., 1;
            lidar_to_imu_ = imu_to_lidar.inverse();

            LOG(INFO)<<"InitCalibration works"<< std::endl;
        //}
    }

    return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (velocity_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;
    //LOG(INFO) << "Has Data works" << std::endl;

    return true;
}

bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

    if (diff_velocity_time > 0.05) {
        velocity_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    //LOG(INFO) << "ValidData works" << std::endl;
    return true;
}

bool DataPretreatFlow::TransformData() {
    // get GNSS & IMU pose prior:
    gnss_pose_ = Eigen::Matrix4f::Identity();
    // a. get position from GNSS
    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    // b. get orientation from IMU:
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    // this is lidar pose in GNSS/map frame:
    gnss_pose_ *= lidar_to_imu_;

    // this is lidar velocity:
    current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    // motion compensation for lidar measurements:
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);
    //LOG(INFO) << "Transform data works" << std::endl;

    return true;
}

bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);
    //LOG(INFO) << "Publish data works" << std::endl;

    return true;
}
}