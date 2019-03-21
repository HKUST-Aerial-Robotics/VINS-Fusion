/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

// These are added by mpkuse and the purpose is to ignore incoming images and imus when the state
// is 'kidnapped'
bool rcvd_tracked_feature = true;
bool rcvd_imu_msg = true;



cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

int knd = 0; // how many times inside `if`
int cnd = 0; // how many times kidnapped (mean is low and std dev is low)
int bnd = 0; // how many times in `if` and looks like unkidnapped
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    if( rcvd_tracked_feature == false ) {


        // continue publishing /vins_estimator/keyframe_point.
        knd++;
        if( knd%5 != 0 )
            return;

        ROS_INFO( "[img0_callback] Ignoring Tracked Features" );
        // fake_publish( 20 );
        cv::Mat ximage0 = getImageFromMsg(img_msg);
        // cv::Scalar ans = cv::mean( ximage0 );
        // cout << ans << endl;

        cv::Scalar xmean, xstd;
        cv::meanStdDev( ximage0, xmean, xstd );
        cout << "xmean: " << xmean[0] << "\t" << "xstd: "  << xstd[0] << endl;;


        if( xmean[0] < 35. && xstd[0] < 15. )
            cnd++;
        else
            bnd++;

        if( bnd > 10 ) {
            fake_publish(img_msg->header, 100);
            return;
        }
        if( cnd > 10 ) {
            // fake_publish(img_msg->header, 10);
            // return ;
        }


        return;
    }

    cnd = 0; knd=0; bnd=0;
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    if( rcvd_tracked_feature == false ) {
        // ROS_INFO( "[img1_callback] Ignoring Tracked Features" );
        return;
    }

    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}



// extract images with same timestamp from two topics
void sync_process()
{
    // while(1)
    while(ros::ok())
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                if(time0 < time1)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if( rcvd_imu_msg == false ) {
        // ROS_INFO( "Ignoring IMU message" );
        return;
    }

    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if( rcvd_tracked_feature == false ) {
        ROS_WARN( "[feature_callback] Ignoring Tracked Features" );
        return;
    }

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}



void rcvd_inputs_callback( const std_msgs::BoolConstPtr& rcvd_ ) {

    if( rcvd_->data == true && rcvd_tracked_feature==false && rcvd_imu_msg==false ) {
        ROS_INFO( "\n######################## rcvd_= true. So from now on I start reading the image and imu messages." );
        rcvd_tracked_feature = true;
        rcvd_imu_msg = true;

        // start the thread
        estimator.clearState();
        estimator.clearVars();
        estimator.setParameterOnly();
        estimator.processThread_swt = true;
        estimator.startProcessThread();

        return;
    }

    if( rcvd_->data == false && rcvd_tracked_feature==true && rcvd_imu_msg==true ) {
        ROS_INFO( "\n######################## rcvd_= false. Will reset the vins_estimator now" );
        rcvd_tracked_feature = false;
        rcvd_imu_msg = false;

        // stop the processing thread.
        estimator.processThread_swt = false;
        estimator.processThread.join();

        // empty the queues and restart the estimator
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        while(!img0_buf.empty())
            img0_buf.pop();
        while(!img1_buf.empty())
            img1_buf.pop();
        m_buf.unlock();

        ROS_INFO( "all the queues have been emptied");
        estimator.clearState();
        estimator.clearVars();
        return;
    }

    ROS_INFO( "Ignoring rcvd_ message, because it seems invalid." );

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    // estimator.setParameter();

    estimator.setParameterOnly();
    estimator.processThread_swt = true;
    estimator.startProcessThread();


#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    //< If you send a true will enable receiving sensor data, if you send false,
    // will start ignoring sensor data
    ros::Subscriber sub_rcvd_flag = n.subscribe("/feature_tracker/rcvd_flag", 2000, rcvd_inputs_callback);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);

    std::thread sync_thread{sync_process};
    ros::spin();

    estimator.processThread_swt = false;
    estimator.processThread.join();
    sync_thread.join();

    return 0;
}
