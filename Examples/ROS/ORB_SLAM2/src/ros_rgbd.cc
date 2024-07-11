/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <opencv2/core/core.hpp>

#include <System.h>
#include <time.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh; 

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));


    //publish dense global map pointcloud

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGBA>);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
    ros::Rate loop_rate(10);
    global_map = SLAM.GetPointCloudMapping()->getGlobalMap();

    shared_ptr<Eigen::Vector2d> startPoint_ptr = SLAM.GetPointCloudMapping()->getCurrentPosition();
    //printf("start point: %f, %f\n", SLAM.GetPointCloudMapping()->getCurrentPosition()(0), SLAM.GetPointCloudMapping()->getCurrentPosition()(1));
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/orbslam2/pointcloud", 100);
    ros::Publisher startPoint_pub = nh.advertise<geometry_msgs::Point>("/orbslam2/startpoint", 1);
    float cameraToGround = SLAM.cameraToGround;
    float camera_height = SLAM.camera_height;

    while (ros::ok())
    {
        pcl::copyPointCloud(*global_map, *global_map_copy);
        //transform the pointcould to be parallel to the ground
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        //user-defined parameters, specific to robot camera rotation to the ground
        //TODO:check the trajectory if should be inverse transformed
        transform.rotate(Eigen::AngleAxisf(-M_PI/2*cameraToGround, Eigen::Vector3f(1,0,0)));
        transform.translate(Eigen::Vector3f(0,camera_height,0));
        pcl::transformPointCloud(*global_map_copy, *global_map_copy, transform);
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*global_map_copy,output);

        ROS_INFO("global map points size: %d", global_map_copy->points.size());
        output.header.stamp=ros::Time::now();
        output.header.frame_id  ="global_map";
        pcl_pub.publish(output);

        geometry_msgs::Point startPointMsg;
        startPointMsg.x = startPoint_ptr->x();
        startPointMsg.y = startPoint_ptr->y();
        printf("start point: %f, %f\n", startPoint_ptr->x(), startPoint_ptr->y());/
        startPoint_pub.publish(startPointMsg);


        ros::spinOnce();
        loop_rate.sleep();
    }
    if(SLAM.savePointCloud)
    {
        time_t currentTime = time(NULL);
	    char chCurrentTime[256];
	    strftime(chCurrentTime, sizeof(chCurrentTime), "%m%d%H%M", localtime(&currentTime));
        string stCurrentTime = chCurrentTime;
        string filename = "./maps/global_map_" + stCurrentTime + ".pcd";
        pcl::io::savePCDFileASCII (filename, *global_map);
        cout << endl << "global_map saved! It has " <<global_map->points.size()<<" points."<< endl;
    }
    //pcl::io::savePCDFileASCII ("global_map.pcd", *global_map);
    //cout << endl << "global_map saved! It has " <<global_map->points.size()<<" points."<< endl;
    //ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}


