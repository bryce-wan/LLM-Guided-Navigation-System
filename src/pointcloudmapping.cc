/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/projection_matrix.h>
#include "Converter.h"

#include <boost/make_shared.hpp>
#include <pcl/keypoints/uniform_sampling.h>


PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >();
    currentPosition = make_shared< Eigen::Vector2d >();
    //PointCloud::Ptr cloud_source (new PointCloud);
    //globalMap = cloud_source;
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::alignGlobalMap, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );

    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>4)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;

            p.r = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.b = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    //test downsample
    //PointCloud::Ptr tmp(new PointCloud());
    // cout<<"downsample!!!<<edn"
    // int ori_size = cloud->points.size(); 
    // voxel.setInputCloud( cloud );
    // voxel.filter( *cloud );
    // cout<< "gobal map downsampling " <<ori_size<<"to "<< cloud->points.size() << endl;
    //globalMap->swap( *tmp );

    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<". the current global map size is "<<this->globalMap->points.size()<<endl;
    return cloud;
}


pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::getGlobalMap()
{

    return this->globalMap;
}

shared_ptr<Eigen::Vector2d> PointCloudMapping::getCurrentPosition() 
{
    return this->currentPosition;
}

void PointCloudMapping::alignGlobalMap()
{
    //TODO
    while (true)
    {
        
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {   
                cout<<"shut down!!!!!!"<<endl;
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            *globalMap += *p;
            Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( keyframes[i]->GetPose() );
            Eigen::Vector3d cameraCenter(0,0,0);
            Eigen::Vector3d cameraCenterTransformed = T.inverse() * cameraCenter;
            *currentPosition = Eigen::Vector2d(cameraCenterTransformed(0), cameraCenterTransformed(2));
            cout<<"camera center in world coordinate: "<<cameraCenterTransformed.transpose()<<endl;
        }
        lastKeyframeSize = N;
    }
    
}
void PointCloudMapping::viewer()
{
    
    pcl::visualization::CloudViewer viewer("viewer");
    //int count = 0;
    while(1)
    {    cout<<"viewer start!!!"<<endl;
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {   
                cout<<"shut down!!!!!!"<<endl;
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            *globalMap += *p;
        }
        // PointCloud::Ptr tmp(new PointCloud());
        // pcl::UniformSampling<PointT> usample;
        // usample.setInputCloud(globalMap);
        // usample.setRadiusSearch(0.01f);
        // usample.filter(*tmp);
        // cout<<"after downsampling: "<<tmp->points.size()<<endl;
        // cout<<"debug!!!"<<endl;
        // float leafSize = 0.01f;
        // PointCloud::Ptr tmp(new PointCloud());
        // pcl::VoxelGrid<PointT>  voxel_test;
        // cout<<"debug1!!!"<<endl;
        // voxel_test.setInputCloud(globalMap);
        // cout<<"debug2!!!"<<endl;
        // voxel_test.setLeafSize(leafSize, leafSize, leafSize);
        // cout<<"debug3!!!"<<endl;
        // voxel_test.filter(*tmp);
        // cout<<"debug4!!! num: "<<tmp->points.size()<<endl;
        //count ++;
        //cout<<"global point size: "<<globalMap->points.size()<<endl;
        // PointCloud::Ptr tmp(new PointCloud());
        // int ori_size = globalMap->points.size(); 
        // voxel.setInputCloud( globalMap );
        // voxel.filter( *globalMap );
        // cout<< "gobal map downsampling " <<ori_size<<"to "<< globalMap->points.size() << endl;
        //globalMap->swap( *tmp );
        viewer.showCloud(globalMap);
        //pcl::io::savePCDFile("globalMap.pcd", *globalMap);
        cout << "show global map, size=" << globalMap->points.size() << endl;
        lastKeyframeSize = N;
    }
}

