/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <eigen3/Eigen/Eigen>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
//#include <sensor_msgs/Image.h>
#include <geometry_msgs/Polygon.h>
// #include "src/cpp/udp_comm.h"

using namespace UNITREE_LEGGED_SDK;

// high cmd
constexpr uint16_t TARGET_PORT = 8082;
constexpr uint16_t LOCAL_PORT = 8081;
constexpr char TARGET_IP[] = "192.168.123.220";   // target IP address
boost::mutex traj_mutex;

//// low cmd
//constexpr uint16_t TARGET_PORT = 8007;
//constexpr uint16_t LOCAL_PORT = 8082;
//constexpr char TARGET_IP[] = "192.168.123.10";   // target IP address

namespace EXTRA_FUNT{

    bool areVectorsEqual(const std::vector<Eigen::Vector2d>& a, const std::vector<Eigen::Vector2d>& b)
    {
        if (a.size() != b.size())
            return false;

        for (size_t i = 0; i < a.size(); ++i)
        {
            if (!a[i].isApprox(b[i]))
                return false;
        }

        return true;
    }
}

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::Aliengo),
        udp(LOCAL_PORT, TARGET_IP,TARGET_PORT, sizeof(HighCmd), sizeof(HighState))
    {
        udp.InitCmdData(cmd);
            //init value, for debugging
        // traj.push_back(Eigen::Vector2d(1.0,0.0));
        // traj.push_back(Eigen::Vector2d(1.0,-1.0));
        // traj.push_back(Eigen::Vector2d(0,0));
        //traj.push_back(Eigen::Vector2d(0.0,0.0));
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void Traj2Action();
    void GrabTraj(const geometry_msgs::Polygon::ConstPtr& msg);

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    float motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    bool init_state = true;
    float arrive_thr = 0.01; //1cm
    float theta = 0;

    std::vector<Eigen::Vector2d> traj;
    int next_wp_index = {0};
    //initialize false, waitting fir a trajectory
    bool receive_flag = false; 
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}


void Custom::Traj2Action()
{
    if(receive_flag)
    {
        boost::mutex::scoped_lock lock(traj_mutex);
        udp.GetRecv(state);
        //std::cout << "motiontime:\t" << motiontime << " " << state.imu.rpy[2] << "\n";
        //init state
        {
            cmd.velocity[0] = 0.0f;
            cmd.velocity[1] = 0.0f;
            cmd.position[0] = 0.0f;
            cmd.position[1] = 0.0f;
            cmd.yawSpeed = 0.0f;

            cmd.mode = 0;
            cmd.rpy[0]  = 0;
            cmd.rpy[1] = 0;
            cmd.rpy[2] = 0;
            cmd.gaitType = 0;
            cmd.dBodyHeight = 0;
            cmd.dFootRaiseHeight = 0;
        }


        float next_wp_x = traj[next_wp_index][0];
        float next_wp_y = traj[next_wp_index][1];


        ///theta = arcos(cos(theta))
        //TODO:need to fix
        float vector_x = next_wp_x - state.position[0];
        float vector_y = next_wp_y - state.position[1];
        float distance = sqrt(vector_x*vector_x + vector_y*vector_y);  
        theta = vector_y>0 ? acos(vector_x/distance):-acos(vector_x/distance);


        cmd.mode = 3; // mode 3: walk to target position in ground frame
        cmd.gaitType = 0;
        cmd.speedLevel = 0; // adjust speedlevel
        cmd.position[0] = next_wp_x;
        cmd.position[1] = next_wp_y;
        cmd.rpy[2] = theta;
        cmd.dBodyHeight = 0;
        udp.SetSend(cmd);

        if(fabs(state.position[0]-cmd.position[0])<arrive_thr && fabs(state.position[1]-cmd.position[1])<arrive_thr)
        {
            if(next_wp_index<traj.size()-1)
            {
                next_wp_index++;
                std::cout <<"<<<<<<<<<<< going to next position now <<<<<<<<<<<"<< std::endl;
                std::cout <<"current position: "<<state.position[0] <<" "<<state.position[1]<<" "<<state.position[2]<< 
                " "<<" next target position: "<<traj[next_wp_index][0]<<" "<<traj[next_wp_index][1]<<std::endl;
            }

            else{

                std::cout<<"<<<<<<<<<<< Navigation finished <<<<<<<<<<<"<<std::endl;
                receive_flag = false;
                //traj.clear();
            }

        }

    }




}

void Custom::GrabTraj(const geometry_msgs::Polygon::ConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
   boost::mutex::scoped_lock lock(traj_mutex);
   //std::cout<<"received trajectory"<<std::endl;
   std::vector<Eigen::Vector2d> traj_temp;
   for(const auto& point : msg->points)
   {
       Eigen::Vector2d wp;
       wp[0] = point.x;
       wp[1] = point.y;
       traj_temp.push_back(wp);
   }
   if(!EXTRA_FUNT::areVectorsEqual(traj_temp, traj))
   {
        receive_flag = true;
        traj = traj_temp;
        std::cout<<"received vaild trajectory"<<std::endl;

   }
    
//    std::cout<<"trajectory is as follows:"<<std::endl;
//    for(const auto waypoint : traj)
//    {
//     std::cout<<waypoint<<std::endl;
//    }
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_to_action");
    
    ROS_INFO("ROS_MASTER_URI: %s", getenv("ROS_MASTER_URI"));
    ROS_INFO("ROS_IP: %s", getenv("ROS_IP"));
    //ros::start();

    ros::NodeHandle nh;

    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);


    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();




    Custom custom(HIGHLEVEL);
    InitEnvironment();
    //traj subscriber
    ros::Subscriber sub = nh.subscribe("planner/trajectory", 1, &Custom::GrabTraj, &custom);
    //boost::bind(&Custom::Traj2Action, &custom);
    LoopFunc loop_traj2action("udp_control", custom.dt, boost::bind(&Custom::Traj2Action,  &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));
    loop_udpSend.start();
    loop_udpRecv.start();
    loop_traj2action.start();


    // while(1){
    //     sleep(10);
    // };

    ros::spin();
    

    return 0;
}
