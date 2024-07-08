#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>
#include <math.h>
#include <eigen3/Eigen/Eigen>

// go2 communication
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include <unitree/idl/go2/SportModeState_.hpp>

// ros communication    
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Polygon.h>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

// global
// boost::mutex traj_mutex;
constexpr char TARGET_TRAJ_TOPIC[] = "/traj";
constexpr char TARGET_IP[] = "192.168.123.220";

namespace EXTRA_FUNT
{
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
    Custom() 
    {
        // ????
        sport_client.SetTimeout(10.0f);
        sport_client.Init();

        suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

        // design the traj manually
        // traj.push_back(Eigen::Vector2d(1,0));
        // traj.push_back(Eigen::Vector2d(0,0));
        // traj.push_back(Eigen::Vector2d(0,1));
    }

    // void control();
    void Traj2Action();
    void GrabTraj(const geometry_msgs::Polygon::ConstPtr& msg);
    void GetInitState();
    void HighStateHandler(const void *message);

    unitree_go::msg::dds_::SportModeState_ state;
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::SportModeState_> puber;

    int c = 0;
    float dt = 0.002; // 0.001~0.01
    double px0, py0, yaw0; // initial state
    bool receive_flag = false; // waiting for a traj
    std::vector<Eigen::Vector2d> traj;
};

// Get initial position
void Custom::GetInitState()
{
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
};

// Get the high state of the robot
void Custom::HighStateHandler(const void *message)
{
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;
    // std::cout << "Position: " << state.position()[0] << ", " << state.position()[1] << ", " << state.position()[2] << std::endl;
    // std::cout << "IMU rpy: " << state.imu_state().rpy()[0] << ", " << state.imu_state().rpy()[1] << ", " << state.imu_state().rpy()[2] << std::endl;
};


// Control the robot
void Custom::Traj2Action()
{
    if(receive_flag)
    {
        // boost::mutex::scoped_lock lock(traj_mutex);

        c++;
        int32_t ret;

        float vx = 0.2;
        float time_seg = 0.06;
        static float count = 0;
        count += dt;
        
        unitree::robot::go2::PathPoint p;
        std::vector<unitree::robot::go2::PathPoint> path;

        for (int i = 0; i < traj.size(); i++) 
        {
            //float var = (count + i * delta);  
            p.timeFromStart = i * time_seg;
            p.x = traj[i][0];
            p.y = traj[i][1];

            float vector_x = p.x - state.position()[0];
            float vector_y = p.y - state.position()[1];
            float distance = sqrt(vector_x*vector_x + vector_y*vector_y);  
            float yaw = vector_y>0 ? acos(vector_x/distance):-acos(vector_x/distance);

            p.yaw = yaw;
            p.vx = vx;
            p.vy = vx;
            p.vyaw = vx;
            path.push_back(p);
        }

        ret = sport_client.TrajectoryFollow(path);
        
        if(ret != 0)
        {
            std::cout << "Call TrajectoryFollow: " << ret << std::endl;
        }

        std::cout << "c: " << c << std::endl;

        receive_flag = false;

    }
    
}


// Get the trajectory from the topic
void Custom::GrabTraj(const geometry_msgs::Polygon::ConstPtr& msg)
{
    
    // boost::mutex::scoped_lock lock(traj_mutex);

    std::vector<Eigen::Vector2d> traj_temp;
    for(const auto& point : msg->points)
    {
        Eigen::Vector2d wp;
        wp[0] = point.x;
        wp[1] = point.y;
        traj_temp.push_back(wp);
    }

    // check if the 
    if(!EXTRA_FUNT::areVectorsEqual(traj_temp, traj))
    {
        receive_flag = true;
        traj = traj_temp;
        std::cout << "Received Vaild Trajectory" << std::endl;
    }
    
    // print the trajectory
    std::cout << "the trajectory is as follows:" << std::endl;
    for(const auto waypoint : traj)
    {
        std::cout << waypoint << std::endl;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_to_action");

    // print the environment variables
    ROS_INFO("ROS_MASTER_URI: %s", getenv("ROS_MASTER_URI"));
    ROS_INFO("ROS_IP: %s", getenv("ROS_IP"));

    ros::NodeHandle nh;

    // unitree::robot::ChannelFactory::Instance()->Init(0);
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    
    Custom custom;
    
    custom.sport_client.SetTimeout(10.0f);
    custom.sport_client.Init();
    ros::Subscriber sub = nh.subscribe("planner/trajectory", 1, &Custom::GrabTraj, &custom);

    sleep(1);

    unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::Traj2Action, &custom));

    // while (1)
    // {
    //     sleep(10);
    // }

    ros::spin();

    return 0;
}
