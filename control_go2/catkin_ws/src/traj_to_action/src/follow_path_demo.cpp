#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>
#include <math.h>
#include <eigen3/Eigen/Eigen>


class Custom
{
public:
  Custom() {}
  void control();

  unitree::robot::go2::SportClient tc;

  int c = 0;
  float dt = 0.002; // 0.001~0.01
};

void Custom::control()
{

    std::vector<Eigen::Vector2d> traj;
    traj.push_back(Eigen::Vector2d(1,0));
    traj.push_back(Eigen::Vector2d(0,0));
    traj.push_back(Eigen::Vector2d(0,1));
    c++;
    
    int32_t ret;

    float vx = 0.3;
    float delta = 0.06;
    static float count = 0;
    count += dt;
    std::vector<unitree::robot::go2::PathPoint> path;
    for (int i=0; i<traj.size(); i++) {
      unitree::robot::go2::PathPoint p;
      //float var = (count + i * delta);
      p.timeFromStart = i * delta;
      p.x = traj[i][0];
      p.y = traj[i][1];
      float current_state_x;
      float current_state_y;
      if (i == 0) //specialize the first point
        {
          current_state_x = 0;
          current_state_y = 0;
        }
      else
        {
          current_state_x = traj[i-1][0];
          current_state_y = traj[i-1][1];
        }
      float vector_x = p.x - current_state_x;
      float vector_y = p.y - current_state_y;
      float distance = sqrt(vector_x*vector_x + vector_y*vector_y);  
      float yaw = vector_y>0 ? acos(vector_x/distance):-acos(vector_x/distance);

      p.yaw = yaw;
      p.vx = vx;
      p.vy = vx;
      p.vyaw = vx;
      path.push_back(p);
    }

    ret = tc.TrajectoryFollow(path);
    if(ret != 0){
      std::cout << "Call TrajectoryFollow: " << ret << std::endl;
    }

    std::cout << c << std::endl;
}

int main(int argc, char** argv)
{
  unitree::robot::ChannelFactory::Instance()->Init(0);

  Custom custom;
  custom.tc.SetTimeout(10.0f);
  custom.tc.Init();

  unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::control, &custom));

  while (1)
  {
    sleep(10);
  }

  return 0;
}
