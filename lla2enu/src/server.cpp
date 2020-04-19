#include "ros/ros.h"
#include "lla2enu/ComputeDistance.h"

bool dis(service::ComputeDistance::Request  &req,
         service::ComputeDistance::Response &res)
{
  res.res.dist = sqrt(((req.car1 - req.obst1)*(req.car1 - req.obst1) )+ ((req.car2 - req.obst2)*(req.car2 - req.obst2) )+ ((req.car3 - req.obst3)* (req.car3 - req.obst3)))
   return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_distance_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("compute_distance", dis);
  ros::spin();

  return 0;
}
