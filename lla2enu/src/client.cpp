#include "ros/ros.h"
#include "lla2enu/ComputeDistance.h"
#include "lla2enu/Distance.h"



int main(int argc, char **argv)
{
	/* FAI IL CASO IN CUI NON ARRIVA UN VALORE
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }*/
  
  
  float par1;
  float par2;

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<service::ComputeDistance>("compute_distance");
	
	ros::Publisher distance_pub = n.advertise<ll2enu::Distance>("distance", 1000);
	
	
	ros::Rate loop_rate(10);
	
	
  service::ComputeDistance a;
	
	
	
	
	
	
  a.request.car1; // = inserire valori;
  a.request.car2;
  a.request.car3;
  
  a.request.obst1
  a.request.obst2
  a.request.obst3
	  
	  lla2enu::Distance s;
  
  if (client.call(a))
  {
    n.getParam("/trashold1", par1);
    n.getParam("/trashold2", par2);
    
    lla2enu::distance dis= a.response.dist;
    //-1 = cash, 0= rschio, 1= sicuro
    if (s.dis> par2)
    {
    	s.stato=1;
	}
	else if (s.dis> par1 && s.dis< par2)
    {
    	s.stato=0;
	}
	else if (s.dis< par1)
    {
    	.stato=-1;
	}
	
	while (ros::ok()){
		distance_pub.publish(s);
		
		
    		ros::spinOnce();

    		loop_rate.sleep();
	}

  }
  else
  {
    ROS_ERROR("Failed to call service compute_distance");
    return 1;
  }

  return 0;
}
