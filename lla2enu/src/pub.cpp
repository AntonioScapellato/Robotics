#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <sstream>


int main(int argc, char **argv){
    
	ros::init(argc, argv, "publisher");
	ros::NodeHandle n;
	
	//pub e sub devono avere lo stesso tipo di messaggio

	ros::Publisher pub1 = n.advertise<sensor_msgs::NavSatFix>("car", 1000);
	ros::Publisher pub2 = n.advertise<sensor_msgs::NavSatFix>("obstacle", 1000);

	ros::Rate loop_rate(1);
	
	//PERCHE QUESTO COUNTER?	
	int count = 0;
  
  
  	while (ros::ok()){
	    
		sensor_msgs::NavSatFix msg1;
		sensor_msgs::NavSatFix msg2;
		
		/*
		msg1.header.stamp = ros::Time::now();
		msg1.header.frame_id = "f1";
		//PRENDI DA FILE BAG
		msg1.vector.x = 1;
		msg1.vector.y = 1;
		msg1.vector.z = 1;
		
		msg2.header.stamp = ros::Time::now();
		msg2.header.frame_id = "f2";
		msg2.vector.x = 2;
		msg2.vector.y = 2;
		msg2.vector.z = 2;
*/
		
		n.getParam("/originPointLatitude", msg1.latitude);
		n.getParam("/originPointLongitude", msg1.longitude);
		n.getParam("/originPointAltitude", msg1.altitude);
		
		//prendi valori ostacolo e mettili in msg2

    		pub1.publish(msg1);
    		pub2.publish(msg2);
    		ROS_INFO ("Publishing message");
    		ros::spinOnce();
    		loop_rate.sleep();
    		++count;
  	}

  	return 0;
}

