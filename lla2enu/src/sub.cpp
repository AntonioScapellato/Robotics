#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>
#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "lla2enu/Distance.h"
		

void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  ROS_INFO("Input position: [%f,%f, %f]", msg->latitude, msg->longitude,msg->altitude);

  // fixed values

  double a = 6378137;
  double b = 6356752.3142;
  double f = (a - b) / a;
  double e_sq = f * (2-f);
  float deg_to_rad = 0.0174533;
  
  // input data from msg
  float latitude = msg->latitude;
  float longitude = msg->longitude;
  float h = msg->altitude;

  // fixed position
	//USO PARAMETERS:
  /*float latitude_init = 45.6311926152;
  float longitude_init = 9.2947495255;
  float h0 = 231.506675163;*/
	
	
float latitude_init;
  float longitude_init;
  float h0;
	
	
	ros::NodeHandle f;
	f.getParam("/zeroLatitude", latitude_init);
f.getParam("/zeroLongitude", longitude_init);
	f.getParam("/zeroAltitude", h0);

  //lla to ecef
  float lamb = deg_to_rad*(latitude);
  float phi = deg_to_rad*(longitude);
  float s = sin(lamb);
  float N = a / sqrt(1 - e_sq * s * s);

  float sin_lambda = sin(lamb);
  float  cos_lambda = cos(lamb);
  float  sin_phi = sin(phi);
  float  cos_phi = cos(phi);

  float  x = (h + N) * cos_lambda * cos_phi;
  float  y = (h + N) * cos_lambda * sin_phi;
  float  z = (h + (1 - e_sq) * N) * sin_lambda;
  
  ROS_INFO("ECEF position: [%f,%f, %f]", x, y,z);
  

  // ecef to enu
 
  lamb = deg_to_rad*(latitude_init);
  phi = deg_to_rad*(longitude_init);
  s = sin(lamb);
  N = a / sqrt(1 - e_sq * s * s);

  sin_lambda = sin(lamb);
  cos_lambda = cos(lamb);
  sin_phi = sin(phi);
  cos_phi = cos(phi);

  float  x0 = (h0 + N) * cos_lambda * cos_phi;
  float  y0 = (h0 + N) * cos_lambda * sin_phi;
  float  z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

  float xd = x - x0;
  float  yd = y - y0;
  float  zd = z - z0;

  float  xEast = -sin_phi * xd + cos_phi * yd;
  float  yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
  float  zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

  ROS_INFO("ENU position: [%f,%f, %f]", xEast, yNorth,zUp);
  
  // NEW COORDINATES
  msg->latitude = xEast;
  msg->longitude = yNorth;
  msg->altitude = zUp;
  
  
}


void callback(const geometry_msgs::Vector3StampedConstPtr& msg1, const geometry_msgs::Vector3StampedConstPtr& msg2)
{
  ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)", msg1->vector.x,msg1->vector.y,msg1->vector.z, msg2->vector.x, msg2->vector.y, msg2->vector.z);

	//CONVERTING THE CAR POSITION 
	//geometry_msgs::Vector3Stamped position_car;
	//position_car = chatterCallback(msg1);
	
	chatterCallback(msg1);
	chatterCallback(msg2);
	
	
	//CONVERTING THE OBSTACLE POSITION 
	//geometry_msgs::Vector3Stamped position_obstacle;
	//position_obstacle = chatterCallback(msg2);
	//
	// DISTANCE 
	
	
	/*
	// la variabile deve essere del tipo custom message creato: distance
	custom_messages::distance dist;
	
	//ATTENZIONE: controllare num. parentesi, ne ho aggiunta una
	dist = sqrt(((position_car.x - position_obstacle.x)*(position_car.x - position_obstable.x))+((position_car.y - position_obstacle.y)*(position_car.y - position_obstable.y))+((position_car.z - position_obstacle.z)*(position_car.z - position_obstable.z)));
	*/
	
	//CONTROLLO CRASH,    ATTENZIONE: SCRIVERE CODICE QUANDO UGUALE A 1M O 5M)
	// leggo i parametri par1= 1m e par2= 5m 
	
	//ilflag è di tipo custom message: 1=sicuro, 0= non sicuro, -1= cash
	
		

	
	
	// SERVICE CON CUSTOM MESSAGE 
	//STATUS --> ON CLIENT

}


int main(int argc, char **argv){
  	/*
	OLD MAIN lla2enu
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("/swiftnav/front/gps_pose", 1000, chatterCallback);
  	ros::spin();
	*/
	
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle n;

	// RECEIVE THE CAR POSITION
	message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub1(n, "car", 1);
	//RECEIVE THE OBSTACLE POSITION
	message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub2(n, "obstacle", 1);
	message_filters::TimeSynchronizer<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> sync(sub1, sub2, 10);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	
 	ros::spin();
 	return 0;

  return 0;
}


