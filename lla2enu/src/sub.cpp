#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>
#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "lla2enu/ComputeDistance.h"
#include "lla2enu/Distance.h"
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
// AGGIUNGERE LIBRERIA 
		

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
	
	//RETRIEVE ZERO POSITION FROM LAUNCH FILE
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


void callback(const sensor_msgs::NavSatFix::ConstPtr& msg1, const sensor_msgs::NavSatFix::ConstPtr& msg2)
{
  ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)", msg1->vector.x,msg1->vector.y,msg1->vector.z, msg2->vector.x, msg2->vector.y, msg2->vector.z);
	
	//CONVERTING THE POSITIONS 
	chatterCallback(msg1);
	chatterCallback(msg2);
 // DISTANCE CALCULATION AND PUBLISHING 
  
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<service::ComputeDistance>("compute_distance");
  ros::Publisher distance_pub = n.advertise<ll2enu::Distance>("distance", 1000);
  ros::Rate loop_rate(10);
  service::ComputeDistance a;
	
  //SENDING PARAMETERS TO CALCULATE
  a.request.car1 = msg1->latitude; 
  a.request.car2 = msg1->longitude; 
  a.request.car3 = msg1->altitude; 
  a.request.obst1 = msg2->latitude;
  a.request.obst2 = msg2->longitude;
  a.request.obst3 = msg2->altitude;
	  
  lla2enu::Distance s;
	
  float par1;
  float par2;
float startLat;
float startLong;
float startAlt;
	
	
  
  if (client.call(a))
  {
	n.getParam("/zeroLatitude", startLat);  
	n.getParam("/zeroLongitude", startLong);
	n.getParam("/zeroAltitude", startAlt);
	  
    n.getParam("/threshold1", par1);
    n.getParam("/threshold2", par2);
	
	  std::stringstream s1;
		s1 << "Crash";
	  
	 std::stringstream s2;
		s2 << "Unsafe";
	  
	std::stringstream s3;
		s3 << "Safe";  
    
    // RECEIVING THE DISTANCE
    lla2enu::distance dis= a.response.dist;
	  
    //STATUS ELABORATION
    
    if (s.dis> par2)
    {
	s.stato.data = s1.str();
    }else if (s.dis> par1 && s.dis< par2)
    {
	s.stato.data = s2.str();
    }else if (s.dis< par1)
    {
	s.stato.data = s3.str();
    }
	  
    // PUBLISHING TF CAR 
    
    //ros::NodeHandle n; 
	  
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg1->latitude, msg1->longitude, msg1->altitude));
    tf::Quaternion q;
    q.setRPY(0, 0, 0); 
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "cartf"));
	  
    tf::TransformBroadcaster br2;
    tf::Transform transform2;
    transform2.setOrigin(tf::Vector3(msg2->latitude, msg2->longitude, msg2->altitude));
    tf::Quaternion q2;
    q2.setRPY(0, 0, 0); 
    transform2.setRotation(q2);
    br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "map", "obstacletf"));
	  
  //PUBLISHING ODOMETRY MESSAGE CAR
	  ros::Publisher odom_pubCar = n.advertise<nav_msgs::Odometry>("odomCar", 50);
	  
	nav_msgs::Odometry odomCar;
	odomCar.header.frame_id = "odomCar";
	odomCar.child_frame_id = "base_link";
	  
	  odomCar.pose.pose.position.x = msg1->latitude;
      odomCar.pose.pose.position.y = msg1->longitude;
       odomCar.pose.pose.position.z = msg1->altitude;
       odomCar.pose.pose.orientation = q;
	  
	  odom_pub.publish(odomCar);
	  
//PUBLISHING ODOMETRY MESSAGE OBSTACLE
	   ros::Publisher odom_pubObst = n.advertise<nav_msgs::Odometry>("odomObst", 50);
	  
	nav_msgs::Odometry odomCar;
	odomObst.header.frame_id = "odomObst";
	odomObst.child_frame_id = "base_link";
	  
	  odomObst.pose.pose.position.x = msg1->latitude;
      odomObst.pose.pose.position.y = msg1->longitude;
       odomObst.pose.pose.position.z = msg1->altitude;
       odomObst.pose.pose.orientation = q;
	  
	  odom_pubObst.publish(odomObst);


    // PUBLISHING THE CUSTOM MESSAGE
    distance_pub.publish(s);
    loop_rate.sleep();
  }
  else
  {
    ROS_ERROR("Failed to call service compute_distance");
    //return 1;
  }

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
	message_filters::Subscriber<sensor_msgs::NavSatFix> sub1(n, "car", 1);
	//RECEIVE THE OBSTACLE POSITION
	message_filters::Subscriber<sensor_msgs::NavSatFix> sub2(n, "obstacle", 1);
	
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> MySyncPolicy;
	
	
	//senza policy: message_filters::TimeSynchronizer<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> sync(sub1, sub2, 10);
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
	
	
	//BINDING THE TWO VALUES OF THE DIFFERENT TOPICS
	sync.registerCallback(boost::bind(&callback, _1, _2));
 	ros::spin();
 	return 0;

  return 0;
}


