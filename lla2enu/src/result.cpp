#include "ros/ros.h"
#include "std_msgs/String.h"


class pub_sub
{

	std_msgs::String messagio;
	std_msgs::String messagio2;

	private:
	ros::NodeHandle n; 

	ros::Subscriber sub;
	ros::Subscriber sub2;
	ros::Publisher pub; 
	ros::Timer timer1;
	
	
public:
  	pub_sub(){
	  	sub = n.subscribe("/chatter", 1, &pub_sub::callback, this);
		sub2 = n.subscribe("/chatter2", 1, &pub_sub::callback2, this);
		pub = n.advertise<std_msgs::String>("/rechatter", 1);
		timer1 = n.createTimer(ros::Duration(1), &pub_sub::callback1, this);

	}

void callback(const std_msgs::String::ConstPtr& msg){
	messagio=*msg;

}

void callback2(const std_msgs::String::ConstPtr& msg){
	messagio2=*msg;

}

void callback1(const ros::TimerEvent&)
{
	pub.publish(messagio);
	pub.publish(messagio2);
  	ROS_INFO("Callback 1 triggered");
}


};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "subscribe_and_publish");
 	pub_sub my_pub_sub;
 	ros::spin();
 	return 0;
}
