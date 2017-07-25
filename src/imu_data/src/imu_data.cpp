#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
 
void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg){
    ROS_INFO("\nlinear acceleration\
                \nx: [%f]\ny:[%f]\nz:[%f]", msg->linear_acceleration.x,
                msg->linear_acceleration.y, msg->linear_acceleration.z);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_data");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/mavros/imu/data", 1000, chatterCallback);
    ros::spin();
    return 0;
}
