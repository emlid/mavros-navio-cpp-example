# Mavros Navio C++ example

## **Overview**

In this tutorial we will create a simple subscriber that obtains data from mavros node and prints it. Though mavros publishes a lot of different topics, we will examine /mavros/imu/data as for example. Handling another one will be slightly different, but general principles are the same. 

We will consider the following steps:

- Creating a workspace
- Creating a package
- Writing a simple subscriber
- Running a node
## **Creating a catkin workspace**

If you aren’t familiar with catkin, you can look through [this page](http://wiki.ros.org/catkin/conceptual_overview). Let's create and build a catkin workspace:

    $ mkdir -p ~/imu_example/src
    $ cd ~/imu_example/
    $ catkin_make

This will produce CMakeLists.txt and many other important files in your home directory that we will inspect later.

Source your new setup file to overlay this workspace on top of your environment:

    $ source devel/setup.bash
## **Creating a package**

First of all change to the source space directory:

    $ cd ~/imu_example/src

Now use the **catkin_create_pkg** script to create a new package called 'imu_data' which depends on mavros, mavros_msgs and roscpp:

    $ catkin_create_pkg imu_data mavros sensor_msgs roscpp

Next you need to build the packages in the catkin workspace:

    $ cd ~/imu_example
    $ catkin_make

To add the workspace to your ROS environment you need to source the generated setup file:

    $ . ~/imu_example/devel/setup.bash


## **Writing a simple subscriber**

Create the imu_data.cpp inside the src/ directoty in your package and paste there the following code:

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

**The code explained:**

    #include "ros/ros.h"

This will include all the headers necessary to use the most common public pieces of the ROS system.


    #include "sensor_msgs/Imu.h"

This includes the sensor_msgs/Imu message, which resides in the sensor_msgs package. You can find another message types in **/opt/ros/indigo/include**. (e.g., std_msgs, shape_msgs, etc.)


    ros::init(argc, argv, "imu_data");

Initialize ROS. This allows ROS to do name remapping through the command line


    ros::Subscriber sub = n.subscribe("/mavros/imu/data", 1000, chatterCallback);

Subscribe to the /mavros/imu/data topic with the master. ROS will call the chatterCallback() function whenever a new message arrives. The 2nd argument is the queue size, in case we are not able to process messages fast enough. In this case, if the queue reaches 1000 messages, we will start throwing away old messages as new ones arrive.


    ros::spin();

Enter a loop, calling message callbacks as fast as possible.

**Building your node:**

Open CMakeLists.txt inside your package and make sure it looks like the following:

    cmake_minimum_required(VERSION 2.8.3)
    project(imu_data)                                                           
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        mavros
        sensor_msgs
    )
    catkin_package()
    include_directories(
        ${catkin_INCLUDE_DIRS}                                                           
    )
    include_directories(include ${catkin_INCLUDE_DIRS})
    add_executable(imu_data src/imu_data.cpp)
    target_link_libraries(imu_data ${catkin_LIBRARIES}) 

Now run catkin_make to build your subscriber:

    $ cd ~/imu_example
    $ catkin_make


## **Running your node**

Make sure that you understand what’s going on in [this tutorial.](https://docs.emlid.com/navio2/common/dev/ros/) To run your subscriber you need to have both roscore and mavros_node running. Enter the following:

    $ rosrun imu_data imu_data

If you did everything correctly, you will see the messages like this:

    [ INFO] [1500984138.478721632]:
    linear acceleration            
    x: [0.245166]
    y:[-0.313813]
    z:[9.865490]

