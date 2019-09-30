#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
/*
ros::Publisher pub1;
ros::Publisher pub2;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tf2_listener;
*/
class Listener
{
    public:
        sensor_msgs::PointCloud2 input_cloud;
        geometry_msgs::QuaternionStamped* input_quaternion;
        geometry_msgs::TransformStamped transform_stamped;
        sensor_msgs::PointCloud2 transformed_cloud;
        //std::shared_ptr<sensor_msgs::PointCloud2> PCPC{new sensor_msgs::PointCloud2};
        void callbackPC(const sensor_msgs::PointCloud2::ConstPtr& msg1);
        //void callbackTF(const geometry_msgs::QuaternionStamped::ConstPtr& msg2);
        void transform();
};

void Listener::callbackPC(const sensor_msgs::PointCloud2::ConstPtr& msg1)
{
    ROS_INFO("pointcloud_kita");
    input_cloud = *msg1;

}
/*
void Listener::callbackTF(const geometry_msgs::QuaternionStamped::ConstPtr& msg2)
{
    ROS_INFO("QUaternion_kita");
}
*/
void Listener::transform()
{

    ROS_INFO("transform");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    transform_stamped = tfBuffer.lookupTransform("odom","imu",ros::Time(0),ros::Duration(2.0));
    

    Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();

    pcl_ros::transformPointCloud(mat,input_cloud,transformed_cloud);
    
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"pointcloud_tf_trans");

    ros::NodeHandle nh;
    Listener listener;
    ros::Rate loop_rate(10);
    ros::Subscriber sub1 = nh.subscribe("/camera/depth_registered/points",5,&Listener::callbackPC,&listener);
    ros::Subscriber sub2 = nh.subscribe("quaternion",5,&Listener::callbackTF,&listener);
    ros::Publisher Pub1 = nh.advertise<sensor_msgs::PointCloud2>("aaaaa",1);
    while(nh.ok()){
        
        listener.transform();
        Pub1.publish(listener.transformed_cloud);
        ros::spinOnce();
        loop_rate.sleep();

        
    }
    ros::spin();
    return 0;
}


