#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudTransformer
{
public:
    PointCloudTransformer() : tf_listener(tf_buffer)
    {
        // 在构造函数中设置 tf_buffer 使用独立线程
        tf_buffer.setUsingDedicatedThread(true);

        ros::NodeHandle nh;
        ros::NodeHandle priv_nh("~");

        // load params
        priv_nh.param<std::string>("target_frame", target_frame, "target_frame");

        pub = priv_nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);
        sub = priv_nh.subscribe("cloud_in", 1, &PointCloudTransformer::cloudCallback, this);
    }

private:
    ros::Subscriber sub;
    ros::Publisher pub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    std::string target_frame;

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer.lookupTransform(target_frame, cloud_msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        sensor_msgs::PointCloud2 transformed_cloud;
        tf2::doTransform(*cloud_msg, transformed_cloud, transform_stamped);
        transformed_cloud.header.frame_id = target_frame;
        pub.publish(transformed_cloud);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_point_cloud");
    PointCloudTransformer transformer;
    ros::spin();
    return 0;
}