// This code synchronizes the 3d point cloud and 2D image and publishes 3d locations of human skeletons
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <openpose_ros_msgs/GetPersons.h>
// Declare Publishers
ros::Publisher           pc_pub;
ros::Publisher           image_pub;   

// Declare Service Client
ros::ServiceClient client;
openpose_ros_msgs::GetPersons srv;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

//Declare Callback
void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::ImageConstPtr& image_msg){
    ROS_INFO("Cloud and Image Messages Received!");
    ROS_INFO("    Cloud Time Stamp: %f", cloud_msg->header.stamp.toSec());
    ROS_INFO("    Image Time Stamp: %f", image_msg->header.stamp.toSec());  

    srv.request.image = *image_msg;
      if (client.call(srv))
      {
        ROS_INFO("ROS Service call Successful");
        // Prepare a new ROS Message for all skeletal detections
        // Prepare a new point cloud to publish

        // for each detection,
          // Prepare a new ROS Message for this skeletal detection
          // Prepare a new color to use

          // for each body part 
              // grab a small area of pixels surrounding the x,y image coordinate

                  //for each pixel in that area, check if it is a valid point: (the point exists in the organized point cloud and that its z-depth is less than Z_MAX)
                  //      mean-shift valid 3D points and store largest cluster

                  //      

//        ROS_INFO("Hello!: %ld", (long int)srv.response.sum);
      }
      else
      {
        ROS_ERROR("Failed to call service detect_poses");
      }

}

int main(int argc, char** argv){
    // Initialize ROS
    ros::init(argc, argv, "skeleton_extract_3d_node");

    // Declare Node Handle
    ros::NodeHandle nh("~");

    // Declare Subscribers
    // Synchronize Point Cloud and Image Subscription Received Messages
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "point_cloud", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "image", 1);
    client = nh.serviceClient<openpose_ros_msgs::GetPersons>("skeleton_2d_detector");

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Spin Forever
    ros::spin();

    return 0;
}
    