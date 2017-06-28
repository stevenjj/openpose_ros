// This code tests the openpose rosservice node by loading an image and calling the service.#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <openpose_ros_msgs/GetPersons.h>

int main(int argc, char** argv){
    // Initialize ROS
    ros::init(argc, argv, "test_openpose_ros_service_call");
    // Declare Node Handle
    ros::NodeHandle nh;
    
    // Declare Service
    ros::ServiceClient client = nh.serviceClient<openpose_ros_msgs::GetPersons>("detect_poses");
    openpose_ros_msgs::GetPersons srv;

    // Declare Publisher
    ros::Publisher input_image_pub  = nh.advertise<sensor_msgs::Image>( "/openpose_ros/input_image", 0 );  

    // Initialize cv_ptr
    sensor_msgs::Image ros_image;
    ros_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);

    // Read Image
    std::string path = ros::package::getPath("openpose_ros_pkg");
    std::string filename = path + "/examples/COCO_val2014_000000000192.jpg";

    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_COLOR); 
    cv_ptr->image = image;

    // Convert to ros Image msg
    ros_image = *(cv_ptr->toImageMsg());

    // Publish Input Msg
    while (true){
      input_image_pub.publish(ros_image);

      // Begin Service Call
      srv.request.image = ros_image;
        if (client.call(srv))
        {
          ROS_INFO("Call Successful"); //subscribe to /openpose_ros/detected_poses_image to view the result
        }
        else
        {
          ROS_ERROR("Failed to call service detect_poses");
        }

      ros::spinOnce();
    }  

    return 0;
}
    