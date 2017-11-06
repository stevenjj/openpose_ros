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

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

#include <vector>
#include <cmath>

#include <openpose_ros_msgs/BodypartDetection_3d.h>
#include <openpose_ros_msgs/PersonDetection_3d.h>


// Set resolution

#define width  960
#define height 540



// Declare Publishers
ros::Publisher           pc_pub;
ros::Publisher           image_pub;   


// Declare 3d keypoints publisher
ros::Publisher	         keypoints_3d_pub;


// Function to initialize all skeletal detections

openpose_ros_msgs::BodypartDetection_3d getNANBodypart()
{
  openpose_ros_msgs::BodypartDetection_3d bodypart_depth;
  bodypart_depth.x = NAN;
  bodypart_depth.y = NAN;
  bodypart_depth.z = NAN;
  bodypart_depth.confidence = NAN;
  return bodypart_depth;
}


// Function for when a bodypart is not detected
void notDetectedBodyPart(std::string bodypart_name)
{
  std::cerr << bodypart_name << " not detected!!" << std::endl;

  std::cerr << bodypart_name << " pixel coordinates (x,y,z): " << std::endl;
  std::cerr << "( "<< "nan" << ", " 
           	   << "nan" << ", "
                   << "nan" << ")" << std::endl;

  std::cerr << bodypart_name << " real world coordinates (x,y,z): " << std::endl;
  std::cerr << "( " << "nan" << ", " 
           	    << "nan" << ", "
                    << "nan" << ")" << std::endl;

}


// Function to make the average of a vector

double Average(std::vector<double> v)
{
double total = 0.0;
double size  = 0.0;
for (int n = 0; n < v.size(); n++){total += v[n];size++;}

return total/size;

}


// Function to get 3d detections

openpose_ros_msgs::BodypartDetection_3d get3dcoordinates(const openpose_ros_msgs::BodypartDetection bodypart_2d , const pcl::PointCloud<pcl::PointXYZ>::Ptr  temp_cloud, const double maxx, const double minx, const double maxy, const double miny, const double maxz, const double minz, const std::string bodypart_name)
{
  openpose_ros_msgs::BodypartDetection_3d bodypart_depth;

  // Include 2d confidence 3d detections message
  bodypart_depth.confidence = bodypart_2d.confidence;

  // If not detected bodypart
  if (std::isnan(bodypart_depth.confidence) || bodypart_depth.confidence == 0.0 || (bodypart_2d.x == 0 && bodypart_depth.y == 0) || bodypart_2d.x > width || bodypart_2d.y > height )
  {
  notDetectedBodyPart(bodypart_name);
  bodypart_depth.x = NAN;
  bodypart_depth.y = NAN;
  bodypart_depth.z = NAN;
  }

  // If detected bodypart
  else
  {

  // Get keypoint pixel coordinates
  unsigned long long int x_pixel = bodypart_2d.x;
  unsigned long long int y_pixel = bodypart_2d.y;

  // Vector for storing the keypoint index and the surrounding indices ones
  std::vector<unsigned long long int> indices;
  int index=0;

  // Number of colums and rows of indices surrounding keypoint to get (both must be even)
  int rows = 3;
  int columns = 3;

  // Store in the vector the indices surrounding the keypoint
  for (int i = -(rows-1)/2 ; i <= (rows-1)/2; i++)
   {
    for (int j = -(columns-1)/2; j <= (columns-1)/2; j++)
    {
    index = width*(y_pixel+i)+x_pixel+j+1;
    indices.push_back(index);
    }
   }
  
  // Vector for storing possible world coordinates of indices in the cluster
  std::vector<double> possible_x;
  std::vector<double> possible_y;
  std::vector<double> possible_z;
  
  // Get coordinates if are valid
  for(int n=0; n < indices.size(); n++)
   {
   if (not std::isnan(temp_cloud->points[indices[n]].x) && not std::isnan(temp_cloud->points[indices[n]].y) && not std::isnan(temp_cloud->points[indices[n]].z))
     {
      if (temp_cloud->points[indices[n]].x <= maxx && temp_cloud->points[indices[n]].x >= minx)
        {  
	  if (temp_cloud->points[indices[n]].y <= maxy && temp_cloud->points[indices[n]].y >= miny)
	   {
	    if (temp_cloud->points[indices[n]].z <= maxz && temp_cloud->points[indices[n]].z >= minz)
	    {
	    possible_x.push_back(temp_cloud->points[indices[n]].x);
  	    possible_y.push_back(temp_cloud->points[indices[n]].y);
  	    possible_z.push_back(temp_cloud->points[indices[n]].z);
	    }
	   }
         }

      }
    }

  	// Check if vectors are empty
  	if (possible_x.size() == 0 || possible_y.size() == 0 || possible_z.size() == 0)
  	{
  	notDetectedBodyPart(bodypart_name);
  	bodypart_depth.x = NAN;
  	bodypart_depth.y = NAN;
  	bodypart_depth.z = NAN;
  	}
	else
	{
  	// Make the mean for each coordinate
        bodypart_depth.x = Average(possible_x);
        bodypart_depth.y = Average(possible_y);
        bodypart_depth.z = Average(possible_z);


        std::cerr << bodypart_name << " pixel coordinates (x,y,z): " << std::endl;
        std::cerr << "( "<< bodypart_2d.x << ", " 
           	         << bodypart_2d.y << ", "
                         << bodypart_depth.z << ")" << std::endl;

        std::cerr << bodypart_name << " real world coordinates (x,y,z): " << std::endl;
        std::cerr << "( " << bodypart_depth.x << ", " 
           	          << bodypart_depth.y << ", "
                          << bodypart_depth.z << ")" << std::endl;


	}

  }

return bodypart_depth;

}
// Declare Service Client
ros::ServiceClient client;
openpose_ros_msgs::GetPersons srv;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

//Declare Callback
void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::ImageConstPtr& image_msg){
    ROS_INFO("Cloud and Image Messages Received!");
    ROS_INFO("    Cloud Time Stamp: %f", cloud_msg->header.stamp.toSec());
    ROS_INFO("    Image Time Stamp: %f", image_msg->header.stamp.toSec()); 


    // Publish input pointcloud and image
    pc_pub.publish(*cloud_msg);
    image_pub.publish(*image_msg); 
 
    srv.request.image = *image_msg;
      if (client.call(srv))
      {
        ROS_INFO("ROS Service call Successful");
        // Prepare a new ROS Message for all skeletal detections


	// Initialize message for skeletal detections
        openpose_ros_msgs::PersonDetection_3d person_msg;

        // Number of people detected
        int num_people = srv.response.detections.size();

        // Number of bodyparts, we suppose we are working with COCO
        int num_bodyparts = 18;

        // for each detection (person),

	for (size_t person_idx = 0; person_idx < num_people; person_idx++)
  	{

          // Prepare a new ROS Message for this skeletal detection

	   // Add number of people detected
           person_msg.num_people_detected = num_people;

	   // Add person ID
	   person_msg.person_ID = person_idx;	

           // Initialize all bodyparts (x,y,z=0 & confidence = nan)

    	   person_msg.nose = getNANBodypart();
           person_msg.neck = getNANBodypart();
    	   person_msg.right_shoulder = getNANBodypart();
    	   person_msg.right_elbow = getNANBodypart();
           person_msg.right_wrist = getNANBodypart();
           person_msg.left_shoulder = getNANBodypart();
           person_msg.left_elbow = getNANBodypart();
           person_msg.left_wrist = getNANBodypart();
           person_msg.right_hip = getNANBodypart();
           person_msg.right_knee = getNANBodypart();
           person_msg.right_ankle = getNANBodypart();
           person_msg.left_hip = getNANBodypart();
           person_msg.left_knee = getNANBodypart();
           person_msg.left_ankle = getNANBodypart();
           person_msg.right_eye = getNANBodypart();
           person_msg.left_eye = getNANBodypart();
           person_msg.right_ear = getNANBodypart();
           person_msg.left_ear = getNANBodypart();
           person_msg.chest = getNANBodypart();


	   // Declare pcl<xyz> pointcloud
  	   pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	   //Load pointcloud data from cloud_msg (sensor_msgs::Pointcloud2) to temp_cloud of type pcl::Pointcloud<pcl::PointXYZ>
  		pcl::fromROSMsg (*cloud_msg, *temp_cloud);

		// Get max and min coordinates
     
     		// Declare pcl points
     		pcl::PointXYZ min;
     		pcl::PointXYZ max;

     		// Get min and max depth coordinate
     		pcl::getMinMax3D<pcl::PointXYZ>(*temp_cloud,min,max);

		// Get minimun x,y,z in the screen
		std::cerr << "Minimum pointcloud (x,y,z) coordinates: " << std::endl;
    		std::cerr << "    (" << min.x << ", " 
                            	     << min.y << ", " 
                                     << min.z << " )"<< std::endl;

		// Get maximum x,y,z in the screen
		std::cerr << "Maximum pointcloud (x,y,z) coordinates: " << std::endl;
    		std::cerr << "    (" << max.x << ", " 
                            	     << max.y << ", " 
                            	     << max.z << " )"<< std::endl;

          // for each body part 

	   for (size_t bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++)
           {
		// Get corresponding 2d skeleton detection for person with ID idx
		openpose_ros_msgs::PersonDetection skeleton_detections = srv.response.detections[person_idx];

		// Initialize bodypart msg 
		openpose_ros_msgs::BodypartDetection bodypart_detections;

		if (bodypart_idx == 0)
	   	{
	     		bodypart_detections = skeleton_detections.nose;
			person_msg.nose = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Nose");
	    	}
	   	else if (bodypart_idx == 1)
	    	{
	     		bodypart_detections = skeleton_detections.neck;
			person_msg.neck = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Neck");
	 	}
	   	else if (bodypart_idx == 2)
	    	{
			bodypart_detections = skeleton_detections.right_shoulder;
			person_msg.right_shoulder = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Right shoulder");
		}
	   	else if (bodypart_idx == 3)
	    	{
			bodypart_detections = skeleton_detections.right_elbow;
			person_msg.right_elbow = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Right elbow");
		}
	   	else if (bodypart_idx == 4)
	    	{
			bodypart_detections = skeleton_detections.right_wrist;
			person_msg.right_wrist = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Right wrist");
		}
	   	else if (bodypart_idx == 5)
	    	{
			bodypart_detections = skeleton_detections.left_shoulder;
			person_msg.left_shoulder = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Left shoulder");
		}
	   	else if (bodypart_idx == 6)
	    	{
			bodypart_detections = skeleton_detections.left_elbow;
			person_msg.left_elbow = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Left elbow");
		}
	   	else if (bodypart_idx == 7)
	    	{
			bodypart_detections = skeleton_detections.left_wrist;
			person_msg.left_wrist = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Left wrist");
		}
	   	else if (bodypart_idx == 8)
	    	{
			bodypart_detections = skeleton_detections.right_hip;
			person_msg.right_hip = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Right hip");
		}
	   	else if (bodypart_idx == 9)
	    	{
			bodypart_detections = skeleton_detections.right_knee;
			person_msg.right_knee = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Right knee");
		}
	   	else if (bodypart_idx == 10)
	    	{
			bodypart_detections = skeleton_detections.right_ankle;
			person_msg.right_ankle = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Right ankle");
		}
	   	else if (bodypart_idx == 11)
	    	{
			bodypart_detections = skeleton_detections.left_hip;
			person_msg.left_hip = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Left hip");
		}
	   	else if (bodypart_idx == 12)
	    	{
			bodypart_detections = skeleton_detections.left_knee;
			person_msg.left_knee = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Left knee");
		}
	   	else if (bodypart_idx == 13)
	    	{
			bodypart_detections = skeleton_detections.left_ankle;
			person_msg.left_ankle = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Left ankle");
		}
	   	else if (bodypart_idx == 14)
	    	{
			bodypart_detections = skeleton_detections.right_eye;
			person_msg.right_eye = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Right eye");
		}
	   	else if (bodypart_idx == 15)
	    	{
			bodypart_detections = skeleton_detections.left_eye;
			person_msg.left_eye = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Left eye");
		}
	   	else if (bodypart_idx == 16)
	    	{
			bodypart_detections = skeleton_detections.right_ear;
			person_msg.right_ear = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Right ear");
		}
	   	else if (bodypart_idx == 17)
	    	{
			bodypart_detections = skeleton_detections.left_ear;
			person_msg.left_ear = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Left ear");
		}
/*	   	else if (bodypart_idx == 18)
	    	{
			bodypart_detections = skeleton_detections.chest;
			person_msg.chest = get3dcoordinates(bodypart_detections , temp_cloud, max.x, min.x, max.y, min.y, max.z , min.z, "Chest");
		}
*/	
	   }

	   // Publish 3D detection
	   keypoints_3d_pub.publish(person_msg);

        }
      }
      else
      {
        ROS_ERROR("Failed to call service detect_poses_3d");
        ROS_ERROR("Did you remap the service and client names?");
        ROS_ERROR("This node expects a service called skeleton_2d_detector in which a launch file should have handled the remapping");
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


    // Pointcloud publisher topic /openpose_ros/input_pointcloud
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>( "/openpose_ros/skeleton_3d/input_pointcloud", 0);

    // Image publisher topic /openpose_ros/input_rgb
    image_pub = nh.advertise<sensor_msgs::Image>( "/openpose_ros/skeleton_3d/input_rgb", 0);

    // Keypoints in 3D topic /openpose_ros/detected_poses_keypoints_3d
    keypoints_3d_pub = nh.advertise<openpose_ros_msgs::PersonDetection_3d>( "/openpose_ros/skeleton_3d/detected_poses_keypoints_3d", 0);


    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Spin Forever
    ros::spin();

    return 0;
}
    
