#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <math.h>   

#include <openpose_ros_msgs/BodypartDetection_3d.h>
#include <openpose_ros_msgs/PersonDetection_3d.h>

// Declare marker and skeleton publishers

ros::Publisher   marker_pub;
ros::Publisher   skeleton_pub;

// Function to check if 3d detection is not NAN
bool PointISValid(const openpose_ros_msgs::BodypartDetection_3d bodypart){
  if (std::isnan(bodypart.x) || std::isnan(bodypart.y) || std::isnan(bodypart.z)){return false;}
  else{return true;}
}

// Function to add joint coordinates to each point marker
geometry_msgs::Point AddPoint(const openpose_ros_msgs::BodypartDetection_3d bodypart){

  geometry_msgs::Point p;
  p.x = bodypart.x;
  p.y = bodypart.y;
  p.z = bodypart.z;
   
  return p;
}


// Declare class to subscribe to 3d detections and publish visualization markers
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topics you want to publish
    marker_pub = n_.advertise<visualization_msgs::Marker>("/openpose_ros/skeleton_3d/visualization_markers", 1);
    skeleton_pub = n_.advertise<visualization_msgs::Marker>("/openpose_ros/skeleton_3d/visualization_skeleton", 1);

    //Topics you want to subscribe
    sub_ = n_.subscribe("/openpose_ros/skeleton_3d/detected_poses_keypoints_3d", 1, &SubscribeAndPublish::callback, this);
  }

  // Declare callback for subscriber
  void callback(const openpose_ros_msgs::PersonDetection_3d& person_msg)
  {

    ROS_INFO("3D Detection Received!");

	
    // Declare marker for the body joints
    visualization_msgs::Marker marker;

    // Set boyjoints markers
    marker.header.frame_id = "/kinect2_ir_optical_frame";
    marker.id = person_msg.person_ID;
    marker.ns = "joints";
    marker.header.stamp = ros::Time();
    // Markers will be spheres
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    // Joints are red
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // Set marker duration in 150ms
    marker.lifetime = ros::Duration(0.15);

   // Declare line strip marker for the skeleton
   visualization_msgs::Marker skeleton;

   skeleton.id  = person_msg.person_ID;
   skeleton.header.frame_id = "/kinect2_ir_optical_frame";
   skeleton.ns = "skeleton";
   skeleton.header.stamp = ros::Time();
   // Skeleton will be lines
   skeleton.type = visualization_msgs::Marker::LINE_LIST;
   skeleton.scale.x = 0.03;
   skeleton.scale.y = 0.03;
   skeleton.scale.z = 0.03;
   // Skeleton is blue
   skeleton.color.a = 1.0;
   skeleton.color.r = 0.0;
   skeleton.color.g = 0.0;
   skeleton.color.b = 1.0;

   // Set skeleton lifetime
   skeleton.lifetime = ros::Duration(0.15);

   // Assign 3D joints coordinates to each marker
   // Check if two consecutive markers exist, if true, draw a line
   for (size_t bodypart_idx = 0; bodypart_idx < 18 ; bodypart_idx++){

	if (bodypart_idx == 0)
	{
		if (PointISValid(person_msg.nose))
        	{marker.points.push_back(AddPoint(person_msg.nose));}
	}
	else if (bodypart_idx == 1)
	{
		if (PointISValid(person_msg.neck)){
        		marker.points.push_back(AddPoint(person_msg.neck));
			// Draw line between neck and nose
			if (PointISValid(person_msg.nose)){
				skeleton.points.push_back(AddPoint(person_msg.nose));
				skeleton.points.push_back(AddPoint(person_msg.neck));
			}
		}

	}
	else if (bodypart_idx == 2)
	{
		if (PointISValid(person_msg.right_shoulder)){
			marker.points.push_back(AddPoint(person_msg.right_shoulder));
			// Draw line between right shoulder and neck
			if (PointISValid(person_msg.neck)){
				skeleton.points.push_back(AddPoint(person_msg.neck));
				skeleton.points.push_back(AddPoint(person_msg.right_shoulder));
			}
		}
	}
	else if (bodypart_idx == 3)
	{
		if (PointISValid(person_msg.right_elbow)){
			marker.points.push_back(AddPoint(person_msg.right_elbow));
			// Draw line between right shoulder and right elbow
			if (PointISValid(person_msg.right_shoulder)){
				skeleton.points.push_back(AddPoint(person_msg.right_shoulder));
				skeleton.points.push_back(AddPoint(person_msg.right_elbow));
			}
		}

	}
	else if (bodypart_idx == 4)
	{
		if (PointISValid(person_msg.right_wrist)){
        	marker.points.push_back(AddPoint(person_msg.right_wrist));
			// Draw line between right elbow and right wrist
			if (PointISValid(person_msg.right_elbow)){
				skeleton.points.push_back(AddPoint(person_msg.right_elbow));
				skeleton.points.push_back(AddPoint(person_msg.right_wrist));
			}
		}
	}
	else if (bodypart_idx == 5)
	{
		if (PointISValid(person_msg.left_shoulder)){
        	marker.points.push_back(AddPoint(person_msg.left_shoulder));
			// Draw line between left shoulder and neck
			if (PointISValid(person_msg.neck)){
				skeleton.points.push_back(AddPoint(person_msg.neck));
				skeleton.points.push_back(AddPoint(person_msg.left_shoulder));
			}
		}

	}
	else if (bodypart_idx == 6)
	{
		if (PointISValid(person_msg.left_elbow)){
        	marker.points.push_back(AddPoint(person_msg.left_elbow));
			// Draw line between left shoulder and left elbow
			if (PointISValid(person_msg.left_shoulder)){
				skeleton.points.push_back(AddPoint(person_msg.left_shoulder));
				skeleton.points.push_back(AddPoint(person_msg.left_elbow));
			}
		}
	}
	else if (bodypart_idx == 7)
	{
		if (PointISValid(person_msg.left_wrist)){
        	marker.points.push_back(AddPoint(person_msg.left_wrist));
			// Draw line between left elbow and left wrist
			if (PointISValid(person_msg.left_elbow)){
				skeleton.points.push_back(AddPoint(person_msg.left_elbow));
				skeleton.points.push_back(AddPoint(person_msg.left_wrist));
			}
		}
	}
	else if (bodypart_idx == 8)
	{
		if (PointISValid(person_msg.right_hip)){
        	marker.points.push_back(AddPoint(person_msg.right_hip));
			// Draw line between right hip and neck
			if (PointISValid(person_msg.neck)){
				skeleton.points.push_back(AddPoint(person_msg.neck));
				skeleton.points.push_back(AddPoint(person_msg.right_hip));
			}
		}
	}
	else if (bodypart_idx == 9)
	{
		if (PointISValid(person_msg.right_knee)){
        	marker.points.push_back(AddPoint(person_msg.right_knee));
			// Draw line between right hip and right knee
			if (PointISValid(person_msg.right_hip)){
				skeleton.points.push_back(AddPoint(person_msg.right_hip));
				skeleton.points.push_back(AddPoint(person_msg.right_knee));
			}
		}
	}
	else if (bodypart_idx == 10)
	{
		if (PointISValid(person_msg.right_ankle)){
        	marker.points.push_back(AddPoint(person_msg.right_ankle));
			// Draw line between right ankle and right knee
			if (PointISValid(person_msg.right_hip)){
				skeleton.points.push_back(AddPoint(person_msg.right_knee));
				skeleton.points.push_back(AddPoint(person_msg.right_ankle));
			}
		}
	}
	else if (bodypart_idx == 11)
	{
		if (PointISValid(person_msg.left_hip)){
        	marker.points.push_back(AddPoint(person_msg.left_hip));
			// Draw line between left hip and neck
			if (PointISValid(person_msg.neck)){
				skeleton.points.push_back(AddPoint(person_msg.neck));
				skeleton.points.push_back(AddPoint(person_msg.left_hip));
			}
		}
	}
	else if (bodypart_idx == 12)
	{
		if (PointISValid(person_msg.left_knee)){
        	marker.points.push_back(AddPoint(person_msg.left_knee));
			// Draw line between left knee and left hip
			if (PointISValid(person_msg.left_hip)){
				skeleton.points.push_back(AddPoint(person_msg.left_hip));
				skeleton.points.push_back(AddPoint(person_msg.left_knee));
			}
		}
	}
	else if (bodypart_idx == 13)
	{
		if (PointISValid(person_msg.left_ankle)){
        	marker.points.push_back(AddPoint(person_msg.left_ankle));
			// Draw line between left knee and left ankle
			if (PointISValid(person_msg.left_knee)){
				skeleton.points.push_back(AddPoint(person_msg.left_knee));
				skeleton.points.push_back(AddPoint(person_msg.left_ankle));
			}
		}
	}
	else if (bodypart_idx == 14)
	{
		if (PointISValid(person_msg.right_eye)){
        	marker.points.push_back(AddPoint(person_msg.right_eye));
			// Draw line between right eye and nose
			if (PointISValid(person_msg.nose)){
				skeleton.points.push_back(AddPoint(person_msg.nose));
				skeleton.points.push_back(AddPoint(person_msg.right_eye));
			}
		}
	}
	else if (bodypart_idx == 15)
	{
		if (PointISValid(person_msg.left_eye)){
        	marker.points.push_back(AddPoint(person_msg.left_eye));
			// Draw line between left eye and nose
			if (PointISValid(person_msg.nose)){
				skeleton.points.push_back(AddPoint(person_msg.nose));
				skeleton.points.push_back(AddPoint(person_msg.left_eye));
			}
		}
	}
	else if (bodypart_idx == 16)
	{
		if (PointISValid(person_msg.right_ear)){
        	marker.points.push_back(AddPoint(person_msg.right_ear));
			// Draw line between right eye and right ear
			if (PointISValid(person_msg.right_eye)){
				skeleton.points.push_back(AddPoint(person_msg.right_eye));
				skeleton.points.push_back(AddPoint(person_msg.right_ear));
			}
		}
	}
	else if (bodypart_idx == 17)
	{
		if (PointISValid(person_msg.left_ear)){
        	marker.points.push_back(AddPoint(person_msg.left_ear));
			// Draw line between left eye and left ear
			if (PointISValid(person_msg.left_eye)){
				skeleton.points.push_back(AddPoint(person_msg.left_eye));
				skeleton.points.push_back(AddPoint(person_msg.left_ear));
			}
		}
	}
/*	else if (bodypart_idx == 18)
	{
		if (PointISValid(person_msg.chest))
        	{marker.points.push_back(AddPoint(person_msg.chest));}
	}
*/
    }
  
  // Publish markers
  marker_pub.publish(marker);
  skeleton_pub.publish(skeleton);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher marker_pub;
  ros::Publisher skeleton_pub;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "skeleton_extract_3d_visualization_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}


