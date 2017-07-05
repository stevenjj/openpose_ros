//#define USE_CAFFE
/*#include <openpose/pose/poseExtractor.hpp>
#include <openpose/pose/poseExtractorCaffe.hpp>
#include <openpose/pose/poseParameters.hpp>

#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>
*/
#include <openpose/headers.hpp>

#include <std_srvs/Empty.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <ros/init.h>
#include <cv_bridge/cv_bridge.h>

#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <openpose_ros_msgs/GetPersons.h>

std::shared_ptr<op::PoseExtractor> g_pose_extractor;
std::shared_ptr<op::PoseRenderer> poseRenderer;
std::map<unsigned int, std::string> g_bodypart_map;
op::Point<int> g_net_input_size;
op::Point<int> output_size;
int g_num_scales;
double g_scale_gap;

ros::Publisher           image_skeleton_pub;
//!
//! \brief getParam Get parameter from node handle
//! \param nh The nodehandle
//! \param param_name Key string
//! \param default_value Default value if not found
//! \return The parameter value
//!
template <typename T>
T getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value)
{
  T value;
  if (nh.hasParam(param_name))
  {
    nh.getParam(param_name, value);
  }
  else
  {
    ROS_WARN_STREAM("Parameter '" << param_name << "' not found, defaults to '" << default_value << "'");
    value = default_value;
  }
  return value;
}

op::PoseModel stringToPoseModel(const std::string& pose_model_string)
{
  if (pose_model_string == "COCO")
    return op::PoseModel::COCO_18;
  else if (pose_model_string == "MPI")
    return op::PoseModel::MPI_15;
  else if (pose_model_string == "MPI_4_layers")
    return op::PoseModel::MPI_15_4;
  else
  {
    ROS_ERROR("String does not correspond to any model (COCO, MPI, MPI_4_layers)");
    return op::PoseModel::COCO_18;
  }
}

std::map<unsigned int, std::string> getBodyPartMapFromPoseModel(const op::PoseModel& pose_model)
{
  if (pose_model == op::PoseModel::COCO_18)
  {
    return op::POSE_COCO_BODY_PARTS;
  }
  else if (pose_model == op::PoseModel::MPI_15 || pose_model == op::PoseModel::MPI_15_4)
  {
    return op::POSE_MPI_BODY_PARTS;
  }
  else
  {
    ROS_FATAL("Invalid pose model, not map present");
    exit(1);
  }
}

openpose_ros_msgs::BodypartDetection getBodyPartDetectionFromArrayAndIndex(const op::Array<float>& array, size_t idx)
{
  openpose_ros_msgs::BodypartDetection bodypart;
  bodypart.x = array[idx];
  bodypart.y = array[idx+1];
  bodypart.confidence = array[idx+2];
  return bodypart;
}

openpose_ros_msgs::BodypartDetection getNANBodypart()
{
  openpose_ros_msgs::BodypartDetection bodypart;
  bodypart.confidence = NAN;
  return bodypart;
}

bool detectPosesCallback(openpose_ros_msgs::GetPersons::Request& req, openpose_ros_msgs::GetPersons::Response& res)
{
  ROS_INFO("detectPosesCallback");

  // Convert ROS message to opencv image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(req.image, req.image.encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("detectPosesCallback cv_bridge exception: %s", e.what());
    return false;
  }

  cv::Mat image = cv_ptr->image;

//  std::string path = ros::package::getPath("openpose_ros_pkg");
//  std::string filename = path + "/examples/COCO_val2014_000000000192.jpg";
//  cv::Mat image = op::loadImage(filename, CV_LOAD_IMAGE_COLOR);

  ROS_INFO("Parsed image");
  ROS_INFO_STREAM("Perform forward pass with the following settings:");
  ROS_INFO_STREAM("- net_input_size: " << g_net_input_size.x << " " << g_net_input_size.y);
  ROS_INFO_STREAM("- num_scales: " << g_num_scales);
  ROS_INFO_STREAM("- scale_gap: " << g_scale_gap);
  ROS_INFO_STREAM("- image_size: " << image.size());
  op::CvMatToOpInput cv_mat_to_op_input(g_net_input_size, g_num_scales, g_scale_gap);
//  g_pose_extractor->forwardPass(cv_mat_to_op_input.format(image), image.size());

  ROS_INFO("Initialized Net Size");

  op::Array<float> netInputArray;
  std::vector<float> scaleRatios;
  std::tie(netInputArray, scaleRatios) = cv_mat_to_op_input.format(image);
  ROS_INFO("Preparing for forward pass");
  g_pose_extractor->forwardPass(netInputArray,  {image.cols, image.rows}, scaleRatios);

  ROS_INFO("g_pose_extractor->forwardPass done");




  //const op::Array<float> poses;
  const op::Array<float> poses = g_pose_extractor->getPoseKeypoints();  

  // VISUALIZE OUTPUT
  //const auto poseKeypoints = g_pose_extractor->getPoseKeypoints();

  op::Point<int> outputSize(output_size.x, output_size.y);
  op::CvMatToOpOutput cvMatToOpOutput{outputSize};
  op::OpOutputToCvMat opOutputToCvMat{outputSize};

  const op::Point<int> windowedSize = outputSize;
  op::FrameDisplayer frameDisplayer{windowedSize, "OpenPose Example"};

  double scaleInputToOutput;
  op::Array<float> outputArray;
  std::tie(scaleInputToOutput, outputArray) = cvMatToOpOutput.format(image);

  //poseRenderer->renderPose(outputArray, poseKeypoints);
  poseRenderer->renderPose(outputArray, poses);
  auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);


  sensor_msgs::Image ros_image;
  cv_bridge::CvImagePtr cv_ptr_out = cv_bridge::toCvCopy(req.image, req.image.encoding);
  cv_ptr_out->image = outputImage;
  ros_image = *(cv_ptr_out->toImageMsg());


  //frameDisplayer.displayFrame(outputImage, 0); // Alternative: cv::imshow(outputImage) + cv::waitKey(0)
  image_skeleton_pub.publish(ros_image);

  // End Visualize Output


  if (!poses.empty() && poses.getNumberDimensions() != 3)
  {
    ROS_ERROR("pose.getNumberDimensions(): %d != 3", (int) poses.getNumberDimensions());
    return false;
  }

  int num_people = poses.getSize(0);
  int num_bodyparts = poses.getSize(1);

  ROS_INFO("num people: %d", num_people);

  for (size_t person_idx = 0; person_idx < num_people; person_idx++)
  {
    ROS_INFO("    Person ID: %zu", person_idx);
    // Initialize all bodyparts with nan
    openpose_ros_msgs::PersonDetection person_msg;
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

    for (size_t bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++)
    {
      size_t final_idx = 3*(person_idx*num_bodyparts + bodypart_idx);

      std::string body_part_string = g_bodypart_map[bodypart_idx];
      openpose_ros_msgs::BodypartDetection bodypart_detection = getBodyPartDetectionFromArrayAndIndex(poses, final_idx);

      if (body_part_string == "Nose") person_msg.nose = bodypart_detection;
      else if (body_part_string == "Neck") person_msg.neck = bodypart_detection;
      else if (body_part_string == "RShoulder") person_msg.right_shoulder = bodypart_detection;
      else if (body_part_string == "RElbow") person_msg.right_elbow = bodypart_detection;
      else if (body_part_string == "RWrist") person_msg.right_wrist = bodypart_detection;
      else if (body_part_string == "LShoulder") person_msg.left_shoulder = bodypart_detection;
      else if (body_part_string == "LElbow") person_msg.left_elbow = bodypart_detection;
      else if (body_part_string == "LWrist") person_msg.left_wrist = bodypart_detection;
      else if (body_part_string == "RHip") person_msg.right_hip = bodypart_detection;
      else if (body_part_string == "RKnee") person_msg.right_knee = bodypart_detection;
      else if (body_part_string == "RAnkle") person_msg.right_ankle = bodypart_detection;
      else if (body_part_string == "LHip") person_msg.left_hip = bodypart_detection;
      else if (body_part_string == "LKnee") person_msg.left_knee = bodypart_detection;
      else if (body_part_string == "LAnkle") person_msg.left_ankle = bodypart_detection;
      else if (body_part_string == "REye") person_msg.right_eye = bodypart_detection;
      else if (body_part_string == "LEye") person_msg.left_eye = bodypart_detection;
      else if (body_part_string == "REar") person_msg.right_ear = bodypart_detection;
      else if (body_part_string == "LEar") person_msg.left_ear = bodypart_detection;
      else if (body_part_string == "Chest") person_msg.chest = bodypart_detection;
      else
      {
        ROS_ERROR("Unknown bodypart %s, this should never happen!", body_part_string.c_str());
      }

      ROS_INFO("        body part: %s", body_part_string.c_str());
      ROS_INFO("            (x, y, confidence): %i, %i, %f", bodypart_detection.x, bodypart_detection.y, bodypart_detection.confidence);

    }
    res.detections.push_back(person_msg);
  }

  ROS_INFO("Detected %d persons", (int) res.detections.size());

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "openpose_ros_service_node");  

  ros::NodeHandle local_nh("~");
//  g_net_input_size = op::Point(getParam(local_nh, "net_input_width", 656), getParam(local_nh, "net_input_height", 368));

  g_net_input_size.x = getParam(local_nh, "net_input_width", 656);
  g_net_input_size.y = getParam(local_nh, "net_input_height", 368);

  op::Point<int> net_output_size(getParam(local_nh, "net_output_width", 656), getParam(local_nh, "net_output_height", 368));
  //op::Point<int> output_size(getParam(local_nh, "output_width", 1280), getParam(local_nh, "output_height", 720));
  //op::Point<int> output_size(getParam(local_nh, "output_width", 1024), getParam(local_nh, "output_height", 1024));  

  output_size.x = getParam(local_nh, "output_width", 1024);
  output_size.y = getParam(local_nh, "output_height", 1024);

  g_num_scales = getParam(local_nh, "num_scales", 1);
  g_scale_gap = getParam(local_nh, "scale_gap", 0.3);
  unsigned int num_gpu_start = getParam(local_nh, "num_gpu_start", 0);



  std::string package_path = ros::package::getPath("openpose_ros_pkg");
  std::string folder_location = package_path + "/../openpose/models/";

  std::string model_folder = getParam(local_nh, "model_folder", folder_location);
  op::PoseModel pose_model = stringToPoseModel(getParam(local_nh, "pose_model", std::string("COCO")));
  g_bodypart_map = getBodyPartMapFromPoseModel(pose_model);

  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("detect_poses", detectPosesCallback);


  image_skeleton_pub = nh.advertise<sensor_msgs::Image>( "/openpose_ros/detected_poses_image", 0 );  

  g_pose_extractor = std::shared_ptr<op::PoseExtractorCaffe>(
/*        new op::PoseExtractorCaffe(g_net_input_size, net_output_size, output_size, g_num_scales,
                                   g_scale_gap, pose_model, model_folder, num_gpu_start));*/

        new op::PoseExtractorCaffe(g_net_input_size, net_output_size, output_size, g_num_scales, pose_model, 
                                            model_folder, num_gpu_start));

  poseRenderer = std::shared_ptr<op::PoseRenderer>(
      new op::PoseRenderer(net_output_size, output_size, pose_model, nullptr, true, 0.6));
      //poseRenderer{netOutputSize, outputSize, poseModel, nullptr, !FLAGS_disable_blending, (float)FLAGS_alpha_pose};

  g_pose_extractor->initializationOnThread();
  poseRenderer->initializationOnThread();

  ROS_INFO("Initialization Successful!");
  ros::spin();

  return 0;
}


