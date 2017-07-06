//#define USE_CAFFE

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <string> // std::string

#include <opencv2/core/core.hpp> // cv::Mat & cv::Size

// ------------------------- OpenPose Library Tutorial - Pose - Example 1 - Extract from Image -------------------------
// This first example shows the user how to:
    // 1. Load an image (`filestream` module)
    // 2. Extract the pose of that image (`pose` module)
    // 3. Render the pose on a resized copy of the input image (`pose` module)
    // 4. Display the rendered pose (`gui` module)
// In addition to the previous OpenPose modules, we also need to use:
    // 1. `core` module: for the Array<float> class that the `pose` module needs
    // 2. `utilities` module: for the error & logging functions, i.e. op::error & op::log respectively

// 3rdparty dependencies
#include <gflags/gflags.h> // DEFINE_bool, DEFINE_int32, DEFINE_int64, DEFINE_uint64, DEFINE_double, DEFINE_string
#include <glog/logging.h> // google::InitGoogleLogging
// OpenPose dependencies
#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>


//#include <openpose/headers.hpp>

// See all the available parameter options withe the `--help` flag. E.g. `./build/examples/openpose/openpose.bin --help`.
// Note: This command will show you flags for other unnecessary 3rdparty files. Check only the flags for the OpenPose
// executable. E.g. for `openpose.bin`, look for `Flags from examples/openpose/openpose.cpp:`.
// Debugging
DEFINE_int32(logging_level,             4,              "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while"
                                                        " 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for"
                                                        " low priority messages and 4 for important ones.");
// Camera Topic
DEFINE_string(camera_topic,             "/multisense/left/image_rect_color_rotated_180",      "Image topic that OpenPose will process.");
// OpenPose
std::string package_path = ros::package::getPath("openpose_ros_pkg");
std::string model_folder_location = package_path + "/../openpose/models/";

//DEFINE_string(model_folder,             "/home/stevenjj/nstrf_ws/src/openpose_ros/openpose/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");

/*DEFINE_string(net_resolution,           "656x368",      "Multiples of 16. If it is increased, the accuracy usually increases. If it is decreased,"
                                                        " the speed increases.");
// "Multiples of 16. the accuracy usually increases. If it is decreased, the speed increases                                                        */
#define NET_RES_X 656 
#define NET_RES_Y 368  
/*DEFINE_string(resolution,               "1280x720",     "The image resolution (display and output). Use \"-1x-1\" to force the program to use the"
                                                        " default images resolution.");*/
#define OUTPUT_RES_X 1280 // Display Resolution Output Width
#define OUTPUT_RES_Y 720  // Display Resolution Output Height

//DEFINE_string(model_pose,               "COCO",         "Model to be used (e.g. COCO, MPI, MPI_4_layers).");
#define MODEL_POSE  "COCO"                 //"Model to be used (e.g. COCO, MPI, MPI_4_layers).";
DEFINE_int32(num_gpu_start,             0,              "GPU device start number.");
DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless num_scales>1. Initial scale is always 1. If you"
                                                        " want to change the initial scale, you actually want to multiply the `net_resolution` by"
                                                        " your desired initial scale.");
DEFINE_int32(num_scales,                1,              "Number of scales to average.");
// OpenPose Rendering
DEFINE_bool(disable_blending,           false,          "If blending is enabled, it will merge the results with the original frame. If disabled, it"
                                                        " will only display the results.");
DEFINE_double(alpha_pose,               0.6,            "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will"
                                                        " hide it. Only valid for GPU rendering.");
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


class RosImgSub
{
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        cv_bridge::CvImagePtr cv_img_ptr_;

    public:
        RosImgSub(const std::string& image_topic): it_(nh_)
        {
            // Subscribe to input video feed and publish output video feed
            image_sub_ = it_.subscribe(image_topic, 1, &RosImgSub::convertImage, this);
            cv_img_ptr_ = nullptr;
        }

        ~RosImgSub(){}

        void convertImage(const sensor_msgs::ImageConstPtr& msg)
        {
            try
            {
                cv_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }

        cv_bridge::CvImagePtr& getCvImagePtr()
        {
            return cv_img_ptr_;
        }

};

int openPoseROSTutorial()
{
    op::log("OpenPose ROS Node", op::Priority::High);
    // ------------------------- INITIALIZATION -------------------------
    // Step 1 - Set logging level
        // - 0 will output all the logging messages
        // - 255 will output nothing
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.", __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // Step 2 - Read Google flags (user defined configuration)
    // outputSize
    //const auto outputSize = op::flagsToPoint(FLAGS_resolution, "1280x720");
    op::Point<int> outputSize;
    outputSize.x = OUTPUT_RES_X;
    outputSize.y = OUTPUT_RES_Y;

    op::Point<int> netInputSize;
    netInputSize.x = NET_RES_X; //656;
    netInputSize.y = NET_RES_Y; //368;

    // netInputSize
    //const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "656x368");
    // netOutputSize
    const auto netOutputSize = netInputSize;
    // poseModel
    //const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    op::PoseModel poseModel = stringToPoseModel(std::string(MODEL_POSE));
    // Check no contradictory flags enabled
    if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.)
        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
    if (FLAGS_scale_gap <= 0. && FLAGS_num_scales > 1)
        op::error("Incompatible flag configuration: scale_gap must be greater than 0 or num_scales = 1.", __LINE__, __FUNCTION__, __FILE__);
    // Logging
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // Step 3 - Initialize all required classes
    op::CvMatToOpInput cvMatToOpInput{netInputSize, FLAGS_num_scales, (float)FLAGS_scale_gap};
    op::CvMatToOpOutput cvMatToOpOutput{outputSize};
    op::PoseExtractorCaffe poseExtractorCaffe{netInputSize, netOutputSize, outputSize, FLAGS_num_scales, poseModel,
                                              model_folder_location, FLAGS_num_gpu_start};
    op::PoseRenderer poseRenderer{netOutputSize, outputSize, poseModel, nullptr, !FLAGS_disable_blending, (float)FLAGS_alpha_pose};
    //op::PoseRenderer poseRenderer{netOutputSize, outputSize, poseModel, nullptr, true, 0.6};    

    op::OpOutputToCvMat opOutputToCvMat{outputSize};
    // Step 4 - Initialize resources on desired thread (in this case single thread, i.e. we init resources here)
    poseExtractorCaffe.initializationOnThread();
    poseRenderer.initializationOnThread();
    ROS_INFO("Initialization Success");

    // Step 5 - Initialize the image subscriber
    RosImgSub ris(FLAGS_camera_topic);

    int frame_count = 0;
    const std::chrono::high_resolution_clock::time_point timerBegin = std::chrono::high_resolution_clock::now();

    ros::spinOnce();

    // Step 6 - Continuously process images from image subscriber
    while (ros::ok())
    {
        // ------------------------- POSE ESTIMATION AND RENDERING -------------------------
        // Step 1 - Get cv_image ptr and check that it is not null
        cv_bridge::CvImagePtr cvImagePtr = ris.getCvImagePtr();
        if(cvImagePtr != nullptr)
        {
            cv::Mat inputImage = cvImagePtr->image;
    
            // Step 2 - Format input image to OpenPose input and output formats
            op::Array<float> netInputArray;
            std::vector<float> scaleRatios;
            std::tie(netInputArray, scaleRatios) = cvMatToOpInput.format(inputImage);
            double scaleInputToOutput;
            op::Array<float> outputArray;
            std::tie(scaleInputToOutput, outputArray) = cvMatToOpOutput.format(inputImage);
            // Step 3 - Estimate poseKeypoints
            ROS_INFO("Performing Forward Pass");
            poseExtractorCaffe.forwardPass(netInputArray, {inputImage.cols, inputImage.rows}, scaleRatios);
            std::cout << "Forward Pass Success" << std::endl;

            const auto poseKeypoints = poseExtractorCaffe.getPoseKeypoints();
            std::cout << "    Got Keypoints" << std::endl;
            // Step 4 - Render poseKeypoints
            poseRenderer.renderPose(outputArray, poseKeypoints);
            std::cout << "    Rendering Pose" << std::endl;            
            // Step 5 - OpenPose output format to cv::Mat
            auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);
            std::cout << "    Outputing Image" << std::endl;

            // ------------------------- SHOWING RESULT AND CLOSING -------------------------
            // Step 1 - Show results
            cv::imshow("OpenPose ROS", outputImage);
            cv::waitKey(1);
            frame_count++;
        }
        ros::spinOnce();
    }

    // Measuring total time
    const double totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>
                              (std::chrono::high_resolution_clock::now()-timerBegin).count() * 1e-9;
    const std::string message = "Real-time pose estimation demo successfully finished. Total time: " 
                                + std::to_string(totalTimeSec) + " seconds. " + std::to_string(frame_count)
                                + " frames processed. Average fps is " + std::to_string(frame_count/totalTimeSec) + ".";
    op::log(message, op::Priority::Max);

    // Return successful message
    return 0;
}

int main(int argc, char** argv)
{
  google::InitGoogleLogging("openpose_ros_node");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "openpose_ros_node");

  return openPoseROSTutorial();
}