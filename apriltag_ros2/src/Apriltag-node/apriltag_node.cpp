#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <std_msgs/msg/string.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <mutex>
#include <regex>
#include <string>
#include <vector>

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/apriltag_pose.h"

#include "apriltag_ros2/msg/detected_goal.hpp"
#include "apriltag_ros2/msg/detected_goal_array.hpp"
#include "apriltag_ros2/srv/save_current_tag_goal.hpp"


class AprilTagNode : public rclcpp::Node
{
public:
    AprilTagNode()
        : Node("apriltag_node")
    {
        // ---------------------------------------------------------------------
        // AprilTag detector initialization
        // ---------------------------------------------------------------------
        td_ = apriltag_detector_create();
        tf_ = tag36h11_create();
        apriltag_detector_add_family(td_, tf_);

        // ---------------------------------------------------------------------
        // TF2 initialization
        // ---------------------------------------------------------------------
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ---------------------------------------------------------------------
        // ROS2 parameters
        // ---------------------------------------------------------------------
        this->declare_parameter<std::string>("camera_type", "oak");
        this->declare_parameter<bool>("fullscreen_display", true);

        camera_type_ = this->get_parameter("camera_type").as_string();
        fullscreen_display_ = this->get_parameter("fullscreen_display").as_bool();

        // ---------------------------------------------------------------------
        // Camera topic selection
        // ---------------------------------------------------------------------
        if (camera_type_ == "oak")
        {
            image_topic_ = "/oak/rgb/image_raw";
            camera_info_topic_ = "/oak/rgb/camera_info";
            optical_frame_ = "oak_rgb_camera_optical_frame";
            window_name_ = "OAK";
        }
        else if (camera_type_ == "realsense")
        {
            image_topic_ = "/camera/camera/color/image_raw";
            camera_info_topic_ = "/camera/camera/color/camera_info";
            optical_frame_ = "camera_color_optical_frame";
            window_name_ = "Realsense";
        }
        else
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Unknown camera_type: '%s'. Use 'oak' or 'realsense'.",
                camera_type_.c_str()
            );

            throw std::runtime_error("Invalid camera_type");
        }

        RCLCPP_INFO(this->get_logger(), "Using camera_type: %s", camera_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera info topic: %s", camera_info_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Optical frame: %s", optical_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Fullscreen display: %s", fullscreen_display_ ? "true" : "false");

        // ---------------------------------------------------------------------
        // Publisher for detected goals
        // ---------------------------------------------------------------------
        pub_goals_ = this->create_publisher<apriltag_ros2::msg::DetectedGoalArray>(
            "/detected_goals",
            10
        );

        // ---------------------------------------------------------------------
        // Camera subscribers
        // ---------------------------------------------------------------------
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_,
            10,
            std::bind(&AprilTagNode::imageCallback, this, std::placeholders::_1)
        );

        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_,
            10,
            std::bind(&AprilTagNode::cameraInfoCallback, this, std::placeholders::_1)
        );

        // ---------------------------------------------------------------------
        // Service client used by the SAVE POSE button
        // ---------------------------------------------------------------------
        save_pose_client_ =
            this->create_client<apriltag_ros2::srv::SaveCurrentTagGoal>(
                "/save_current_tag_goal"
            );

        // ---------------------------------------------------------------------
        // Gaze subscriber
        //
        // Topic:
        //   /gaze_point
        //
        // Message convention:
        //   msg.x : normalized horizontal gaze coordinate in [0, 1]
        //   msg.y : normalized vertical gaze coordinate in [0, 1]
        //   msg.z : gaze confidence, if available
        //
        // These coordinates are expected to come from the EyeTrax bridge.
        // The direct mapping to image pixels works best when the OpenCV window
        // is fullscreen and EyeTrax is calibrated on the same screen.
        // ---------------------------------------------------------------------
        sub_gaze_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/gaze_point",
            10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(gaze_mutex_);

                gaze_x_norm_ = std::clamp(msg->x, 0.0, 1.0);
                gaze_y_norm_ = std::clamp(msg->y, 0.0, 1.0);
                gaze_confidence_ = msg->z;

                has_gaze_ = true;
            }
        );

        // ---------------------------------------------------------------------
        // Voice command subscriber
        //
        // Topic:
        //   /voice_command
        //
        // Expected message:
        //   std_msgs/msg/String containing a JSON string, for example:
        //
        //   {
        //     "intent": "boost_goal",
        //     "goal_id": 2,
        //     "confidence": 0.85,
        //     "text": "tag deux"
        //   }
        //
        // This node only extracts:
        //   - "text"
        //   - "goal_id"
        //
        // and displays them on top of the OpenCV image.
        // ---------------------------------------------------------------------
        sub_voice_ = this->create_subscription<std_msgs::msg::String>(
            "/voice_command",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(voice_mutex_);

                const std::string &data = msg->data;

                // Extract the recognized sentence from the JSON string.
                std::smatch match_text;
                std::regex text_regex("\"text\"\\s*:\\s*\"([^\"]*)\"");

                if (std::regex_search(data, match_text, text_regex))
                {
                    last_voice_text_ = match_text[1].str();
                }
                else
                {
                    last_voice_text_ = "none";
                }

                // Extract the detected goal ID from the JSON string.
                // If goal_id is null, display "none".
                std::smatch match_goal;
                std::regex goal_regex("\"goal_id\"\\s*:\\s*(null|[0-9]+)");

                if (std::regex_search(data, match_goal, goal_regex))
                {
                    const std::string goal = match_goal[1].str();

                    if (goal == "null")
                    {
                        last_voice_goal_ = "none";
                    }
                    else
                    {
                        last_voice_goal_ = goal;
                    }
                }
                else
                {
                    last_voice_goal_ = "none";
                }
            }
        );

        // ---------------------------------------------------------------------
        // OpenCV display window
        // ---------------------------------------------------------------------
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);

        // Fullscreen is useful because EyeTrax gives normalized gaze
        // coordinates relative to the display/calibration area.
        if (fullscreen_display_)
        {
            cv::setWindowProperty(
                window_name_,
                cv::WND_PROP_FULLSCREEN,
                cv::WINDOW_FULLSCREEN
            );
        }

        // Mouse callback used for the SAVE POSE button.
        cv::setMouseCallback(window_name_, &AprilTagNode::mouseCallback, this);
    }

    ~AprilTagNode()
    {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_);
    }

private:
    // -------------------------------------------------------------------------
    // Camera configuration
    // -------------------------------------------------------------------------
    std::string camera_type_;
    std::string image_topic_;
    std::string camera_info_topic_;
    std::string optical_frame_;
    std::string window_name_;

    bool fullscreen_display_ = true;

    // -------------------------------------------------------------------------
    // TF2 configuration
    // -------------------------------------------------------------------------
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //std::string target_frame_ = "base"; if Franka
    std::string target_frame_ = "base_link"; //if Explorer

    // -------------------------------------------------------------------------
    // ROS2 interfaces
    // -------------------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_gaze_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_voice_;

    rclcpp::Publisher<apriltag_ros2::msg::DetectedGoalArray>::SharedPtr pub_goals_;

    rclcpp::Client<apriltag_ros2::srv::SaveCurrentTagGoal>::SharedPtr save_pose_client_;

    // -------------------------------------------------------------------------
    // AprilTag detector
    // -------------------------------------------------------------------------
    apriltag_detector_t *td_;
    apriltag_family_t *tf_;

    // -------------------------------------------------------------------------
    // Camera intrinsics
    // -------------------------------------------------------------------------
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;

    // -------------------------------------------------------------------------
    // Gaze state
    // -------------------------------------------------------------------------
    std::mutex gaze_mutex_;

    bool has_gaze_ = false;

    double gaze_x_norm_ = 0.5;
    double gaze_y_norm_ = 0.5;
    double gaze_confidence_ = 0.0;

    // -------------------------------------------------------------------------
    // Voice command state
    // -------------------------------------------------------------------------
    std::mutex voice_mutex_;

    std::string last_voice_text_ = "none";
    std::string last_voice_goal_ = "none";

    // -------------------------------------------------------------------------
    // Visible AprilTags
    // -------------------------------------------------------------------------
    std::vector<int> visible_tag_ids_;
    std::mutex visible_tags_mutex_;

    // -------------------------------------------------------------------------
    // SAVE POSE button state
    // -------------------------------------------------------------------------
    cv::Rect save_button_rect_{10, 10, 180, 40};
    std::string save_status_ = "Ready";

    // -------------------------------------------------------------------------
    // Draw the latest gaze point on top of the image.
    // -------------------------------------------------------------------------
    void drawGazeOverlay(cv::Mat &image)
    {
        std::lock_guard<std::mutex> lock(gaze_mutex_);

        // Do not draw anything until a gaze message has been received.
        if (!has_gaze_)
        {
            return;
        }

        // Convert normalized gaze coordinates to image pixel coordinates.
        //
        // This direct conversion assumes that the image occupies the same
        // normalized display area used by EyeTrax calibration.
        int gx = static_cast<int>(gaze_x_norm_ * image.cols);
        int gy = static_cast<int>(gaze_y_norm_ * image.rows);

        // Keep the point inside the image.
        gx = std::clamp(gx, 0, image.cols - 1);
        gy = std::clamp(gy, 0, image.rows - 1);

        // Select color according to confidence.
        cv::Scalar gaze_color;

        if (gaze_confidence_ >= 0.7)
        {
            gaze_color = cv::Scalar(0, 255, 0);      // Green: high confidence.
        }
        else if (gaze_confidence_ >= 0.3)
        {
            gaze_color = cv::Scalar(0, 255, 255);    // Yellow: medium confidence.
        }
        else
        {
            gaze_color = cv::Scalar(0, 0, 255);      // Red: low confidence.
        }

        // Draw gaze circle.
        cv::circle(
            image,
            cv::Point(gx, gy),
            14,
            gaze_color,
            3
        );

        // Draw gaze cross.
        cv::line(
            image,
            cv::Point(gx - 25, gy),
            cv::Point(gx + 25, gy),
            gaze_color,
            2
        );

        cv::line(
            image,
            cv::Point(gx, gy - 25),
            cv::Point(gx, gy + 25),
            gaze_color,
            2
        );

        // Draw gaze label.
        cv::putText(
            image,
            "GAZE",
            cv::Point(gx + 18, gy - 18),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            gaze_color,
            2
        );

        // Draw confidence value.
        const std::string conf_text =
            "conf: " + std::to_string(gaze_confidence_).substr(0, 4);

        cv::putText(
            image,
            conf_text,
            cv::Point(gx + 18, gy + 10),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            gaze_color,
            1
        );
    }

    // -------------------------------------------------------------------------
    // Draw the latest voice command information on top of the image.
    // -------------------------------------------------------------------------
    void drawVoiceOverlay(cv::Mat &image)
    {
        std::lock_guard<std::mutex> lock(voice_mutex_);

        // Bottom-left overlay position.
        const int x = 10;
        const int y = image.rows - 70;

        // Background rectangle to improve readability.
        cv::rectangle(
            image,
            cv::Rect(x - 5, y - 30, 620, 75),
            cv::Scalar(0, 0, 0),
            cv::FILLED
        );

        // Display the last recognized sentence.
        cv::putText(
            image,
            "Phrase: " + last_voice_text_,
            cv::Point(x, y),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(255, 255, 255),
            2
        );

        // Display the last detected vocal goal.
        cv::putText(
            image,
            "Voice goal: " + last_voice_goal_,
            cv::Point(x, y + 30),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 255),
            2
        );
    }

    // -------------------------------------------------------------------------
    // Mouse callback wrapper required by OpenCV.
    // -------------------------------------------------------------------------
    static void mouseCallback(int event, int x, int y, int flags, void *userdata)
    {
        (void)flags;

        if (!userdata)
        {
            return;
        }

        AprilTagNode *self = static_cast<AprilTagNode *>(userdata);
        self->handleMouse(event, x, y);
    }

    // -------------------------------------------------------------------------
    // Handle mouse clicks inside the OpenCV window.
    // -------------------------------------------------------------------------
    void handleMouse(int event, int x, int y)
    {
        if (event != cv::EVENT_LBUTTONDOWN)
        {
            return;
        }

        if (save_button_rect_.contains(cv::Point(x, y)))
        {
            triggerSave();
        }
    }

    // -------------------------------------------------------------------------
    // Trigger the save pose service for the first currently visible AprilTag.
    // -------------------------------------------------------------------------
    void triggerSave()
    {
        if (!save_pose_client_ || !save_pose_client_->service_is_ready())
        {
            save_status_ = "Service not ready";
            RCLCPP_WARN(this->get_logger(), "Service /save_current_tag_goal not ready");
            return;
        }

        int tag_id = -1;

        {
            std::lock_guard<std::mutex> lock(visible_tags_mutex_);

            if (visible_tag_ids_.empty())
            {
                save_status_ = "No visible tag";
                RCLCPP_WARN(this->get_logger(), "No visible tag to save");
                return;
            }

            // Minimal behavior: save the first visible tag.
            tag_id = visible_tag_ids_.front();
        }

        auto req =
            std::make_shared<apriltag_ros2::srv::SaveCurrentTagGoal::Request>();

        req->tag_id = tag_id;
        req->label = "pose";

        save_status_ = "Saving tag " + std::to_string(tag_id);

        save_pose_client_->async_send_request(
            req,
            [this, tag_id](
                rclcpp::Client<apriltag_ros2::srv::SaveCurrentTagGoal>::SharedFuture future
            )
            {
                try
                {
                    auto res = future.get();

                    if (res->success)
                    {
                        save_status_ = "Saved tag " + std::to_string(tag_id);

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Saved pose for tag %d",
                            tag_id
                        );
                    }
                    else
                    {
                        save_status_ = "Save failed";

                        RCLCPP_WARN(
                            this->get_logger(),
                            "Save failed for tag %d: %s",
                            tag_id,
                            res->message.c_str()
                        );
                    }
                }
                catch (const std::exception &e)
                {
                    save_status_ = "Service error";

                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Service exception: %s",
                        e.what()
                    );
                }
            }
        );
    }

    // -------------------------------------------------------------------------
    // Store camera intrinsic parameters.
    // -------------------------------------------------------------------------
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
    }

    // -------------------------------------------------------------------------
    // Main image callback.
    //
    // This callback:
    //   1. Converts the ROS image to OpenCV.
    //   2. Detects AprilTags.
    //   3. Estimates their poses.
    //   4. Transforms poses to the target frame.
    //   5. Publishes detected goals.
    //   6. Draws AprilTags, SAVE button, gaze overlay, and voice overlay.
    // -------------------------------------------------------------------------
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (fx_ == 0.0 || fy_ == 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Camera info not received yet!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "imageCallback called");

        // Convert ROS image to OpenCV BGR image.
        //
        // clone() is used because the image is modified for visualization.
        cv::Mat color = cv_bridge::toCvShare(msg, "bgr8")->image.clone();

        RCLCPP_INFO(
            this->get_logger(),
            "Received image of size: %dx%d",
            color.cols,
            color.rows
        );

        // Convert the image to grayscale for AprilTag detection.
        cv::Mat gray;
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

        // Create an AprilTag image header from the grayscale OpenCV image.
        image_u8_t im_header = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        // Detect AprilTags.
        zarray_t *detections = apriltag_detector_detect(td_, &im_header);

        const int ndets = zarray_size(detections);

        RCLCPP_INFO(this->get_logger(), "Number of detections: %d", ndets);

        // Reset the list of currently visible tags.
        {
            std::lock_guard<std::mutex> lock(visible_tags_mutex_);
            visible_tag_ids_.clear();
        }

        // Message published on /detected_goals.
        apriltag_ros2::msg::DetectedGoalArray msg_goals;
        msg_goals.header.stamp = this->now();
        msg_goals.header.frame_id = target_frame_;

        // ---------------------------------------------------------------------
        // Process each detected AprilTag.
        // ---------------------------------------------------------------------
        for (int i = 0; i < ndets; i++)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            RCLCPP_INFO(this->get_logger(), "Detected tag ID: %d", det->id);

            // AprilTag pose estimation input.
            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = 0.024; //size apriltag
            info.fx = fx_;
            info.fy = fy_;
            info.cx = cx_;
            info.cy = cy_;

            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);
            (void)err;

            // Extract translation.
            const double x = matd_get(pose.t, 0, 0);
            const double y = matd_get(pose.t, 1, 0);
            const double z = matd_get(pose.t, 2, 0);

            RCLCPP_INFO(
                this->get_logger(),
                "Pose X: %f Y: %f Z: %f",
                x,
                y,
                z
            );

            // Extract rotation matrix.
            const double r11 = matd_get(pose.R, 0, 0);
            const double r12 = matd_get(pose.R, 0, 1);
            const double r13 = matd_get(pose.R, 0, 2);

            const double r21 = matd_get(pose.R, 1, 0);
            const double r22 = matd_get(pose.R, 1, 1);
            const double r23 = matd_get(pose.R, 1, 2);

            const double r31 = matd_get(pose.R, 2, 0);
            const double r32 = matd_get(pose.R, 2, 1);
            const double r33 = matd_get(pose.R, 2, 2);

            // Convert rotation matrix to quaternion.
            Eigen::Matrix3d R;
            R << r11, r12, r13,
                 r21, r22, r23,
                 r31, r32, r33;

            Eigen::Quaterniond q(R);

            // Create pose in the camera frame.
            geometry_msgs::msg::Pose pose_msg;
            pose_msg.position.x = x;
            pose_msg.position.y = y;
            pose_msg.position.z = z;
            pose_msg.orientation.x = q.x();
            pose_msg.orientation.y = q.y();
            pose_msg.orientation.z = q.z();
            pose_msg.orientation.w = q.w();

            geometry_msgs::msg::PoseStamped pose_camera;
            pose_camera.header.stamp = msg->header.stamp;
            pose_camera.header.frame_id = msg->header.frame_id;
            pose_camera.pose = pose_msg;

            // Transform the AprilTag pose to the target frame.
            try
            {
                geometry_msgs::msg::TransformStamped transform_stamped =
                    tf_buffer_->lookupTransform(
                        target_frame_,
                        pose_camera.header.frame_id,
                        tf2::TimePointZero
                    );

                geometry_msgs::msg::PoseStamped pose_base;
                tf2::doTransform(pose_camera, pose_base, transform_stamped);

                apriltag_ros2::msg::DetectedGoal detected_goal;
                detected_goal.id = det->id;
                detected_goal.pose = pose_base.pose;

                msg_goals.goals.push_back(detected_goal);

                {
                    std::lock_guard<std::mutex> lock(visible_tags_mutex_);
                    visible_tag_ids_.push_back(det->id);
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Goal transformed to %s: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    target_frame_.c_str(),
                    pose_base.pose.position.x,
                    pose_base.pose.position.y,
                    pose_base.pose.position.z,
                    pose_base.pose.orientation.x,
                    pose_base.pose.orientation.y,
                    pose_base.pose.orientation.z,
                    pose_base.pose.orientation.w
                );
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Could not transform from %s to %s: %s",
                    pose_camera.header.frame_id.c_str(),
                    target_frame_.c_str(),
                    ex.what()
                );

                continue;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Goal in camera frame: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                x,
                y,
                z,
                q.x(),
                q.y(),
                q.z(),
                q.w()
            );

            // Draw AprilTag border.
            for (int j = 0; j < 4; j++)
            {
                cv::line(
                    color,
                    cv::Point(det->p[j][0], det->p[j][1]),
                    cv::Point(det->p[(j + 1) % 4][0], det->p[(j + 1) % 4][1]),
                    cv::Scalar(0, 0, 255),
                    2
                );
            }

            // Draw AprilTag ID at the center of the tag.
            cv::putText(
                color,
                std::to_string(det->id),
                cv::Point(det->c[0], det->c[1]),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(0, 255, 0),
                2
            );
        }

        // ---------------------------------------------------------------------
        // Draw SAVE POSE button.
        // ---------------------------------------------------------------------
        cv::rectangle(
            color,
            save_button_rect_,
            cv::Scalar(255, 0, 0),
            cv::FILLED
        );

        cv::putText(
            color,
            "SAVE POSE",
            cv::Point(save_button_rect_.x + 15, save_button_rect_.y + 27),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(255, 255, 255),
            2
        );

        cv::putText(
            color,
            save_status_,
            cv::Point(10, 65),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 255),
            2
        );

        // Publish detected goals.
        pub_goals_->publish(msg_goals);

        // Draw external modality overlays after all AprilTag drawings.
        drawGazeOverlay(color);
        drawVoiceOverlay(color);

        // Display the final image.
        cv::imshow(window_name_, color);
        cv::waitKey(1);

        // Free AprilTag detections.
        apriltag_detections_destroy(detections);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<AprilTagNode>());

    rclcpp::shutdown();

    return 0;
}