#include "vision_localizer/object_localizer.hpp"

namespace vision_localizer
{
    ObjectLocalizer::ObjectLocalizer(const rclcpp::NodeOptions &options)
        : Node("object_localizer", options)
    {
        // Declare and load camera parameters
        this->declare_parameter("camera_fov_x_deg", 0.0);
        this->declare_parameter("camera_fov_y_deg", 0.0);
        this->declare_parameter("camera_resolution_x", 0.0);
        this->declare_parameter("camera_resolution_y", 0.0);

        fov_x_deg_ = this->get_parameter("camera_fov_x_deg").as_double();
        fov_y_deg_ = this->get_parameter("camera_fov_y_deg").as_double();
        resolution_x_ = this->get_parameter("camera_resolution_x").as_double();
        resolution_y_ = this->get_parameter("camera_resolution_y").as_double();

        setupCameraParameters(); // Compute focal length and optical center

        // Obtain the transform of the image with respect to the world
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        if (tf_buffer_->canTransform("world", "camera_rgbd", tf2::TimePointZero, tf2::durationFromSec(5.0)))
        {
            try
            {
                geometry_msgs::msg::TransformStamped transformStamped =
                    tf_buffer_->lookupTransform("world", "camera_rgbd", tf2::TimePointZero);
    
                const auto &t = transformStamped.transform.translation;
                const auto &q = transformStamped.transform.rotation;
    
                // Convert quaternion to rotation matrix
                tf2::Quaternion quat(q.x, q.y, q.z, q.w);
                tf2::Matrix3x3 rot_matrix(quat);
    
                // Riempie la tua extrinsics_matrix_ come matrice 3x4 (righe concatenate)
                for (int i = 0; i < 3; ++i)
                {
                    extrinsics_matrix_[i * 4 + 0] = rot_matrix[i][0];
                    extrinsics_matrix_[i * 4 + 1] = rot_matrix[i][1];
                    extrinsics_matrix_[i * 4 + 2] = rot_matrix[i][2];
                }
    
                extrinsics_matrix_[3] = t.x;
                extrinsics_matrix_[7] = t.y;
                extrinsics_matrix_[11] = t.z;
    
                RCLCPP_INFO(this->get_logger(), "Extrinsics matrix set from tf.");
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not get transform camera → world: %s", ex.what());
                std::fill(std::begin(extrinsics_matrix_), std::end(extrinsics_matrix_), 0.0);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Transform world → camera_rgbd not available after 5 seconds");
        }

        // Create subscribers for RGB and depth image streams
        this->declare_parameter("topic_rgb_image", "/camera/rgb");
        this->declare_parameter("topic_depth_image", "/camera/depth");

        rgb_topic_name_ = this->get_parameter("topic_rgb_image").as_string();
        depth_topic_name = this->get_parameter("topic_depth_image").as_string();

        rgb_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            rgb_topic_name_, 10, std::bind(&ObjectLocalizer::rgbCallback, this, std::placeholders::_1));

        depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_name, 10, std::bind(&ObjectLocalizer::depthCallback, this, std::placeholders::_1));

        // Publisher for 3D object position
        object_position_publisher_ = this->create_publisher<vision_localizer::msg::ObjectInfo>("/object_info", 10);
    
        // Declare and load object dimensions
        this->declare_parameter("object_dimension_x", 0.05);
        this->declare_parameter("object_dimension_y", 0.05);
        this->declare_parameter("object_dimension_z", 0.05);

        object_dimensions_[0] = this->get_parameter("object_dimension_x").as_double();
        object_dimensions_[1] = this->get_parameter("object_dimension_y").as_double();
        object_dimensions_[2] = this->get_parameter("object_dimension_z").as_double();

        RCLCPP_INFO(this->get_logger(),
                    "Object dimensions: x=%.3f, y=%.3f, z=%.3f",
                    object_dimensions_[0], object_dimensions_[1], object_dimensions_[2]);

        RCLCPP_INFO(this->get_logger(), "ObjectLocalizer node started.");
    }

    void ObjectLocalizer::setupCameraParameters()
    {
        double fov_x_rad = fov_x_deg_ * M_PI / 180.0;
        double fov_y_rad = fov_y_deg_ * M_PI / 180.0;

        focal_x_px_ = (resolution_x_ / 2.0) / std::tan(fov_x_rad / 2.0);
        focal_y_px_ = (resolution_y_ / 2.0) / std::tan(fov_y_rad / 2.0);

        c_x_ = resolution_x_ / 2.0;
        c_y_ = resolution_y_ / 2.0;
    }

    void ObjectLocalizer::rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat cv_image = cv_bridge::toCvCopy(msg, "rgb8")->image;
            cv::flip(cv_image, cv_image, 0);
            cv::cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);
            cv::imshow("RGB Image", cv_image);
            cv::setMouseCallback("RGB Image", [](int event, int x, int y, int /*flags*/, void *userdata)
                                 {
                    auto self = static_cast<ObjectLocalizer*>(userdata);
                    if (event == cv::EVENT_LBUTTONDOWN && self) {
                        self->processMouseClick(x, y);
                    } }, this);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "RGB image error: %s", e.what());
        }
    }

    void ObjectLocalizer::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat depth_image = cv_bridge::toCvCopy(msg, "32FC1")->image;
            cv::flip(depth_image, depth_image, 0);
            current_depth_image_ = depth_image.clone();

            cv::Mat norm, color;
            cv::normalize(depth_image, norm, 0, 255, cv::NORM_MINMAX);
            norm.convertTo(norm, CV_8UC1);
            cv::applyColorMap(norm, color, cv::COLORMAP_JET);
            cv::imshow("Depth Map", color);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Depth image error: %s", e.what());
        }
    }

    void ObjectLocalizer::processMouseClick(int x, int y)
    {
        if (current_depth_image_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No depth image available");
            return;
        }

        if (x < 0 || x >= current_depth_image_.cols || y < 0 || y >= current_depth_image_.rows)
        {
            RCLCPP_WARN(this->get_logger(), "Click out of bounds");
            return;
        }

        float z = current_depth_image_.at<float>(y, x);
        if (z <= 0.0f || z > 10.0f)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid depth value: %.3f", z);
            return;
        }

        double x_cam = (c_x_ - x) * z / focal_x_px_;
        double y_cam = (c_y_ - y) * z / focal_y_px_;
        double z_cam = z;

        RCLCPP_INFO(this->get_logger(), "Object Position - Image Coordinates: [%.3f, %.3f, %.3f]", x_cam, y_cam, z_cam);

        auto world = transformCamToWorld(x_cam, y_cam, z_cam);

        publishObjectInfo(world[0], world[1], world[2]);
    }

    std::array<double, 3> ObjectLocalizer::transformCamToWorld(double xCam, double yCam, double zCam)
    {
        std::array<double, 3> out;
        out[0] = extrinsics_matrix_[0] * xCam + extrinsics_matrix_[1] * yCam + extrinsics_matrix_[2] * zCam + extrinsics_matrix_[3];
        out[1] = extrinsics_matrix_[4] * xCam + extrinsics_matrix_[5] * yCam + extrinsics_matrix_[6] * zCam + extrinsics_matrix_[7];
        out[2] = extrinsics_matrix_[8] * xCam + extrinsics_matrix_[9] * yCam + extrinsics_matrix_[10] * zCam + extrinsics_matrix_[11];
        RCLCPP_INFO(this->get_logger(), "Object Position - World Coordinates: [%.3f, %.3f, %.3f]", out[0], out[1], out[2]);
        return out;
    }

    void ObjectLocalizer::publishObjectInfo(double x, double y, double z)
    {
        vision_localizer::msg::ObjectInfo msg;
        msg.center.x = static_cast<float>(x);
        msg.center.y = static_cast<float>(y);
        msg.center.z = static_cast<float>(z);
        msg.size.x = object_dimensions_[0];
        msg.size.y = object_dimensions_[1];
        msg.size.z = object_dimensions_[2];

        RCLCPP_INFO(this->get_logger(), "Object Position: [%.3f, %.3f, %.3f]", x, y, z);
        object_position_publisher_->publish(msg);
    }
}