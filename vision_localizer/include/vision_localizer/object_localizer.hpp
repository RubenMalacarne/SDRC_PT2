#ifndef VISION_LOCALIZER_IMAGE_LISTENER_HPP_
#define VISION_LOCALIZER_IMAGE_LISTENER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <array>
#include <vector>
#include <vision_localizer/msg/object_info.hpp>

namespace vision_localizer
{
    /**
     * @brief Node that localizes objects from RGBD images.
     */
    class ObjectLocalizer : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor for the ObjectLocalizer node.
         * @param options Node options (to load parameters from file).
         */
        explicit ObjectLocalizer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        /**
         * @brief Default destructor.
         */
        ~ObjectLocalizer() override = default;

    private:
        /**
         * @brief Callback for the depth image.
         * @param msg Message containing the depth image (32FC1 format).
         */
        void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

        /**
         * @brief Processes a mouse click at given pixel coordinates.
         * @param x X pixel coordinate (column).
         * @param y Y pixel coordinate (row).
         */
        void processMouseClick(int x, int y);

        /**
         * @brief Publishes the localized object position.
         * @param x X coordinate in world frame.
         * @param y Y coordinate in world frame.
         * @param z Z coordinate in world frame.
         */
        void publishObjectInfo(double x, double y, double z);

        /**
         * @brief Callback for the RGB image.
         * @param msg Message containing the RGB image.
         */
        void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg);

        /**
         * @brief Calculates camera internal parameters such as focal lengths and optical centers.
         */
        void setupCameraParameters();

        /**
         * @brief Transforms a 3D point from camera frame to world frame.
         * @param xCam X coordinate in camera frame.
         * @param yCam Y coordinate in camera frame.
         * @param zCam Z coordinate in camera frame.
         * @return 3D point in world coordinates.
         */
        std::array<double, 3> transformCamToWorld(double xCam, double yCam, double zCam);

        /// Subscription to the RGB image stream.
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscriber_;

        /// Subscription to the depth image stream.
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;

        /// Publishes the computed object 3D position.
        rclcpp::Publisher<vision_localizer::msg::ObjectInfo>::SharedPtr object_position_publisher_;

        /// Last received depth image (32FC1, meters).
        cv::Mat current_depth_image_;

        /// Horizontal field of view (in degrees).
        double fov_x_deg_;

        /// Vertical field of view (in degrees).
        double fov_y_deg_;

        /// Camera resolution along X axis (in pixels).
        double resolution_x_;

        /// Camera resolution along Y axis (in pixels).
        double resolution_y_;

        /// Focal length along X axis (in pixels).
        double focal_x_px_;

        /// Focal length along Y axis (in pixels).
        double focal_y_px_;

        /// Optical center X coordinate (in pixels).
        double c_x_;

        /// Optical center Y coordinate (in pixels).
        double c_y_;

        /// Extrinsic transformation matrix from camera frame to world frame (3x4).
        double extrinsics_matrix_[12];

        /// Dimensions of the localized object [x, y, z] in meters.
        std::array<double, 3> object_dimensions_;

        /// Topic names
        std::string rgb_topic_name_;
        std::string depth_topic_name;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
}

#endif