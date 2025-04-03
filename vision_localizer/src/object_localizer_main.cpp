#include "vision_localizer/object_localizer.hpp"

using namespace vision_localizer;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectLocalizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
