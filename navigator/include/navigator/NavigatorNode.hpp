#ifndef SUBSCRIBER__SUBSCRIBERNODE_HPP_
#define SUBSCRIBER__SUBSCRIBERNODE_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "irobot_create_msgs/msg/ir_intensity_vector.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"

namespace navigator
{

struct VFFVectors
{
  std::vector<float> attractive;
  std::vector<float> repulsive;
  std::vector<float> result;
  std::vector<std::vector<float>> sensors;
};

typedef enum {RED, GREEN, BLUE} VFFColor;

class NavigatorNode : public rclcpp::Node
{
public:
  NavigatorNode();

  void scan_callback(irobot_create_msgs::msg::IrIntensityVector::UniquePtr msg);
  void control_cycle();

protected:
  VFFVectors get_vff(const irobot_create_msgs::msg::IrIntensityVector & scan);

  visualization_msgs::msg::MarkerArray get_debug_vff(const VFFVectors & vff_vectors);
  visualization_msgs::msg::Marker make_marker(const std::vector<float> & vector, VFFColor vff_color);
  visualization_msgs::msg::Marker make_markers(const std::vector<float> & vector);

  irobot_create_msgs::msg::LightringLeds get_leds(irobot_create_msgs::msg::LightringLeds leds);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<irobot_create_msgs::msg::IrIntensityVector>::SharedPtr scan_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<irobot_create_msgs::msg::LightringLeds>::SharedPtr light_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vff_debug_pub_;

  irobot_create_msgs::msg::IrIntensityVector::UniquePtr last_scan_;
  
  // For real robot
  const std::vector<float> dirs {65.3, 38.0, 20.0, 3.0, -14.25, -34.0, -65.3};
  // For Create 3 simulator
  // const std::vector<float> dirs {3.0, -14.25, 20.0, -34.0, 38.0, -65.3, 65.3};
  int id;
  int led_counter = 1;
  int led_counter2 = 3;
  int min_idx;
  float min_lenght;
  int result_angle;
};

}

#endif 