#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "irobot_create_msgs/msg/ir_intensity_vector.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"

#include "navigator/NavigatorNode.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace navigator
{

NavigatorNode::NavigatorNode()
: Node("subscriber_node")
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);
  light_pub_ = create_publisher<irobot_create_msgs::msg::LightringLeds>("cmd_lightring", 100);
  scan_sub_ = create_subscription<irobot_create_msgs::msg::IrIntensityVector>(
    "ir_intensity", rclcpp::SensorDataQoS(), std::bind(&NavigatorNode::scan_callback, this, _1));
  vff_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vff_debug", 100);
  timer_ = create_wall_timer(50ms, std::bind(&NavigatorNode::control_cycle, this));
}
void
NavigatorNode::scan_callback(irobot_create_msgs::msg::IrIntensityVector::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void
NavigatorNode::control_cycle()
{
  if (last_scan_ == nullptr || (now() - last_scan_->header.stamp) > 1s) {
    return;
  }

  const double MAX_SPEED = 0.2;
  const double MAX_TURN = 0.8;

  // Maximum movement speeds for the Create 3:
  // const double MAX_SPEED = 0.5;
  // const double MAX_TURN = 2.0;
  
  // Get VFF vectors
  const VFFVectors & vff = get_vff(*last_scan_);

  // Create ouput message, controlling speed limits
  geometry_msgs::msg::Twist vel;
  vel.linear.x = std::clamp(vff.result[0] * MAX_SPEED, -MAX_SPEED, MAX_SPEED);
  vel.angular.z = std::clamp(vff.result[1] * MAX_TURN, -MAX_TURN, MAX_TURN);
    
  vel_pub_->publish(vel);

  // Produce debug information, if any interested
  if (vff_debug_pub_->get_subscription_count() > 0) {
    vff_debug_pub_->publish(get_debug_vff(vff));
  }

  // Light ring
  irobot_create_msgs::msg::LightringLeds leds;
  leds.header.frame_id = "base_link";
  leds.header.stamp = now();

  if (vff.repulsive[0] != 0) {
    leds.override_system = true;
    leds = get_leds(leds);
  } else {
    leds.override_system = false;
  }

  light_pub_->publish(leds);
}

VFFVectors
NavigatorNode::get_vff(const irobot_create_msgs::msg::IrIntensityVector & scan)
{
  // Init vectors
  VFFVectors vff_vector;
  vff_vector.attractive = {1.0, 0.0};  // Robot wants to go forward
  vff_vector.repulsive = {0.0, 0.0};
  vff_vector.result = {0.0, 0.0};

  int active_vectors = 0;
  std::vector<float> values;

  // Analyze readings 
  for (int i = 0; i < static_cast<int>(scan.readings.size()); i++) {
    std::vector<float> vector {0.0, 0.0};
    float distance;
    float complementary_dist;
    float angle = M_PI / 180 * dirs[i];
    float oposite_angle = angle + M_PI;

    int x = scan.readings[i].value;
    if (x < 37) {
      distance = 82.2453 - 2.9149*x + 3.9393*pow(10,-2)*pow(x,2);  
    } else if (x < 330) {
      distance = 33.0739 - 0.13588*x + 2.0024*pow(10,-4)*pow(x,2);  
    } else {
      distance = 12.7179 - 8.7737*pow(10,-3)*x + 1.8066*pow(10,-6)*pow(x,2);
    }
    distance /= 50;
    distance = std::clamp(distance, 0.0f, 1.0f);
    
    if (distance > 0.70) {
      distance = 1.0;
    } else {
      active_vectors++;
    }

    // Creating vectors for debugging
    vector[0] = cos(angle) * distance;
    vector[1] = sin(angle) * distance;
    vff_vector.sensors.push_back(vector);
    values.push_back(distance);

    // Adding the vector to repulsive vector
    complementary_dist = 1 - distance;
    vff_vector.repulsive[0] += cos(oposite_angle) * complementary_dist;
    vff_vector.repulsive[1] += sin(oposite_angle) * complementary_dist;
  }
  
  // Dividing the repulsive vector with the number of added vectors
  // to shorten the lenght.
  // (1.5 and 3 are multipliers to enhance the navigation characteristics)
  if (active_vectors != 0) {
    vff_vector.repulsive[0] = vff_vector.repulsive[0] / active_vectors * 1.5;
    vff_vector.repulsive[1] = vff_vector.repulsive[1] / active_vectors * 3;
  }
  
  // Creating the result vector
  vff_vector.result[0] = (vff_vector.repulsive[0] + vff_vector.attractive[0]);
  vff_vector.result[1] = (vff_vector.repulsive[1] + vff_vector.attractive[1]);

  // Calculating values to be used with the LEDs
  result_angle = atan2(vff_vector.result[0],vff_vector.result[1]) * 180 / M_PI;
  min_idx = std::min_element(values.begin(), values.end()) - values.begin();
  min_lenght = values[min_idx];

  return vff_vector;
}

visualization_msgs::msg::MarkerArray
NavigatorNode::get_debug_vff(const VFFVectors & vff_vectors)
{
  visualization_msgs::msg::MarkerArray marker_array;

  marker_array.markers.push_back(make_marker(vff_vectors.attractive, BLUE));
  marker_array.markers.push_back(make_marker(vff_vectors.repulsive, RED));
  marker_array.markers.push_back(make_marker(vff_vectors.result, GREEN));
  id = 3;
  for (auto i : vff_vectors.sensors) {
    marker_array.markers.push_back(make_markers(i));
  }

  return marker_array;
}

visualization_msgs::msg::Marker
NavigatorNode::make_marker(const std::vector<float> & vector, VFFColor vff_color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = now();
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.id = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  geometry_msgs::msg::Point end;
  start.x = vector[0];
  start.y = vector[1];
  marker.points = {end, start};

  marker.scale.x = 0.05;
  marker.scale.y = 0.1;

  switch (vff_color) {
    case RED:
      marker.id = 0;
      marker.color.r = 1.0;
      break;
    case GREEN:
      marker.id = 1;
      marker.color.g = 1.0;
      break;
    case BLUE:
      marker.id = 2;
      marker.color.b = 1.0;
      break;
  }
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::msg::Marker
NavigatorNode::make_markers(const std::vector<float> & vector)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "base_link";
  marker.header.stamp = now();
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.id = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  geometry_msgs::msg::Point end;
  start.x = vector[0];
  start.y = vector[1];
  marker.points = {end, start};

  marker.scale.x = 0.05;
  marker.scale.y = 0.1;

  marker.id = id;
  id++;

  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.2;

  return marker;
}

irobot_create_msgs::msg::LightringLeds
NavigatorNode::get_leds(irobot_create_msgs::msg::LightringLeds leds)
{ 
  // LED behaviour 1:
  /*
  irobot_create_msgs::msg::LedColor led;
  leds.leds[led_counter] = led;
  
  led_counter++;
  if (led_counter > 5) {
    led_counter = 0;
  }
  
  led.blue = 255;
  leds.leds[led_counter] = led;
  
  led.blue = 0;
  leds.leds[led_counter2] = led;
  led_counter2--;
  if (led_counter2 < 0) {
    led_counter2 = 5;
  }

  if (led_counter == led_counter2) {
    led.red = 255;
    led.blue = 255;
  } else {
    led.red = 255;
  }

  leds.leds[led_counter2] = led;
  return leds;
  */

  // LED behaviour 2:
  
  irobot_create_msgs::msg::LedColor led;
  led.red = 255;
  led.green = std::clamp(min_lenght - 0.1f, 0.0f, 1.0f) * 255;
  led.blue = std::clamp(min_lenght - 0.1f, 0.0f, 1.0f) * 255;
  if (result_angle < -120) {
    leds.leds[0] = led;
  } else if (result_angle < -60) {
    leds.leds[5] = led; 
  } else if (result_angle < 0) {
    leds.leds[4] = led; 
  } else if (result_angle < 60) {
    leds.leds[3] = led; 
  } else if (result_angle < 120) {
    leds.leds[2] = led; 
  } else {
    leds.leds[1] = led; 
  }
  return leds;
}

}