#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/link_padding.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace
{

template<typename ParameterT>
ParameterT get_or_declare(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const ParameterT & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<ParameterT>();
  }
  return node->declare_parameter<ParameterT>(name, default_value);
}

void validate_box_ids(const std::vector<std::string> & box_ids)
{
  if (box_ids.empty()) {
    throw std::runtime_error("box_ids must contain at least one collision object ID");
  }

  std::unordered_set<std::string> unique_ids;
  for (const auto & id : box_ids) {
    if (id.empty() || !unique_ids.insert(id).second) {
      throw std::runtime_error("box_ids must contain unique, non-empty IDs");
    }
  }
}

std::vector<double> read_box_geometry(
  const rclcpp::Node::SharedPtr & node,
  const std::string & id)
{
  const std::string parameter_name = "boxes." + id;
  if (!node->has_parameter(parameter_name)) {
    throw std::runtime_error("Missing parameter '" + parameter_name + "'");
  }

  const auto geometry = node->get_parameter(parameter_name).as_double_array();
  if (geometry.size() != 6U) {
    throw std::runtime_error(
            "Parameter '" + parameter_name +
            "' must be [x, y, z, size_x, size_y, size_z]");
  }
  for (const auto value : geometry) {
    if (!std::isfinite(value)) {
      throw std::runtime_error("Parameter '" + parameter_name + "' contains a non-finite value");
    }
  }
  if (geometry[3] <= 0.0 || geometry[4] <= 0.0 || geometry[5] <= 0.0) {
    throw std::runtime_error("Box dimensions in '" + parameter_name + "' must be positive");
  }

  return geometry;
}

moveit_msgs::msg::CollisionObject make_collision_box(
  const std::string & id,
  const std::string & frame_id,
  const std::vector<double> & geometry)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {geometry[3], geometry[4], geometry[5]};
  object.primitives.push_back(std::move(primitive));

  geometry_msgs::msg::Pose pose;
  pose.position.x = geometry[0];
  pose.position.y = geometry[1];
  pose.position.z = geometry[2];
  pose.orientation.w = 1.0;
  object.primitive_poses.push_back(std::move(pose));
  return object;
}

std::vector<moveit_msgs::msg::CollisionObject> read_boxes(
  const rclcpp::Node::SharedPtr & node,
  const std::string & frame_id)
{
  const auto box_ids = get_or_declare<std::vector<std::string>>(node, "box_ids", {});
  validate_box_ids(box_ids);

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.reserve(box_ids.size());
  for (const auto & id : box_ids) {
    const auto geometry = read_box_geometry(node, id);
    objects.push_back(make_collision_box(id, frame_id, geometry));
  }
  return objects;
}

// Safety expansion of the robot's own collision geometry. MoveIt applies link
// padding only to robot-vs-world checks (the boxes above plus the food octomap);
// self-collision stays unpadded unless CollisionRequest::pad_self_collisions is
// set, so the SRDF adjacencies are unaffected.
std::vector<moveit_msgs::msg::LinkPadding> read_link_padding(
  const rclcpp::Node::SharedPtr & node)
{
  const std::string prefix = "link_padding.";

  // Unlike `boxes`, the padded links are not enumerated by an explicit ID list:
  // the YAML map is read straight off the parameter overrides, so adding a link
  // is a one-line edit. Iteration order is the map's, i.e. deterministic.
  std::vector<moveit_msgs::msg::LinkPadding> padding;
  for (const auto & [name, value] :
    node->get_node_parameters_interface()->get_parameter_overrides())
  {
    if (name.rfind(prefix, 0) != 0) {
      continue;
    }

    const std::string link_name = name.substr(prefix.size());
    if (link_name.empty()) {
      throw std::runtime_error("'link_padding' must map link names to padding values");
    }

    double amount = 0.0;
    switch (value.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        amount = value.get<double>();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        amount = static_cast<double>(value.get<std::int64_t>());
        break;
      default:
        throw std::runtime_error("Padding for link '" + link_name + "' must be a number");
    }
    if (!std::isfinite(amount) || amount < 0.0) {
      throw std::runtime_error(
              "Padding for link '" + link_name + "' must be finite and non-negative");
    }

    moveit_msgs::msg::LinkPadding entry;
    entry.link_name = link_name;
    entry.padding = amount;
    padding.push_back(std::move(entry));
  }
  return padding;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("environment_loader", options);

  try {
    const auto frame_id = get_or_declare<std::string>(node, "frame_id", "map");
    if (frame_id.empty()) {
      throw std::runtime_error("frame_id must not be empty");
    }
    const auto service_name = get_or_declare<std::string>(
      node, "planning_scene_service", "/apply_planning_scene");
    const auto service_wait_timeout = get_or_declare<double>(
      node, "service_wait_timeout", 30.0);
    if (service_wait_timeout <= 0.0) {
      throw std::runtime_error("service_wait_timeout must be positive");
    }

    auto objects = read_boxes(node, frame_id);
    auto link_padding = read_link_padding(node);
    auto client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>(service_name);
    if (!client->wait_for_service(std::chrono::duration<double>(service_wait_timeout))) {
      throw std::runtime_error("Planning scene service '" + service_name + "' is unavailable");
    }

    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene.is_diff = true;
    request->scene.robot_state.is_diff = true;
    request->scene.world.collision_objects = std::move(objects);
    request->scene.link_padding = std::move(link_padding);

    const std::size_t object_count = request->scene.world.collision_objects.size();
    // MoveIt silently keeps padding entries for link names it does not know, so
    // log them: a typo shows up here and nowhere else.
    for (const auto & entry : request->scene.link_padding) {
      RCLCPP_INFO(
        node->get_logger(), "Padding link '%s' by %.3f m",
        entry.link_name.c_str(), entry.padding);
    }
    const std::size_t padded_link_count = request->scene.link_padding.size();
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Failed while calling planning scene service '" + service_name + "'");
    }
    if (!future.get()->success) {
      throw std::runtime_error("MoveIt rejected the environment planning scene diff");
    }
    RCLCPP_INFO(
      node->get_logger(), "Added %zu environment boxes in frame '%s' and padded %zu links",
      object_count, frame_id.c_str(), padded_link_count);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Unable to load environment: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
