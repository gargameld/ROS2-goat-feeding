/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/logger.hpp>

namespace mujoco_ros2_control
{

/**
 * @brief Validated access to the `<param>` key-value pairs of a ros2_control description.
 *
 * Wraps either the `<hardware>` parameters or the parameters of a single component
 * (a joint, a sensor, ...). Values are trimmed of surrounding whitespace before they are
 * parsed, because xacro readily leaves newlines and indentation inside a `<param>` element.
 *
 * Accessors without a default value throw std::invalid_argument when the parameter is
 * missing, and every accessor throws std::invalid_argument when the value does not parse or
 * lies outside the documented range. The wrapped parameter map must outlive this object.
 */
class HardwareParameters
{
public:
  /** @brief Wraps the `<hardware>` parameters of the ros2_control description. */
  explicit HardwareParameters(const hardware_interface::HardwareInfo& hardware_info);

  /** @brief Wraps the parameters of one component of the ros2_control description. */
  explicit HardwareParameters(const hardware_interface::ComponentInfo& component);

  /**
   * @brief Wraps an arbitrary parameter map.
   * @param source_description How the map is named in error messages, e.g. "sensor 'camera'".
   */
  HardwareParameters(const std::unordered_map<std::string, std::string>& parameters,
                     std::string source_description);

  /** @brief Returns the trimmed value of @p name, or nullopt when it is not set. */
  std::optional<std::string> find(const std::string& name) const;

  /** @brief Returns the trimmed value of the required parameter @p name. */
  std::string get_string(const std::string& name) const;

  /** @brief Returns the trimmed value of @p name, or @p default_value when it is not set. */
  std::string get_string(const std::string& name, const std::string& default_value) const;

  /** @brief Returns @p name parsed as a finite number that is zero or greater. */
  double get_non_negative_double(const std::string& name, double default_value) const;

  /** @brief Returns @p name parsed as a finite number greater than zero. */
  double get_positive_double(const std::string& name, double default_value) const;

  /** @brief Returns the required parameter @p name parsed as an integer greater than zero. */
  unsigned int get_positive_unsigned(const std::string& name) const;

  /**
   * @brief Splits the required parameter @p name on @p separator into trimmed, non-empty entries.
   * @throws std::invalid_argument when the parameter is missing, empty, or has an empty entry.
   */
  std::vector<std::string> get_string_list(const std::string& name, char separator = ',') const;

  /** @brief Logs every parameter of the wrapped description, for debugging a launch. */
  void log_all(const rclcpp::Logger& logger) const;

private:
  /** @brief Builds the "parameter 'x' of hardware 'y'" part of an error message. */
  std::string describe(const std::string& name) const;

  /** @brief Parses @p value as a finite double inside [@p minimum, infinity). */
  double parse_double(const std::string& name, const std::string& value, double minimum,
                      bool minimum_is_inclusive) const;

  const std::unordered_map<std::string, std::string>& parameters_;
  const std::string source_description_;
};

}  // namespace mujoco_ros2_control
