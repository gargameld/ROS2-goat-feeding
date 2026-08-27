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

#include "mujoco_ros2_control/hardware_parameters.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <rclcpp/logging.hpp>

namespace mujoco_ros2_control
{
namespace
{

std::string trim_whitespace(std::string value)
{
  value.erase(0, value.find_first_not_of(" \t\n\r\f\v"));
  value.erase(value.find_last_not_of(" \t\n\r\f\v") + 1);
  return value;
}

std::string describe_component(const hardware_interface::ComponentInfo& component)
{
  const std::string component_type = component.type.empty() ? "component" : component.type;
  return component_type + " '" + component.name + "'";
}

}  // namespace

HardwareParameters::HardwareParameters(const hardware_interface::HardwareInfo& hardware_info)
  : HardwareParameters(hardware_info.hardware_parameters, "hardware '" + hardware_info.name + "'")
{
}

HardwareParameters::HardwareParameters(const hardware_interface::ComponentInfo& component)
  : HardwareParameters(component.parameters, describe_component(component))
{
}

HardwareParameters::HardwareParameters(const std::unordered_map<std::string, std::string>& parameters,
                                       std::string source_description)
  : parameters_(parameters), source_description_(std::move(source_description))
{
}

std::optional<std::string> HardwareParameters::find(const std::string& name) const
{
  const auto parameter = parameters_.find(name);
  if (parameter == parameters_.end())
  {
    return std::nullopt;
  }
  return trim_whitespace(parameter->second);
}

std::string HardwareParameters::get_string(const std::string& name) const
{
  const auto value = find(name);
  if (!value.has_value())
  {
    throw std::invalid_argument("Missing " + describe(name));
  }
  return value.value();
}

std::string HardwareParameters::get_string(const std::string& name, const std::string& default_value) const
{
  return find(name).value_or(default_value);
}

double HardwareParameters::get_non_negative_double(const std::string& name, double default_value) const
{
  const auto value = find(name);
  if (!value.has_value())
  {
    return default_value;
  }
  return parse_double(name, value.value(), 0.0, true);
}

double HardwareParameters::get_positive_double(const std::string& name, double default_value) const
{
  const auto value = find(name);
  if (!value.has_value())
  {
    return default_value;
  }
  return parse_double(name, value.value(), 0.0, false);
}

unsigned int HardwareParameters::get_positive_unsigned(const std::string& name) const
{
  const std::string value = get_string(name);

  std::size_t parsed_characters = 0;
  unsigned long parsed_number = 0;
  try
  {
    parsed_number = std::stoul(value, &parsed_characters);
  }
  catch (const std::exception&)
  {
    parsed_characters = 0;
  }

  if (parsed_characters != value.size() || parsed_number == 0 ||
      parsed_number > std::numeric_limits<unsigned int>::max())
  {
    throw std::invalid_argument(describe(name) + " must be a positive integer, got '" + value + "'");
  }

  return static_cast<unsigned int>(parsed_number);
}

std::vector<std::string> HardwareParameters::get_string_list(const std::string& name, char separator) const
{
  const std::string value = get_string(name);
  if (value.empty())
  {
    throw std::invalid_argument(describe(name) + " must not be empty");
  }

  std::vector<std::string> entries;
  std::istringstream value_stream(value);
  std::string entry;
  while (std::getline(value_stream, entry, separator))
  {
    entry = trim_whitespace(entry);
    if (entry.empty())
    {
      throw std::invalid_argument(describe(name) + " contains an empty entry");
    }
    entries.push_back(entry);
  }
  return entries;
}

void HardwareParameters::log_all(const rclcpp::Logger& logger) const
{
  for (const auto& [name, value] : parameters_)
  {
    RCLCPP_INFO(logger, "Parameter of %s: '%s' = '%s'", source_description_.c_str(), name.c_str(), value.c_str());
  }
}

std::string HardwareParameters::describe(const std::string& name) const
{
  return "parameter '" + name + "' of " + source_description_;
}

double HardwareParameters::parse_double(const std::string& name, const std::string& value, double minimum,
                                        bool minimum_is_inclusive) const
{
  std::size_t parsed_characters = 0;
  double parsed_number = 0.0;
  try
  {
    parsed_number = std::stod(value, &parsed_characters);
  }
  catch (const std::exception&)
  {
    parsed_characters = 0;
  }

  const bool above_minimum = minimum_is_inclusive ? parsed_number >= minimum : parsed_number > minimum;
  if (parsed_characters != value.size() || !std::isfinite(parsed_number) || !above_minimum)
  {
    const std::string bound = minimum_is_inclusive ? "greater than or equal to " : "greater than ";
    throw std::invalid_argument(describe(name) + " must be a finite number " + bound +
                                std::to_string(minimum) + ", got '" + value + "'");
  }
  return parsed_number;
}

}  // namespace mujoco_ros2_control
