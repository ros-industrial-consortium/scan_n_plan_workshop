/**
 * @file conversions.cpp
 * @brief Tesseract ROS Utils conversions
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2018, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <fstream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/ros2/conversions.h>
#include <tesseract_rosutils/ros2/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/flatten_utils.h>

namespace tesseract_rosutils::ros2
{
Eigen::VectorXd toEigen(const std::vector<double>& vector)
{
  return Eigen::VectorXd::Map(vector.data(), static_cast<long>(vector.size()));
}

Eigen::VectorXd toEigen(const sensor_msgs::msg::JointState& joint_state, const std::vector<std::string>& joint_names)
{
  Eigen::VectorXd position;
  position.resize(static_cast<long>(joint_names.size()));
  int i = 0;
  for (const auto& joint_name : joint_names)
  {
    auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
    assert(it != joint_state.name.end());
    size_t index = static_cast<size_t>(std::distance(joint_state.name.begin(), it));
    position[i] = joint_state.position[index];
    ++i;
  }

  return position;
}

bool toCSVFile(const std::vector<tesseract_msgs::msg::JointState>& trajectory_msg,
               const std::string& file_path,
               char separator)
{
  std::ofstream myfile;
  myfile.open(file_path);

  for (const auto& js : trajectory_msg)
  {
    myfile << js.joint_names.size();
    std::copy(js.joint_names.begin(), js.joint_names.end(), std::ostream_iterator<std::string>(myfile, &separator));
    std::copy(js.position.begin(), js.position.end(), std::ostream_iterator<double>(myfile, &separator));
    myfile << "," + std::to_string(js.time_from_start.sec + js.time_from_start.nanosec / 1.0e9) + ",\n";
  }
  myfile.close();
  return true;
}

std::vector<tesseract_msgs::msg::JointState> trajectoryFromCSVFile(const std::string& file_path, char separator)
{
  std::vector<tesseract_msgs::msg::JointState> trajectory;
  std::ifstream csv_file(file_path);

  // Make sure the file is open
  if (!csv_file.is_open())
    throw std::runtime_error("Could not open csv file");

  // Read data, line by line
  std::string line;
  while (std::getline(csv_file, line))
  {
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(std::string(&separator)), boost::token_compress_on);

    if (!tesseract_common::isNumeric(tokens[0]))
      throw std::runtime_error("jointTrajectoryFromCSVFile: Invalid format");

    std::size_t num_vals{ 0 };
    std::size_t cnt{ 0 };
    tesseract_common::toNumeric<std::size_t>(tokens[cnt++], num_vals);

    tesseract_msgs::msg::JointState js;
    js.joint_names.reserve(num_vals);
    js.position.reserve(num_vals);

    // Get Joint Names
    for (std::size_t i = 0; i < num_vals; ++i)
    {
      std::string jn = tokens[cnt++];
      tesseract_common::trim(jn);
      js.joint_names.push_back(jn);
    }

    // Get Positions
    for (std::size_t i = 0; i < num_vals; ++i)
    {
      if (!tesseract_common::isNumeric(tokens[cnt++]))
        throw std::runtime_error("jointTrajectoryFromCSVFile: Invalid format");

      double val = 0;
      tesseract_common::toNumeric<double>(tokens[cnt], val);
      js.position.push_back(val);
    }

    if (!tesseract_common::isNumeric(tokens[cnt++]))
      throw std::runtime_error("jointTrajectoryFromCSVFile: Invalid format");

    double val = 0;
    tesseract_common::toNumeric<double>(tokens[cnt], val);
    js.time_from_start = rclcpp::Duration(std::chrono::duration<double>(val));

    trajectory.push_back(js);
  }

  // Close file
  csv_file.close();

  return trajectory;
}

}  // namespace tesseract_rosutils
