/**
 * @file conversions.h
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
#ifndef TESSERACT_ROSUTILS_ROS2_CONVERSIONS_H
#define TESSERACT_ROSUTILS_ROS2_CONVERSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tesseract_msgs/msg/joint_state.hpp>
#include <tesseract_command_language/core/instruction.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace tesseract_rosutils::ros2
{
/**
 * @brief Convert STD Vector to Eigen Vector
 * @param vector The STD Vector to be converted
 * @return Eigen::VectorXd
 */
Eigen::VectorXd toEigen(const std::vector<double>& vector);

/**
 * @brief Converts JointState position to Eigen vector in the order provided by joint_names
 * @param joint_state The JointState
 * @param joint_names The vector joint names used to order the output
 * @return Eigen::VectorXd in the same order as joint_names
 */
Eigen::VectorXd toEigen(const sensor_msgs::msg::JointState& joint_state, const std::vector<std::string>& joint_names);

/**
 * @brief Convert a joint trajectory to csv format and write to file
 * @param trajectory Trajectory to be written to file
 * @param file_path The location to save the file
 * @param separator The separator to use
 * @return true if successful
 */
bool toCSVFile(const std::vector<tesseract_msgs::msg::JointState>& trajectory_msg,
               const std::string& file_path,
               char separator = ',');

/**
 * @brief Convert CSVFile to JointTrajectory
 * @param file_path File path to CSV
 * @param separator The separator to use
 * @return Joint Trajectory
 */
std::vector<tesseract_msgs::msg::JointState> trajectoryFromCSVFile(const std::string& file_path, char separator = ',');

}  // namespace tesseract_rosutils
#endif  // TESSERACT_ROSUTILS_ROS2_CONVERSIONS_H
