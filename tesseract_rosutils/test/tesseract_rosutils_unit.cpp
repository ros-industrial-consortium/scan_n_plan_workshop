#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rosutils/conversions.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_environment/environment.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_srdf/kinematics_information.h>

#include <tesseract_msgs/EnvironmentState.h>
#include <tesseract_common/types.h>
#include <Eigen/Eigen>

using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_rosutils;

class TesseractROSUtilsUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;

  void SetUp() override
  {
    auto locator = std::make_shared<ROSResourceLocator>();
    env_ = std::make_shared<Environment>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
    EXPECT_TRUE(env_->init(urdf_path, srdf_path, locator));
  }
};

TEST_F(TesseractROSUtilsUnit, Instantiation)  // NOLINT
{
  using namespace tesseract_rosutils;
}

TEST_F(TesseractROSUtilsUnit, processEnvironmentStateMsg)  // NOLINT
{
  ros::Time::init();

  tesseract_msgs::EnvironmentState environment_state_msg;

  toMsg(environment_state_msg, *env_);

  EXPECT_EQ(environment_state_msg.id, env_->getName());
  EXPECT_EQ(environment_state_msg.revision, env_->getRevision());

  const std::string link_name1 = "link_n1";
  const std::string joint_name1 = "joint_n1";
  Link link_1(link_name1);

  Joint joint_1(joint_name1);
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;
  auto cmd = std::make_shared<AddLinkCommand>(link_1, joint_1);

  Commands commands;
  commands.push_back(cmd);

  env_->applyCommands(commands);

  tesseract_msgs::EnvironmentState environment_state_msg2;
  toMsg(environment_state_msg2, *env_);

  EXPECT_EQ(environment_state_msg2.id, env_->getName());
  EXPECT_EQ(environment_state_msg2.revision, env_->getRevision());
  EXPECT_EQ(environment_state_msg.revision + 1, environment_state_msg2.revision);
}

TEST_F(TesseractROSUtilsUnit, toFromMsgTesseract)  // NOLINT
{
  tesseract_msgs::Environment tesseract_msg;
  EXPECT_TRUE(toMsg(tesseract_msg, env_));

  std::string filepath = "/tmp/tesseract.bin";
  EXPECT_TRUE(toFile<tesseract_msgs::Environment>(filepath, tesseract_msg));

  auto new_tesseract_msg = fromFile<tesseract_msgs::Environment>(filepath);

  auto new_tesseract = fromMsg(tesseract_msg);
  EXPECT_TRUE(new_tesseract);
}

TEST_F(TesseractROSUtilsUnit, toFromFile)  // NOLINT
{
  std_msgs::ColorRGBA msg;
  msg.r = 1.1f;
  msg.g = 2.2f;
  msg.b = 3.3f;
  msg.a = 4.4f;

  std::string filepath = "/tmp/rgb_a_msg.bin";
  EXPECT_TRUE(toFile<std_msgs::ColorRGBA>(filepath, msg));

  auto new_msg = fromFile<std_msgs::ColorRGBA>(filepath);
  EXPECT_DOUBLE_EQ(msg.r, new_msg.r);
  EXPECT_DOUBLE_EQ(msg.g, new_msg.g);
  EXPECT_DOUBLE_EQ(msg.b, new_msg.b);
  EXPECT_DOUBLE_EQ(msg.a, new_msg.a);
}

TEST_F(TesseractROSUtilsUnit, KinematicsInformation)  // NOLINT
{
  tesseract_srdf::KinematicsInformation kin_info;
  kin_info.group_names = { "manipulator1", "manipulator2", "manipulator3" };
  kin_info.chain_groups["manipulator1"] = { std::make_pair("base_link", "tip_link") };
  kin_info.joint_groups["manipulator2"] = { "joint_1", "joint_2", "joint_3" };
  kin_info.link_groups["manipulator3"] = { "base_link", "link_1", "link_2" };
  tesseract_srdf::GroupsJointState js;
  js["joint_0"] = 1.1;
  js["joint_1"] = 2.1;
  tesseract_srdf::GroupsJointStates jss;
  jss["home"] = js;
  kin_info.group_states["manipulator1"] = jss;

  tesseract_srdf::GroupsTCPs gts;
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  gts["sander"] = p;
  kin_info.group_tcps["manipulator1"] = gts;

  kin_info.kinematics_plugin_info.search_paths.insert("/usr/local/lib");
  kin_info.kinematics_plugin_info.search_libraries.insert("tesseract_kdl_factories");

  {
    tesseract_common::PluginInfo info;
    info.class_name = "KDLFwdKinChainFactory";
    info.is_default = true;
    info.config["base_link"] = "base_link";
    info.config["tip_link"] = "tool0";
    kin_info.kinematics_plugin_info.fwd_plugin_infos["manipulator1"]["KDLFwdKinChain"] = info;
  }

  {
    tesseract_common::PluginInfo info;
    info.class_name = "KDLInvKinChainLMAFactory";
    info.is_default = true;
    info.config["base_link"] = "base_link";
    info.config["tip_link"] = "tool0";
    kin_info.kinematics_plugin_info.inv_plugin_infos["manipulator2"]["KDLInvKinChainLMA"] = info;
  }

  {
    tesseract_common::PluginInfo info;
    info.class_name = "KDLInvKinChainNRFactory";
    info.is_default = true;
    info.config["base_link"] = "base_link";
    info.config["tip_link"] = "tool0";
    kin_info.kinematics_plugin_info.inv_plugin_infos["manipulator2"]["KDLInvKinChainNR"] = info;
  }

  tesseract_msgs::KinematicsInformation kin_info_msg;
  tesseract_rosutils::toMsg(kin_info_msg, kin_info);

  EXPECT_EQ(kin_info.group_names.size(), kin_info_msg.group_names.size());
  EXPECT_EQ(std::vector<std::string>(kin_info.group_names.begin(), kin_info.group_names.end()),
            kin_info_msg.group_names);

  EXPECT_EQ(kin_info.chain_groups.size(), kin_info_msg.chain_groups.size());
  EXPECT_EQ(kin_info_msg.chain_groups[0].name, "manipulator1");

  EXPECT_EQ(kin_info.joint_groups.size(), kin_info_msg.joint_groups.size());
  EXPECT_EQ(kin_info_msg.joint_groups[0].name, "manipulator2");

  EXPECT_EQ(kin_info.link_groups.size(), kin_info_msg.link_groups.size());
  EXPECT_EQ(kin_info_msg.link_groups[0].name, "manipulator3");

  EXPECT_EQ(kin_info.group_states.size(), kin_info_msg.group_joint_states.size());
  EXPECT_EQ(kin_info_msg.group_joint_states[0].name, "manipulator1");

  EXPECT_EQ(kin_info.group_tcps.size(), kin_info_msg.group_tcps.size());
  EXPECT_EQ(kin_info_msg.group_tcps[0].name, "manipulator1");
  EXPECT_EQ(kin_info_msg.group_tcps[0].tcps.size(), 1);
  EXPECT_EQ(kin_info_msg.group_tcps[0].tcps[0].name, "sander");

  EXPECT_EQ(kin_info.kinematics_plugin_info.search_paths.size(),
            kin_info_msg.kinematics_plugin_info.search_paths.size());
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.search_paths[0], "/usr/local/lib");

  EXPECT_EQ(kin_info.kinematics_plugin_info.search_libraries.size(),
            kin_info_msg.kinematics_plugin_info.search_libraries.size());
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.search_libraries[0], "tesseract_kdl_factories");

  EXPECT_EQ(kin_info.kinematics_plugin_info.fwd_plugin_infos.size(),
            kin_info_msg.kinematics_plugin_info.group_fwd_plugins.size());
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_fwd_plugins[0].group, "manipulator1");
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_fwd_plugins[0].plugins.size(), 1);
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_fwd_plugins[0].plugins[0].first, "KDLFwdKinChain");
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_fwd_plugins[0].plugins[0].second.class_name,
            "KDLFwdKinChainFactory");
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_fwd_plugins[0].plugins[0].second.is_default, true);
  EXPECT_FALSE(kin_info_msg.kinematics_plugin_info.group_fwd_plugins[0].plugins[0].second.config.empty());

  EXPECT_EQ(kin_info.kinematics_plugin_info.inv_plugin_infos.size(),
            kin_info_msg.kinematics_plugin_info.group_inv_plugins.size());
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].group, "manipulator2");
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].plugins.size(), 2);
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].plugins[0].first, "KDLInvKinChainLMA");
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].plugins[0].second.class_name,
            "KDLInvKinChainLMAFactory");
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].plugins[0].second.is_default, true);
  EXPECT_FALSE(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].plugins[0].second.config.empty());

  EXPECT_EQ(kin_info.kinematics_plugin_info.inv_plugin_infos.size(),
            kin_info_msg.kinematics_plugin_info.group_inv_plugins.size());
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].group, "manipulator2");
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].plugins.size(), 2);
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].plugins[1].first, "KDLInvKinChainNR");
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].plugins[1].second.class_name,
            "KDLInvKinChainNRFactory");
  EXPECT_EQ(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].plugins[1].second.is_default, true);
  EXPECT_FALSE(kin_info_msg.kinematics_plugin_info.group_inv_plugins[0].plugins[1].second.config.empty());

  tesseract_srdf::KinematicsInformation kin_info2;
  tesseract_rosutils::fromMsg(kin_info2, kin_info_msg);

  EXPECT_EQ(kin_info2.group_names.size(), kin_info_msg.group_names.size());
  EXPECT_EQ(std::vector<std::string>(kin_info.group_names.begin(), kin_info.group_names.end()),
            kin_info_msg.group_names);

  EXPECT_EQ(kin_info2.chain_groups.size(), kin_info_msg.chain_groups.size());
  EXPECT_TRUE(kin_info2.chain_groups.find("manipulator1") != kin_info2.chain_groups.end());

  EXPECT_EQ(kin_info2.joint_groups.size(), kin_info_msg.joint_groups.size());
  EXPECT_TRUE(kin_info2.joint_groups.find("manipulator2") != kin_info2.joint_groups.end());

  EXPECT_EQ(kin_info2.link_groups.size(), kin_info_msg.link_groups.size());
  EXPECT_TRUE(kin_info2.link_groups.find("manipulator3") != kin_info2.link_groups.end());

  EXPECT_EQ(kin_info2.group_states.size(), kin_info_msg.group_joint_states.size());
  EXPECT_TRUE(kin_info2.group_states.find("manipulator1") != kin_info2.group_states.end());

  EXPECT_EQ(kin_info2.group_tcps.size(), kin_info_msg.group_tcps.size());
  EXPECT_TRUE(kin_info2.group_tcps.find("manipulator1") != kin_info2.group_tcps.end());

  EXPECT_EQ(kin_info2.kinematics_plugin_info.search_paths.size(),
            kin_info_msg.kinematics_plugin_info.search_paths.size());
  EXPECT_EQ(*kin_info2.kinematics_plugin_info.search_paths.begin(), "/usr/local/lib");

  EXPECT_EQ(kin_info2.kinematics_plugin_info.search_libraries.size(),
            kin_info_msg.kinematics_plugin_info.search_libraries.size());
  EXPECT_EQ(*kin_info2.kinematics_plugin_info.search_libraries.begin(), "tesseract_kdl_factories");

  EXPECT_EQ(kin_info2.kinematics_plugin_info.fwd_plugin_infos.size(),
            kin_info_msg.kinematics_plugin_info.group_fwd_plugins.size());
  EXPECT_NO_THROW(kin_info2.kinematics_plugin_info.fwd_plugin_infos.at("manipulator1"));
  EXPECT_EQ(kin_info2.kinematics_plugin_info.fwd_plugin_infos.at("manipulator1").size(), 1);
  EXPECT_NO_THROW(kin_info2.kinematics_plugin_info.fwd_plugin_infos.at("manipulator1").at("KDLFwdKinChain"));
  EXPECT_EQ(kin_info2.kinematics_plugin_info.fwd_plugin_infos.at("manipulator1").at("KDLFwdKinChain").class_name,
            "KDLFwdKinChainFactory");
  EXPECT_EQ(kin_info2.kinematics_plugin_info.fwd_plugin_infos.at("manipulator1").at("KDLFwdKinChain").is_default, true);
  EXPECT_TRUE(kin_info2.kinematics_plugin_info.fwd_plugin_infos.at("manipulator1").at("KDLFwdKinChain").config);

  EXPECT_EQ(kin_info2.kinematics_plugin_info.inv_plugin_infos.size(),
            kin_info_msg.kinematics_plugin_info.group_inv_plugins.size());
  EXPECT_NO_THROW(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2"));
  EXPECT_EQ(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2").size(), 2);
  EXPECT_NO_THROW(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2").at("KDLInvKinChainLMA"));
  EXPECT_EQ(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2").at("KDLInvKinChainLMA").class_name,
            "KDLInvKinChainLMAFactory");
  EXPECT_EQ(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2").at("KDLInvKinChainLMA").is_default,
            true);
  EXPECT_TRUE(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2").at("KDLInvKinChainLMA").config);

  EXPECT_EQ(kin_info2.kinematics_plugin_info.inv_plugin_infos.size(),
            kin_info_msg.kinematics_plugin_info.group_inv_plugins.size());
  EXPECT_NO_THROW(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2"));
  EXPECT_EQ(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2").size(), 2);
  EXPECT_NO_THROW(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2").at("KDLInvKinChainNR"));
  EXPECT_EQ(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2").at("KDLInvKinChainNR").class_name,
            "KDLInvKinChainNRFactory");
  EXPECT_EQ(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2").at("KDLInvKinChainNR").is_default,
            true);
  EXPECT_TRUE(kin_info2.kinematics_plugin_info.inv_plugin_infos.at("manipulator2").at("KDLInvKinChainNR").config);
}

TEST_F(TesseractROSUtilsUnit, toRosJointTrajectory)  // NOLINT
{
  std::vector<std::string> joint_names{ "joint1", "joint2", "joint3", "joint4" };
  std::vector<tesseract_common::JointState> tesseract_joint_trajectory;
  tesseract_common::JointState tesseract_joint_state;

  trajectory_msgs::JointTrajectory ros_joint_trajectory;
  trajectory_msgs::JointTrajectoryPoint ros_joint_state;
  ros_joint_trajectory.joint_names = joint_names;

  tesseract_scene_graph::SceneState env_state;
  env_state.joints[joint_names[0]] = 0;
  env_state.joints[joint_names[1]] = 0;
  env_state.joints[joint_names[2]] = 1;
  env_state.joints[joint_names[3]] = 1;

  // point 1
  ros_joint_state.positions = std::vector<double>{ 40, 40, 2, 1 };
  ros_joint_state.velocities = std::vector<double>{ 0, 0, 0, 0 };
  ros_joint_state.accelerations = std::vector<double>{ 0, 0, 0, 0 };
  ros_joint_state.effort = std::vector<double>{ 0, 0, 0, 0 };

  tesseract_joint_state.joint_names = std::vector<std::string>{ "joint1", "joint2", "joint3" };
  tesseract_joint_state.position.resize(3);
  tesseract_joint_state.position << 40, 40, 2;
  ros_joint_trajectory.points.push_back(ros_joint_state);
  tesseract_joint_trajectory.push_back(tesseract_joint_state);

  // point 2
  ros_joint_state.positions = std::vector<double>{ 40, 10, 2, 40.1 };
  ros_joint_state.velocities = std::vector<double>{ 0, 1, 0, 1 };
  ros_joint_state.accelerations = std::vector<double>{ 0, 1, 0, 1 };
  ros_joint_state.effort = std::vector<double>{ 0, 1, 0, 1 };

  tesseract_joint_state.joint_names = std::vector<std::string>{ "joint2", "joint4" };
  tesseract_joint_state.position.resize(2);
  tesseract_joint_state.position << 10, 40.1;
  tesseract_joint_state.velocity.resize(2);
  tesseract_joint_state.velocity << 1, 1;
  tesseract_joint_state.acceleration.resize(2);
  tesseract_joint_state.acceleration << 1, 1;
  tesseract_joint_state.effort.resize(2);
  tesseract_joint_state.effort << 1, 1;
  ros_joint_trajectory.points.push_back(ros_joint_state);
  tesseract_joint_trajectory.push_back(tesseract_joint_state);

  trajectory_msgs::JointTrajectory calculated_trajectory =
      tesseract_rosutils::toMsg(tesseract_joint_trajectory, env_state);

  EXPECT_EQ(ros_joint_trajectory, calculated_trajectory);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
