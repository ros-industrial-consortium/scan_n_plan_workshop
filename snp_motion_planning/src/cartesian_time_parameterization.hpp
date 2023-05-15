#pragma once

#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/utilities/error.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>
#include <tesseract_time_parameterization/trajectory_container.h>
#include <tesseract_environment/environment.h>

/** @details Adapted from http://docs.ros.org/en/indigo/api/eigen_conversions/html/eigen__kdl_8cpp_source.html#l00090 */
template <typename T>
KDL::Frame toKDL(const T& e)
{
  KDL::Frame k;
  for (unsigned int i = 0; i < 3; ++i)
    k.p[i] = e(i, 3);
  for (unsigned int i = 0; i < 9; ++i)
    k.M.data[i] = e(i / 3, i % 3);

  return k;
}

Eigen::VectorXd fromKDL(const KDL::Twist& k)
{
  Eigen::VectorXd out(6);
  for (int i = 0; i < 6; ++i)
    out[i] = k[i];
  return out;
}

namespace snp_motion_planning
{
class CartesianTimeParameterization
{
public:
  using Ptr = std::shared_ptr<CartesianTimeParameterization>;
  using ConstPtr = std::shared_ptr<const CartesianTimeParameterization>;

  CartesianTimeParameterization(tesseract_environment::Environment::ConstPtr env, const std::string& group, const std::string& tcp,
                                const double max_translational_vel, const double max_translational_acc)
    : motion_group(env->getJointGroup(group))
    , tcp(tcp)
    , max_translational_vel(max_translational_vel)
    , max_translational_acc(max_translational_acc)
  {
    // Construct the KDL chain
    tesseract_kinematics::KDLChainData data;
    if (!tesseract_kinematics::parseSceneGraph(data, *env->getSceneGraph(), motion_group->getBaseLinkName(), tcp))
      throw std::runtime_error("Failed to construct KDL chain");

    kdl_chain_ = data.robot_chain;
  }

  bool compute(tesseract_planning::TrajectoryContainer& trajectory, double max_velocity_scaling_factor = 1.0,
               double max_acceleration_scaling_factor = 1.0) const
  {
    const double eq_radius = 1.0;  // max_translational_velocity / max_rotational_velocity;
    auto path = new KDL::Path_Composite();

    std::vector<double> times;
    times.reserve(trajectory.size());
    times.push_back(0.0);

    for (Eigen::Index i = 1; i < trajectory.size() - 1; ++i)
    {
      const Eigen::VectorXd& start_joints = trajectory.getPosition(i - 1);
      const Eigen::VectorXd& end_joints = trajectory.getPosition(i);

      // Perform FK to get Cartesian poses
      const KDL::Frame start = toKDL(motion_group->calcFwdKin(start_joints).at(tcp));
      const KDL::Frame end = toKDL(motion_group->calcFwdKin(end_joints).at(tcp));

      // Convert to KDL::Path
      KDL::RotationalInterpolation* rot_interp = new KDL::RotationalInterpolation_SingleAxis();
      KDL::Path* segment = new KDL::Path_Line(start, end, rot_interp, eq_radius);

      path->Add(segment);

      // Estimate the time to this waypoint with half-trapezoid velocity profile
      KDL::VelocityProfile_TrapHalf prof(max_velocity_scaling_factor * max_translational_vel,
                                         max_acceleration_scaling_factor * max_translational_acc, true);
      double path_length = path->PathLength() > std::numeric_limits<double>::epsilon() ?
                               path->PathLength() :
                               std::numeric_limits<double>::epsilon();

      prof.SetProfile(0.0, path_length);
      times.push_back(prof.Duration());
    }

    //
    double path_length = path->PathLength();
    auto prof = new KDL::VelocityProfile_Trap(max_velocity_scaling_factor * max_translational_vel,
                                              max_acceleration_scaling_factor * max_translational_acc);
    prof->SetProfile(0.0, path_length);
    const double duration = prof->Duration();

    // Add the last time with the duration from a double ended trapezoidal velocity profile
    times.push_back(duration);

    // Update the trajectory
    Eigen::VectorXd prev_joint_vel = Eigen::VectorXd::Zero(trajectory.dof());
    for (Eigen::Index i = 0; i < trajectory.size(); ++i)
    {
      const Eigen::VectorXd& joints = trajectory.getPosition(i);
      double dt = times[i];

      // Compute the joint velocity and acceleration
      KDL::Trajectory_Segment traj(path, prof, false);
      KDL::Twist vel = traj.Vel(dt);
      KDL::Twist acc = traj.Acc(dt);
      const Eigen::VectorXd joint_vel = computeJointVelocity(fromKDL(vel), joints);
      const Eigen::VectorXd joint_acc = computeJointAcceleration(fromKDL(acc), joints, joint_vel);

      // Check for joint velocity/acceleration limit violations
      Eigen::Array<bool, Eigen::Dynamic, 1> vel_limit_violations =
          motion_group->getLimits().velocity_limits.array() < joint_vel.array().abs();
      if (vel_limit_violations.any())
      {
        std::stringstream ss;
        ss << "Joint velocity limit violations: "
           << vel_limit_violations.cast<int>().transpose().format(Eigen::IOFormat(1, 0, " ", "\n", "[", "]"));
        CONSOLE_BRIDGE_logError(ss.str().c_str());
        return false;
      }

      Eigen::Array<bool, Eigen::Dynamic, 1> acc_limit_violations =
          motion_group->getLimits().acceleration_limits.array() < joint_acc.array().abs();
      if (acc_limit_violations.any())
      {
        std::stringstream ss;
        ss << "Joint acceleration limit violations: "
           << acc_limit_violations.cast<int>().transpose().format(Eigen::IOFormat(1, 0, " ", "\n", "[", "]"));
        CONSOLE_BRIDGE_logError(ss.str().c_str());
        return false;
      }

      // Update the trajectory container
      trajectory.setData(i, joint_vel, joint_acc, dt);
    }

    return true;
  }

  //  bool compute(tesseract_planning::TrajectoryContainer& trajectory, const std::vector<double>& max_velocity,
  //               const std::vector<double>& max_acceleration, double max_velocity_scaling_factor = 1.0,
  //               double max_acceleration_scaling_factor = 1.0) const
  //  {
  //    return false;
  //  }

  //  bool compute(tesseract_planning::TrajectoryContainer& trajectory,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_acceleration, double max_velocity_scaling_factor = 1.0,
  //               double max_acceleration_scaling_factor = 1.0) const
  //  {
  //    return false;
  //  }

  //  bool compute(tesseract_planning::TrajectoryContainer& trajectory,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_velocity,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_acceleration,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_velocity_scaling_factors,
  //               const Eigen::Ref<const Eigen::VectorXd>& max_acceleration_scaling_factors) const
  //  {
  //    return false;
  //  }

private:
  Eigen::VectorXd computeJointVelocity(const Eigen::VectorXd& twist, const Eigen::VectorXd& joint_state) const
  {
    Eigen::MatrixXd jac = motion_group->calcJacobian(joint_state, tcp);
    // TODO: Use pseudo inverse if DOF != 6
    return jac.colPivHouseholderQr().solve(twist);
  }

  Eigen::VectorXd computeJointAcceleration(const Eigen::VectorXd& twist, const Eigen::VectorXd& joint_position, const Eigen::VectorXd& joint_velocity) const
  {
    // Create a Jacobian derivative solver
    // Use the default hybrid representation because the input twist is for the end effector frame but relative to the base frame
    KDL::ChainJntToJacDotSolver solver(kdl_chain_);

    // Compute the derivative of the Jacobian
    KDL::JntArrayVel q;
    q.q.data = joint_position;
    q.qdot.data = joint_velocity;
    KDL::Twist jac_dot_q_dot;
    int error = solver.JntToJacDot(q, jac_dot_q_dot);
    if (error != 0)
      throw std::runtime_error(solver.strError(error));

    // Compute the jacbian
    Eigen::MatrixXd jac = motion_group->calcJacobian(joint_position, tcp);

    // Compute the joint accelerations
    // d/dt(x_dot) = d/dt(J * q_dot)
    // x_dot_dot = J_dot * q_dot + J * q_dot_dot
    // qdotdot = J^(-1) * (x_dot_dot - J_dot_q_dot)
    // TODO: use pseudo-inverse if DOF != 6
    return jac.colPivHouseholderQr().solve(twist - fromKDL(jac_dot_q_dot));
  }

  tesseract_kinematics::JointGroup::UPtr motion_group;
  KDL::Chain kdl_chain_;
  const std::string tcp;
  const double max_translational_vel;
  const double max_translational_acc;
};

}  // namespace snp_motion_planning
