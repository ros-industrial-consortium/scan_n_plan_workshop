#ifndef SNP_MOTION_PLANNING_UTILS_H
#define SNP_MOTION_PLANNING_UTILS_H

#include <tesseract_motion_planners/robot_config.h>
#include <tesseract_motion_planners/descartes/descartes_vertex_evaluator.h>
#include <descartes_light/core/edge_evaluator.h>

namespace snp
{

inline bool isRobotConfigValid(tesseract_planning::RobotConfig config)
{
  return (config == tesseract_planning::RobotConfig::NUT || config == tesseract_planning::RobotConfig::FUT);
}

inline bool isRobotConfigValid(tesseract_planning::RobotConfig start_config, tesseract_planning::RobotConfig end_config)
{
  if (start_config == end_config)
    return true;

  if (start_config == tesseract_planning::RobotConfig::NUT && end_config == tesseract_planning::RobotConfig::FUT)
    return true;

  if (start_config == tesseract_planning::RobotConfig::FUT && end_config == tesseract_planning::RobotConfig::NUT)
    return true;

  return false;
}

class DescartesStateValidator : public tesseract_planning::DescartesVertexEvaluator
{
public:
  DescartesStateValidator(tesseract_kinematics::JointGroup::ConstPtr manip,
                          std::string robot_base_link,
                          std::string robot_tip_link)
    : manip_(std::move(manip))
    , robot_base_link_(std::move(robot_base_link))
    , robot_tip_link_(std::move(robot_tip_link))
    , limits_(manip_->getLimits().joint_limits)
  {
  }

  bool operator()(const Eigen::Ref<const Eigen::VectorXd>& vertex) const override
  {
    Eigen::Vector2i sign_correction(-1, 1);
    auto robot_config =
        tesseract_planning::getRobotConfig<double>(*manip_, robot_base_link_, robot_tip_link_, vertex, sign_correction);

    if (!isRobotConfigValid(robot_config))
      return false;

    // Do not return redundant solutions
    Eigen::VectorXi start_redun = tesseract_planning::getJointTurns<double>(vertex.tail(3));
    if (start_redun != Eigen::Vector3i::Zero())
      return false;

    return tesseract_common::satisfiesPositionLimits(vertex, limits_);
  }

protected:
  tesseract_kinematics::JointGroup::ConstPtr manip_{ nullptr };
  std::string robot_base_link_;
  std::string robot_tip_link_;
  Eigen::MatrixX2d limits_;
};

template <typename FloatType>
class RobotConfigEdgeEvaluator : public descartes_light::EdgeEvaluator<FloatType>
{
public:
  RobotConfigEdgeEvaluator(tesseract_kinematics::JointGroup::ConstPtr manip,
                           std::string robot_base_link,
                           std::string robot_tip_link)
    : manip_(std::move(manip))
    , robot_base_link_(std::move(robot_base_link))
    , robot_tip_link_(std::move(robot_tip_link))
  {
  }

  std::pair<bool, FloatType> evaluate(const descartes_light::State<FloatType>& start,
                                      const descartes_light::State<FloatType>& end) const override
  {
    // Consider the edge:
    Eigen::Vector2i sign_correction(-1, 1);
    auto start_config = tesseract_planning::getRobotConfig<FloatType>(*manip_, robot_base_link_, robot_tip_link_, start.values, sign_correction);
    auto end_config = tesseract_planning::getRobotConfig<FloatType>(*manip_, robot_base_link_, robot_tip_link_, end.values, sign_correction);

    if (isRobotConfigValid(start_config, end_config))
      return std::make_pair(true, FloatType(0));

    return std::make_pair(false, FloatType(0));
  }

protected:
  tesseract_kinematics::JointGroup::ConstPtr manip_;
  std::string robot_base_link_;
  std::string robot_tip_link_;
};

template <typename FloatType>
class WeightedEuclideanDistanceEdgeEvaluator : public descartes_light::EdgeEvaluator<FloatType>
{
public:
  WeightedEuclideanDistanceEdgeEvaluator(const Eigen::VectorXd& weights) : weights_(weights.cast<FloatType>()) {}

  std::pair<bool, FloatType> evaluate(const descartes_light::State<FloatType>& start,
                                      const descartes_light::State<FloatType>& end) const override
  {
    FloatType cost = (weights_.array() * (end - start).array().abs()).sum();
    return std::make_pair(true, cost);
  }

private:
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> weights_;
};

}

#endif // SNP_MOTION_PLANNING_UTILS_H
