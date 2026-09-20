// 移植自 uzh-rpg/rpg_mpc (GPLv3) include/rpg_mpc/mpc_controller.h。
// 相对原版的改动（都是为了脱离ROS耦合，run()内部真正的MPC调用逻辑一行未改）：
// 1) quadrotor_common::{QuadStateEstimate,Trajectory,ControlCommand} 替换成
//    mpcctrl::{QuadState,Trajectory,ControlCommand}（见types.h），字段名对齐。
// 2) 构造函数不再接ros::NodeHandle，改接rclcpp::Node*只用于loadParameters()
//    一次性读参数；原版还订阅了"mpc/point_of_interest"（相机对准点，本项目
//    机载相机不需要MPC主动指向控制）和"autopilot/off"（原版autopilot状态机
//    专用，本项目FSM在px4ctrl自己那一套，不用rpg的autopilot包）两个话题、
//    以及发布预测轨迹供rviz可视化——这三个都是调试/可视化附加功能，不在核心
//    控制回路上，本次移植先不搬，需要时可以后补（对应原版的
//    pointOfInterestCallback/offCallback/publishPrediction三个函数）。
// 3) ros::Time替换成double（Unix时间戳，秒），避免头文件耦合rclcpp::Time。
#pragma once

#include <thread>

#include <Eigen/Eigen>

#include "mpcctrl/mpc_params.h"
#include "mpcctrl/mpc_wrapper.h"
#include "mpcctrl/types.h"

namespace mpcctrl {

enum STATE {
  kPosX = 0,
  kPosY = 1,
  kPosZ = 2,
  kOriW = 3,
  kOriX = 4,
  kOriY = 5,
  kOriZ = 6,
  kVelX = 7,
  kVelY = 8,
  kVelZ = 9
};

enum INPUT {
  kThrust = 0,
  kRateX = 1,
  kRateY = 2,
  kRateZ = 3
};

template<typename T>
class MpcController {
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static_assert(kStateSize == 10,
                "MpcController: Wrong model size. Number of states does not match.");
  static_assert(kInputSize == 4,
                "MpcController: Wrong model size. Number of inputs does not match.");

  explicit MpcController(const MpcParams<T>& initial_params);
  MpcController() = default;
  // 构造函数里起了preparation_thread_，run()里唯一的join()发生在第一次
  // 真正被调用的时候——如果对象在从未调用过run()的情况下就被销毁（比如
  // 一直没等到odom就退出了，实测跑出来的真实情况），preparation_thread_
  // 还处于joinable状态，std::thread的析构函数遇到这种情况会直接调用
  // std::terminate()，进程会带着"terminate called without an active
  // exception"这种不干净的方式退出。显式析构函数兜底join一下。
  ~MpcController() {
    if (preparation_thread_.joinable()) {
      preparation_thread_.join();
    }
  }

  ControlCommand off();

  // 强制下一次run()从悬停初始猜测重新求解，不用上一次的解热启动——
  // 挂进FSM的resetThrustMapping()调用点（MANUAL_CTRL->AUTO_HOVER、
  // 进入AUTO_TAKEOFF），跟px4ctrl/so3ctrl在同样两个时机清空推力估计/
  // 位置积分是同一个道理："这是一次全新的起飞/悬停，之前累积的状态
  // 不该带过来"。
  void resetWarmStart() { solve_from_scratch_ = true; }

  ControlCommand run(
      const QuadState& state_estimate,
      const Trajectory& reference_trajectory,
      const MpcParams<T>& params);

  // MPC求解完成后预测的整段状态/输入轨迹，run()调用之后可读，供节点层做
  // 调试发布（例如可视化预测路径）用，替代原版的publishPrediction()。
  const Eigen::Matrix<T, kStateSize, kSamples + 1>& predictedStates() const
  {
    return predicted_states_;
  }
  const Eigen::Matrix<T, kInputSize, kSamples>& predictedInputs() const
  {
    return predicted_inputs_;
  }
  T timingFeedbackSeconds() const { return timing_feedback_; }
  T timingPreparationSeconds() const { return timing_preparation_; }

private:
  bool setStateEstimate(const QuadState& state_estimate);

  bool setReference(const Trajectory& reference_trajectory);

  ControlCommand updateControlCommand(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
      double time);

  void preparationThread();

  bool setNewParams(MpcParams<T>& params);

  // Parameters
  MpcParams<T> params_;

  // MPC
  MpcWrapper<T> mpc_wrapper_;

  // Preparation Thread
  std::thread preparation_thread_;

  // Variables
  T timing_feedback_{T(1e-3)};
  T timing_preparation_{T(1e-3)};
  bool solve_from_scratch_{true};
  Eigen::Matrix<T, kStateSize, 1> est_state_{
      (Eigen::Matrix<T, kStateSize, 1>() << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0).finished()};
  Eigen::Matrix<T, kStateSize, kSamples + 1> reference_states_{
      Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()};
  Eigen::Matrix<T, kInputSize, kSamples + 1> reference_inputs_{
      Eigen::Matrix<T, kInputSize, kSamples + 1>::Zero()};
  Eigen::Matrix<T, kStateSize, kSamples + 1> predicted_states_{
      Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()};
  Eigen::Matrix<T, kInputSize, kSamples> predicted_inputs_{
      Eigen::Matrix<T, kInputSize, kSamples>::Zero()};
};

} // namespace mpcctrl
