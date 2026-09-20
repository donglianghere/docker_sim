// 移植自 uzh-rpg/rpg_mpc (GPLv3) src/mpc_controller.cpp。
// run()/setStateEstimate()/setReference()/updateControlCommand()/
// preparationThread()/setNewParams() 六个函数的核心算法逻辑逐行保留未改，
// 改动集中在：类型替换为mpcctrl::types.h（不再是quadrotor_common的ROS消息
// 封装）、ros::Time换成double秒时间戳、ROS_INFO换成可选的stderr打印、
// 构造函数/point_of_interest/off/publishPrediction三处ROS耦合按
// mpc_controller.h里注释说明的范围裁剪。
#include "mpcctrl/mpc_controller.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>

namespace mpcctrl {

namespace {
double nowSeconds()
{
  return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}
}  // namespace

template<typename T>
MpcController<T>::MpcController(const MpcParams<T>& initial_params) :
    mpc_wrapper_(MpcWrapper<T>())
{
  params_ = initial_params;
  setNewParams(params_);

  solve_from_scratch_ = true;
  preparation_thread_ = std::thread(&MpcWrapper<T>::prepare, mpc_wrapper_);
}

template<typename T>
ControlCommand MpcController<T>::off()
{
  ControlCommand command;
  command.zero();
  return command;
}

template<typename T>
ControlCommand MpcController<T>::run(
    const QuadState& state_estimate,
    const Trajectory& reference_trajectory,
    const MpcParams<T>& params) {
  const double call_time = nowSeconds();
  const clock_t start = clock();
  if (params.changed_) {
    params_ = params;
    setNewParams(params_);
  }

  preparation_thread_.join();

  // Convert everything into Eigen format.
  setStateEstimate(state_estimate);
  setReference(reference_trajectory);

  static const bool do_preparation_step(false);

  // Get the feedback from MPC.
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
  if (solve_from_scratch_) {
    if (params_.print_info_)
      std::fprintf(stderr, "[mpcctrl] Solving MPC with hover as initial guess.\n");
    mpc_wrapper_.solve(est_state_);
    solve_from_scratch_ = false;
  } else {
    mpc_wrapper_.update(est_state_, do_preparation_step);
  }
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);

  // Start a thread to prepare for the next execution.
  preparation_thread_ = std::thread(&MpcController<T>::preparationThread, this);

  // Timing
  const clock_t end = clock();
  timing_feedback_ = 0.9 * timing_feedback_ +
                     0.1 * double(end - start) / CLOCKS_PER_SEC;
  if (params_.print_info_)
    std::fprintf(stderr, "[mpcctrl] MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms\n",
                 timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

  // Return the input control command.
  return updateControlCommand(predicted_states_.col(0),
                              predicted_inputs_.col(0),
                              call_time);
}

template<typename T>
bool MpcController<T>::setStateEstimate(
    const QuadState& state_estimate) {
  est_state_(kPosX) = state_estimate.position.x();
  est_state_(kPosY) = state_estimate.position.y();
  est_state_(kPosZ) = state_estimate.position.z();
  est_state_(kOriW) = state_estimate.orientation.w();
  est_state_(kOriX) = state_estimate.orientation.x();
  est_state_(kOriY) = state_estimate.orientation.y();
  est_state_(kOriZ) = state_estimate.orientation.z();
  est_state_(kVelX) = state_estimate.velocity.x();
  est_state_(kVelY) = state_estimate.velocity.y();
  est_state_(kVelZ) = state_estimate.velocity.z();
  const bool quaternion_norm_ok = abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
  return quaternion_norm_ok;
}

template<typename T>
bool MpcController<T>::setReference(
    const Trajectory& reference_trajectory) {
  reference_states_.setZero();
  reference_inputs_.setZero();

  const T dt = mpc_wrapper_.getTimestep();
  Eigen::Matrix<T, 3, 1> acceleration;
  const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
  Eigen::Quaternion<T> q_heading;
  Eigen::Quaternion<T> q_orientation;
  bool quaternion_norm_ok(true);
  if (reference_trajectory.points.size() == 1) {
    q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
        reference_trajectory.points.front().heading,
        Eigen::Matrix<T, 3, 1>::UnitZ()));
    q_orientation = reference_trajectory.points.front().orientation.template cast<T>() * q_heading;
    reference_states_ = (Eigen::Matrix<T, kStateSize, 1>()
        << reference_trajectory.points.front().position.template cast<T>(),
        q_orientation.w(),
        q_orientation.x(),
        q_orientation.y(),
        q_orientation.z(),
        reference_trajectory.points.front().velocity.template cast<T>()
    ).finished().replicate(1, kSamples + 1);

    acceleration << reference_trajectory.points.front().acceleration.template cast<T>() - gravity;
    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() << acceleration.norm(),
        reference_trajectory.points.front().bodyrates.template cast<T>()
    ).finished().replicate(1, kSamples + 1);
  } else {
    auto iterator(reference_trajectory.points.begin());
    const double t_start = reference_trajectory.points.begin()->time_from_start;
    auto last_element = reference_trajectory.points.end();
    last_element = std::prev(last_element);

    for (int i = 0; i < kSamples + 1; i++) {
      while ((iterator->time_from_start - t_start) <= i * dt &&
             iterator != last_element) {
        iterator++;
      }

      q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
          iterator->heading, Eigen::Matrix<T, 3, 1>::UnitZ()));
      q_orientation = q_heading * iterator->orientation.template cast<T>();
      reference_states_.col(i) << iterator->position.template cast<T>(),
          q_orientation.w(),
          q_orientation.x(),
          q_orientation.y(),
          q_orientation.z(),
          iterator->velocity.template cast<T>();
      if (reference_states_.col(i).segment(kOriW, 4).dot(
          est_state_.segment(kOriW, 4)) < 0.0)
        reference_states_.block(kOriW, i, 4, 1) =
            -reference_states_.block(kOriW, i, 4, 1);
      acceleration << iterator->acceleration.template cast<T>() - gravity;
      reference_inputs_.col(i) << acceleration.norm(),
          iterator->bodyrates.template cast<T>();
      quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
    }
  }
  return quaternion_norm_ok;
}

template<typename T>
ControlCommand MpcController<T>::updateControlCommand(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
    double time) {
  Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();

  // Bound inputs for sanity.
  input_bounded(INPUT::kThrust) = std::max(params_.min_thrust_,
                                           std::min(params_.max_thrust_, input_bounded(INPUT::kThrust)));
  input_bounded(INPUT::kRateX) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateX)));
  input_bounded(INPUT::kRateY) = std::max(-params_.max_bodyrate_xy_,
                                          std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateY)));
  input_bounded(INPUT::kRateZ) = std::max(-params_.max_bodyrate_z_,
                                          std::min(params_.max_bodyrate_z_, input_bounded(INPUT::kRateZ)));

  ControlCommand command;

  command.timestamp = time;
  command.armed = true;
  command.control_mode = ControlMode::BODY_RATES;
  command.expected_execution_time = time;
  command.collective_thrust = input_bounded(INPUT::kThrust);
  command.bodyrates.x() = input_bounded(INPUT::kRateX);
  command.bodyrates.y() = input_bounded(INPUT::kRateY);
  command.bodyrates.z() = input_bounded(INPUT::kRateZ);
  command.orientation.w() = state(STATE::kOriW);
  command.orientation.x() = state(STATE::kOriX);
  command.orientation.y() = state(STATE::kOriY);
  command.orientation.z() = state(STATE::kOriZ);
  return command;
}

template<typename T>
void MpcController<T>::preparationThread() {
  const clock_t start = clock();

  mpc_wrapper_.prepare();

  // Timing
  const clock_t end = clock();
  timing_preparation_ = 0.9 * timing_preparation_ +
                        0.1 * double(end - start) / CLOCKS_PER_SEC;
}

template<typename T>
bool MpcController<T>::setNewParams(MpcParams<T>& params) {
  mpc_wrapper_.setCosts(params.Q_, params.R_);
  mpc_wrapper_.setLimits(
      params.min_thrust_, params.max_thrust_,
      params.max_bodyrate_xy_, params.max_bodyrate_z_);
  mpc_wrapper_.setCameraParameters(params.p_B_C_, params.q_B_C_);
  params.changed_ = false;
  return true;
}


template
class MpcController<float>;

template
class MpcController<double>;

} // namespace mpcctrl
