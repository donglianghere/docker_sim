#ifndef __UAV_UTILS_H
#define __UAV_UTILS_H

// ROS1原版这里include <ros/ros.h>只是为了用ROS_ASSERT_MSG，本身跟ROS没有
// 别的耦合。ROS2没有等价宏，这里改用标准assert，顺带让这个纯数学工具头文件
// 彻底不依赖rclcpp，任何ROS2/无ROS环境都能直接用。
#include <cassert>
#include <cstdio>
#include <sstream>

#include <uav_utils/converters.h>
#include <uav_utils/geometry_utils.h>

namespace uav_utils
{

/* judge if value belongs to [low,high] */
template <typename T, typename T2>
bool
in_range(T value, const T2& low, const T2& high)
{
  assert(low < high);
  return (low <= value) && (value <= high);
}

/* judge if value belongs to [-limit, limit] */
template <typename T, typename T2>
bool
in_range(T value, const T2& limit)
{
  assert(limit > 0);
  return in_range(value, -limit, limit);
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& low, const T2& high)
{
  assert(low < high);
  if (value < low)
  {
    value = low;
  }

  if (value > high)
  {
    value = high;
  }

  return;
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& limit)
{
  assert(limit > 0);
  limit_range(value, -limit, limit);
}

typedef std::stringstream DebugSS_t;
} // end of namespace uav_utils

#endif
