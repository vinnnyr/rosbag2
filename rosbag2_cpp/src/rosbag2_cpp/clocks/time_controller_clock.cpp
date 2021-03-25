// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "rcpputils/thread_safety_annotations.hpp"
#include "rosbag2_cpp/clocks/time_controller_clock.hpp"
#include "rosbag2_cpp/types.hpp"

namespace rosbag2_cpp
{

class TimeControllerClockImpl
{
public:
  /**
   * Stores an exact time match between a system steady clock and the playback ROS clock.
   * This snapshot is taken whenever a factor changes such that a new reference is needed,
   * such as pause, resume, rate change, or jump
   */
  struct TimeReference
  {
    rcutils_time_point_value_t ros;
    std::chrono::steady_clock::time_point steady;
  };

  TimeControllerClockImpl() = default;
  virtual ~TimeControllerClockImpl() = default;

  template<typename T>
  rcutils_duration_value_t duration_nanos(const T & duration)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  }

  rcutils_time_point_value_t steady_to_ros(std::chrono::steady_clock::time_point steady_time)
  {
    return reference.ros + static_cast<rcutils_duration_value_t>(
      rate * duration_nanos(steady_time - reference.steady));
  }

  std::chrono::steady_clock::time_point ros_to_steady(rcutils_time_point_value_t ros_time)
  {
    const auto diff_nanos = static_cast<rcutils_duration_value_t>(
      (ros_time - reference.ros) / rate);
    return reference.steady + std::chrono::nanoseconds(diff_nanos);
  }

  std::mutex mutex;
  std::condition_variable cv;
  std::chrono::nanoseconds sleep_timeout;
  PlayerClock::NowFunction now_fn RCPPUTILS_TSA_GUARDED_BY(mutex);
  double rate RCPPUTILS_TSA_GUARDED_BY(mutex) = 1.0;
  TimeReference reference RCPPUTILS_TSA_GUARDED_BY(mutex);
};

TimeControllerClock::TimeControllerClock(
  rcutils_time_point_value_t starting_time,
  std::chrono::nanoseconds sleep_timeout,
  double rate,
  NowFunction now_fn)
: impl_(std::make_unique<TimeControllerClockImpl>())
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->now_fn = now_fn;
  impl_->sleep_timeout = sleep_timeout;
  impl_->reference.ros = starting_time;
  impl_->reference.steady = impl_->now_fn();
  impl_->rate = rate;
}

TimeControllerClock::~TimeControllerClock()
{}

rcutils_time_point_value_t TimeControllerClock::now() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->steady_to_ros(impl_->now_fn());
}

bool TimeControllerClock::sleep_until(rcutils_time_point_value_t until)
{
  {
    std::unique_lock<std::mutex> lock(impl_->mutex);
    const auto until_timeout = impl_->now_fn() + impl_->sleep_timeout;
    auto steady_until = impl_->ros_to_steady(until);
    if (until_timeout < steady_until) {
      steady_until = until_timeout;
    }
    impl_->cv.wait_until(lock, steady_until);
  }
  return now() >= until;
}

double TimeControllerClock::get_rate() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->rate;
}

}  // namespace rosbag2_cpp
