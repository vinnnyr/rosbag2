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

#ifndef ROSBAG2_CPP__CLOCKS__TIME_CONTROLLER_CLOCK_HPP_
#define ROSBAG2_CPP__CLOCKS__TIME_CONTROLLER_CLOCK_HPP_

#include <chrono>
#include <memory>

#include "rosbag2_cpp/clocks/player_clock.hpp"

namespace rosbag2_cpp
{

/**
 * Version of the PlayerClock interface that has control over time.
 * It does not listen to any external ROS Time Source and can optionally publish /clock
 */
class TimeControllerClockImpl;
class TimeControllerClock : public PlayerClock
{
public:
  /**
   * Constructor.
   *
   * \param starting_time: provides an initial offset for managing time
   *    This will likely be the timestamp of the first message in the bag
   * \param sleep_timeout: maximum amount of time to sleep even if time is not reached
   *    This helps dictate spin rate when clock is paused, as well as allowing periodic return
   *    of control even if messages are far apart.
   *    Value <= 0 means that sleep_until could theoretically wait forever without returning.
   * \param rate: Rate of playback, a unit-free ratio. 1.0 is real-time
   * \param now_fn: Function used to get the current steady time
   *   defaults to std::chrono::steady_clock::now
   *   Used to control for unit testing, or for specialized needs
   * \throws std::runtime_error if rate is <= 0
   */
  ROSBAG2_CPP_PUBLIC
  TimeControllerClock(
    rcutils_time_point_value_t starting_time,
    std::chrono::nanoseconds sleep_timeout,
    double rate = 1.0,
    NowFunction now_fn = std::chrono::steady_clock::now);

  ROSBAG2_CPP_PUBLIC
  virtual ~TimeControllerClock();

  /**
   * Calculate and return current rcutils_time_point_value_t based on starting time, playback rate, pause state.
   */
  ROSBAG2_CPP_PUBLIC
  rcutils_time_point_value_t now() const override;

  /**
   * Try to sleep (non-busy) the current thread until the provided time is reached - according to this Clock
   *
   * The user should not take action based on this sleep until it returns true.
   *
   * \param until: The ROS time to sleep until
   * \return true if the `until` has been reached, false if timeout or awakened early.
   */
  ROSBAG2_CPP_PUBLIC
  bool sleep_until(rcutils_time_point_value_t until) override;

  /**
   * Return the current playback rate.
   */
  ROSBAG2_CPP_PUBLIC
  double get_rate() const override;

private:
  std::unique_ptr<TimeControllerClockImpl> impl_;
};


}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CLOCKS__TIME_CONTROLLER_CLOCK_HPP_
