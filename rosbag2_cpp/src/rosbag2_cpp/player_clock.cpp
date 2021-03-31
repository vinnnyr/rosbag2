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
#include "rosbag2_cpp/player_clock.hpp"
#include "rosbag2_cpp/types.hpp"

namespace rosbag2_cpp
{

class PlayerClockImpl
{
public:
  /**
   * Stores an exact time match between a system steady clock and the playback ROS clock.
   * This snapshot is taken whenever a factor changes such that a new reference is needed,
   * such as pause, resume, rate change, or jump
   */
  struct TimeReference
  {
    PlayerClock::ROSTimePoint ros;
    PlayerClock::SteadyTimePoint steady;
  };

  PlayerClockImpl() = default;
  virtual ~PlayerClockImpl() = default;

  template<typename T>
  rcutils_duration_value_t duration_nanos(const T & duration)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
  }

  PlayerClock::ROSTimePoint steady_to_ros(PlayerClock::SteadyTimePoint steady_time)
  {
    return reference.ros + (rate * duration_nanos(steady_time - reference.steady));
  }

  PlayerClock::SteadyTimePoint ros_to_steady(PlayerClock::ROSTimePoint ros_time)
  {
    const rcutils_duration_value_t diff_nanos = (ros_time - reference.ros) / rate;
    return reference.steady + std::chrono::nanoseconds(diff_nanos);
  }

  void snapshot(PlayerClock::ROSTimePoint ros_now)
  {
    reference.ros = ros_now;
    reference.steady = steady_now();
  }

  PlayerClock::ROSTimePoint ros_now()
  {
    if (paused) {
      return reference.ros;
    }
    return steady_to_ros(steady_now());
  }

  PlayerClock::NowFunction steady_now;
  std::condition_variable cv;
  std::mutex mutex;

  double rate RCPPUTILS_TSA_GUARDED_BY(mutex) = 1.0;
  bool paused RCPPUTILS_TSA_GUARDED_BY(mutex) = false;
  TimeReference reference RCPPUTILS_TSA_GUARDED_BY(mutex);
};

PlayerClock::PlayerClock(ROSTimePoint starting_time, double rate, NowFunction now_fn)
: impl_(std::make_unique<PlayerClockImpl>())
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->steady_now = now_fn;
  impl_->rate = rate;
  impl_->snapshot(starting_time);
}

PlayerClock::~PlayerClock()
{}

PlayerClock::ROSTimePoint PlayerClock::now() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->ros_now();
}

bool PlayerClock::sleep_until(ROSTimePoint until)
{
  {
    std::unique_lock<std::mutex> lock(impl_->mutex);
    SteadyTimePoint steady_until = impl_->ros_to_steady(until);
    impl_->cv.wait_until(lock, steady_until);
  }
  return now() >= until;
}

double PlayerClock::get_rate() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->rate;
}

void PlayerClock::pause()
{
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->paused) {
      return;
    }
    // Note: needs to not be paused when taking snapshot, otherwise it will use last ros ref
    impl_->snapshot(now());
    impl_->paused = true;
  }
  impl_->cv.notify_all();
}

void PlayerClock::resume()
{
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->paused) {
      return;
    }
    // Note: needs to not be paused when taking snapshot, otherwise it will use last ros ref
    impl_->paused = false;
    impl_->snapshot(now());
  }
  impl_->cv.notify_all();
}

bool PlayerClock::is_paused() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->paused;
}

}  // namespace rosbag2_cpp
