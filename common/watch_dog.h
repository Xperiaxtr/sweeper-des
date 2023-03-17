#pragma once

#include <cmath>

namespace sweeper {
namespace common {

class WatchDog {
 public:
  bool DogIsOk(int error_times=5);
  WatchDog();
  ~WatchDog();
  void UpdataNow();

 private:
  int dog_now;
  int dog_last;
  // int dog_last_last;
  int error_counts;  //错误次数
  inline void UpdataLast();
  inline void Reset();
};

}  // namespace common
}  // namespace sweeper