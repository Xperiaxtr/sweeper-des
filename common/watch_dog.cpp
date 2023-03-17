#include "watch_dog.h"

namespace sweeper {
namespace common {

WatchDog::WatchDog() : dog_now(0), dog_last(0), error_counts(0) {}
WatchDog::~WatchDog() {}

void WatchDog::UpdataNow() { dog_now++; }

inline void WatchDog::UpdataLast() { dog_last = dog_now; }

// void WatchDog::UpdataLastLast() { dog_last_last = dog_last; }

inline void WatchDog::Reset() {
  if (dog_now > 1000) dog_now = 0;
}

bool WatchDog::DogIsOk(int error_times) {
  if (dog_last == dog_now)
    error_counts++;
  else
    error_counts = 0;
  if (error_counts > error_times) return false;
  UpdataLast();
  Reset();
  return true;
}

}  // namespace common
}  // namespace sweeper