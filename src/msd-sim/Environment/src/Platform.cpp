#include "Platform.hpp"
#include <iostream>

// Constructor
Platform::Platform(uint32_t id)
    : id_{id},
      lastUpdateTime_{std::chrono::milliseconds{0}}
{
  std::cout << "Platform " << id_ << " created.\n";
}

// Destructor
Platform::~Platform()
{
  std::cout << "Platform " << id_ << " destroyed.\n";
}

// Update method
void Platform::update(const std::chrono::milliseconds &currTime)
{
  auto elapsedTime = currTime - lastUpdateTime_;

  // Example: update platform's state based on elapsed time
  // For simplicity, let's just print the elapsed time
  std::cout << "Platform " << id_
            << " updated. Elapsed time since last update: "
            << elapsedTime.count() << " ms\n";

  // Update the last update time to current time
  lastUpdateTime_ = currTime;
}