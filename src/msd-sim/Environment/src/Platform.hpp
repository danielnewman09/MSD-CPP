#ifndef PLATFORM_HPP
#define PLATFORM_HPP

#include <cstdint>

class Platform
{
public:
    Platform();
    ~Platform();

    void update(); // Update the platform

private:
    //! State of the platform
    PlatformState state_;

    //! Agent controlling the platform
    Agent agent_;

    //! Sensor attached to the platform
    Sensor sensor_;

    //! Platform ID
    uint32_t id_;
};

#endif // PLATFORM_HPP