#ifndef WORLD_MODEL_HPP
#define WORLD_MODEL_HPP

#include <vector>
#include <chrono>

#include "msd-sim/Environment/src/Platform.hpp"

namespace msd_sim
{

    class WorldModel
    {
    public:
        WorldModel();
        ~WorldModel();

        void update(); // Update the world model

    private:
        //! Platforms in the world
        std::vector<Platform> platforms_;

        //! Current time in the world
        std::chrono::milliseconds time_;

        //! Time step for the world model
        std::chrono::milliseconds timeStep_;
    };

} // namespace msd_sim

#endif // WORLD_MODEL_HPP