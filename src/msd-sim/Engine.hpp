#ifndef MSD_ENGINE_HPP
#define MSD_ENGINE_HPP

#include "Environment/src/WorldModel.hpp"

namespace msd_sim
{

class Engine
{
public:

Engine();

  void run();

private:
  WorldModel worldModel_;
};

} // namespace msd_sim

#endif // MSD_ENGINE_HPP